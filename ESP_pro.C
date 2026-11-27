/*
  ESP32 - Painel de controle da transferencia entre 3 redes
  (Access Point + servidor web; a logica de protecao fica toda no Arduino)

  Protocolo com o Arduino (9600 baud):
   envia : GET | SET 1 | SET 0 | MANUT 1 | MANUT 2
   recebe: STATE <modo> <rede> <destino> <r1r2r3> <tent> <bloqP> <bloqR> <V1> <V2> <seg>
*/

#include <WiFi.h>
#include <WebServer.h>

// Configuracao do Access Point (troque antes de usar em campo)
const char* ssid = "Controle_Rele_ESP32";
const char* senha = "12345678";

// Codigo que o SOD digita para confirmar manutencao
const char* PIN_SOD = "1234";
const uint8_t PIN_MAX_ERROS = 5;            // erros seguidos antes de travar
const unsigned long PIN_TRAVA_MS = 60000UL; // tempo travado

// Comunicacao UART  (RX do ESP32 recebe 5 V do Arduino: use divisor!)
#define RXD2 16
#define TXD2 17
const unsigned long TIMEOUT_MS = 400;       // janela de 100 ms + resposta ~55 ms + folga
const unsigned long INTERVALO_GET_MS = 500;

WebServer server(80);

struct EstadoArduino {
  String modo = "DESLIGADO";
  String rede = "NENHUMA";
  String destino = "NENHUMA";
  bool rele[3] = {false, false, false};
  int tentativas = 0;
  bool bloq[2] = {false, false};      // 0 = principal, 1 = reserva
  float vrms[2] = {0, 0};
  int restante = 0;                   // segundos para a manobra
  bool online = false;
  unsigned long ultimaResposta = 0;
};

EstadoArduino est;
unsigned long alarmeId = 0;           // incrementa a cada novo bloqueio ou falha
unsigned long ultimaConsulta = 0;
bool aguardandoGet = false;
unsigned long tEnvioGet = 0;

uint8_t errosPin = 0;
unsigned long tTravaPin = 0;

char linhaRx[100];
uint8_t idxRx = 0;

// ---------------------------------------------------------------------
//  Interpretacao da resposta
// ---------------------------------------------------------------------
bool modoValido(const char* m) {
  static const char* const MODOS[] = {"DESLIGADO", "NORMAL", "TRIP", "TESTE",
                                      "ESPERA", "COMUTANDO", "FALHA"};
  for (const char* x : MODOS) if (strcmp(m, x) == 0) return true;
  return false;
}

bool redeValida(const char* r) {
  static const char* const REDES[] = {"PRINCIPAL", "RESERVA", "EMERGENCIA", "NENHUMA"};
  for (const char* x : REDES) if (strcmp(r, x) == 0) return true;
  return false;
}

bool interpretarResposta(const char* linha) {
  char buf[100];
  strncpy(buf, linha, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;

  char* tok[12];
  int n = 0;
  for (char* t = strtok(buf, " \r\n"); t && n < 12; t = strtok(NULL, " \r\n")) {
    tok[n++] = t;
  }

  if (n != 11 || strcmp(tok[0], "STATE") != 0) return false;
  if (!modoValido(tok[1]) || !redeValida(tok[2]) || !redeValida(tok[3])) return false;
  if (strlen(tok[4]) != 3) return false;

  bool novoBloqP = atoi(tok[6]) == 1;
  bool novoBloqR = atoi(tok[7]) == 1;
  bool novaFalha = strcmp(tok[1], "FALHA") == 0 && est.modo != "FALHA";

  // Novo bloqueio ou falha => novo alarme para o SOD
  if ((novoBloqP && !est.bloq[0]) || (novoBloqR && !est.bloq[1]) || novaFalha) {
    alarmeId++;
  }

  est.modo = tok[1];
  est.rede = tok[2];
  est.destino = tok[3];
  for (int i = 0; i < 3; i++) est.rele[i] = (tok[4][i] == '1');
  est.tentativas = atoi(tok[5]);
  est.bloq[0] = novoBloqP;
  est.bloq[1] = novoBloqR;
  est.vrms[0] = atof(tok[8]);
  est.vrms[1] = atof(tok[9]);
  est.restante = atoi(tok[10]);
  return true;
}

// Leitura nao bloqueante: retorna true quando uma linha STATE valida chegou
bool lerLinhaArduino() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      linhaRx[idxRx] = 0;
      idxRx = 0;
      if (interpretarResposta(linhaRx)) {
        est.online = true;
        est.ultimaResposta = millis();
        return true;
      }
    }
    else if (c != '\r' && idxRx < sizeof(linhaRx) - 1) {
      linhaRx[idxRx++] = c;
    }
  }
  return false;
}

// Envia um comando e espera a resposta (usado so nos comandos do usuario)
bool comandoArduino(const char* comando) {
  // Se ha um GET em andamento, espera a resposta dele para nao confundir
  unsigned long t0 = millis();
  while (aguardandoGet && millis() - t0 < TIMEOUT_MS) {
    if (lerLinhaArduino()) aguardandoGet = false;
    delay(1);
  }
  aguardandoGet = false;

  // Limpa restos antigos
  idxRx = 0;
  while (Serial2.available()) Serial2.read();

  Serial2.println(comando);

  t0 = millis();
  while (millis() - t0 < TIMEOUT_MS) {
    if (lerLinhaArduino()) return true;
    delay(1);
  }
  return false;
}

// ---------------------------------------------------------------------
//  JSON
// ---------------------------------------------------------------------
String jsonEstado() {
  String j = "{\"online\":";
  j += est.online ? "true" : "false";
  j += ",\"modo\":\"" + est.modo + "\"";
  j += ",\"rede\":\"" + est.rede + "\"";
  j += ",\"destino\":\"" + est.destino + "\"";
  j += ",\"reles\":[";
  for (int i = 0; i < 3; i++) {
    j += est.rele[i] ? "true" : "false";
    if (i < 2) j += ",";
  }
  j += "],\"tentativas\":" + String(est.tentativas);
  j += ",\"bloqP\":";
  j += est.bloq[0] ? "true" : "false";
  j += ",\"bloqR\":";
  j += est.bloq[1] ? "true" : "false";
  j += ",\"v1\":" + String(est.vrms[0], 2);
  j += ",\"v2\":" + String(est.vrms[1], 2);
  j += ",\"restante\":" + String(est.restante);
  j += ",\"alarme\":" + String(alarmeId);
  j += "}";
  return j;
}

// ---------------------------------------------------------------------
//  Pagina HTML
// ---------------------------------------------------------------------
const char pagina[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Transferencia de Redes</title>

<style>
body {
  font-family: Arial, sans-serif;
  background: #172033;
  color: white;
  text-align: center;
  padding: 25px 12px;
  margin: 0;
}

.card {
  background: #26344d;
  padding: 24px;
  border-radius: 18px;
  max-width: 380px;
  margin: auto;
}

h1 { font-size: 23px; margin: 0 0 10px; }

.badge {
  display: inline-block;
  padding: 8px 14px;
  border-radius: 20px;
  font-weight: bold;
  font-size: 15px;
  margin: 6px 0;
}

.verde { background: #16a34a; }
.amarelo { background: #d97706; }
.vermelho { background: #dc2626; }
.cinza { background: #64748b; }
.azul { background: #2563eb; }

.info {
  font-size: 14px;
  margin: 6px 0;
  color: #d5deec;
  min-height: 17px;
}

.rede {
  display: flex;
  align-items: center;
  justify-content: space-between;
  background: #1d2a40;
  border: 2px solid transparent;
  border-radius: 12px;
  padding: 12px 14px;
  margin: 10px 0;
  text-align: left;
}

.rede.ativa { border-color: #16a34a; }
.rede.pendente { border-color: #dc2626; }

.rede .nome { font-weight: bold; font-size: 16px; }
.rede .tensao { font-size: 13px; color: #b9c5d8; margin-top: 3px; }
.rede .tag {
  font-size: 12px;
  font-weight: bold;
  padding: 5px 9px;
  border-radius: 12px;
  white-space: nowrap;
}

button {
  border: none;
  border-radius: 12px;
  padding: 15px 30px;
  font-size: 18px;
  color: white;
  cursor: pointer;
  width: 100%;
  margin-top: 12px;
}

button:disabled { opacity: 0.45; }

.ligar { background: #16a34a; }
.desligar { background: #dc2626; }
.manut { background: #2563eb; font-size: 15px; padding: 13px 10px; }

#alarme {
  display: none;
  background: #7f1d1d;
  border: 2px solid #dc2626;
  border-radius: 12px;
  padding: 14px;
  margin: 16px 0 5px;
  animation: piscar 1s infinite;
}

@keyframes piscar { 50% { background: #b91c1c; } }

#status {
  font-size: 13px;
  color: #b9c5d8;
  margin-top: 18px;
}
</style>
</head>

<body>

<div class="card">

<h1>Transferencia de Redes</h1>

<div id="modo" class="badge cinza">Consultando...</div>
<div id="detalhe" class="info"></div>

<div id="rede0" class="rede">
  <div><div class="nome">1 - Principal</div><div class="tensao" id="v0">-- Vrms</div></div>
  <span class="tag cinza" id="tag0">--</span>
</div>
<div id="rede1" class="rede">
  <div><div class="nome">2 - Reserva</div><div class="tensao" id="v1">-- Vrms</div></div>
  <span class="tag cinza" id="tag1">--</span>
</div>
<div id="rede2" class="rede">
  <div><div class="nome">3 - Emergencial</div><div class="tensao">sem sensor</div></div>
  <span class="tag cinza" id="tag2">--</span>
</div>

<div id="alarme">
  <strong id="alarmeTitulo">ALARME</strong>
  <div id="alarmeTexto" class="info" style="color:white"></div>
  <button class="manut" id="btnManut1" onclick="manutencao(1)">
    Confirmar manutencao da rede PRINCIPAL
  </button>
  <button class="manut" id="btnManut2" onclick="manutencao(2)">
    Confirmar manutencao da rede RESERVA
  </button>
</div>

<button id="botao" onclick="alternar()">Aguarde...</button>

<div id="status">Conectando...</div>

</div>

<script>

const MAX_TENT = 3;  // igual ao MAX_TENTATIVAS do Arduino
const NOMES = { PRINCIPAL: 'principal', RESERVA: 'reserva', EMERGENCIA: 'emergencial', NENHUMA: '-' };
const IDX = { PRINCIPAL: 0, RESERVA: 1, EMERGENCIA: 2 };

const MODOS = {
  DESLIGADO: ['Desligado manualmente', 'cinza'],
  NORMAL:    ['Operacao normal', 'verde'],
  TRIP:      ['Trip: aguardando religamento', 'amarelo'],
  TESTE:     ['Religando e verificando a rede', 'amarelo'],
  ESPERA:    ['Aguardando tempo de seguranca', 'azul'],
  COMUTANDO: ['Manobrando (todos abertos)', 'azul'],
  FALHA:     ['FALHA DE INTERTRAVAMENTO', 'vermelho']
};

let dados = null;
let ultimoAlarme = -1;
let audioCtx = null;
let ocupado = false;

function beep() {
  try {
    audioCtx = audioCtx || new (window.AudioContext || window.webkitAudioContext)();
    const o = audioCtx.createOscillator();
    const g = audioCtx.createGain();
    o.connect(g);
    g.connect(audioCtx.destination);
    o.frequency.value = 880;
    g.gain.value = 0.2;
    o.start();
    o.stop(audioCtx.currentTime + 0.6);
  } catch (e) {}
}

function dispararAlarme(texto) {
  if (navigator.vibrate) navigator.vibrate([500, 200, 500, 200, 500]);
  beep();
  try {
    if (window.Notification && Notification.permission === 'granted') {
      new Notification('SOD: manutencao necessaria', { body: texto });
    }
  } catch (e) {}
}

// Primeiro toque: libera audio e pede permissao de notificacao
document.addEventListener('click', () => {
  try {
    audioCtx = audioCtx || new (window.AudioContext || window.webkitAudioContext)();
    if (window.Notification && Notification.permission === 'default') {
      Notification.requestPermission();
    }
  } catch (e) {}
}, { once: true });

function tag(i, texto, cor) {
  const t = document.getElementById('tag' + i);
  t.innerText = texto;
  t.className = 'tag ' + cor;
}

function aplicar(d) {
  dados = d;

  const info = MODOS[d.modo] || [d.modo, 'cinza'];
  const badge = document.getElementById('modo');
  badge.innerText = info[0];
  badge.className = 'badge ' + info[1];

  // Linha de detalhe
  let det = '';
  if (d.modo === 'ESPERA') {
    det = 'Manobra para a rede ' + NOMES[d.destino] + ' em ' + d.restante + ' s';
  } else if (d.modo === 'COMUTANDO') {
    det = 'Fechando a rede ' + NOMES[d.destino];
  } else if (d.modo === 'TRIP' || d.modo === 'TESTE') {
    det = 'Rede ' + NOMES[d.rede] + ' | religamentos sem sucesso: ' +
          d.tentativas + ' de ' + MAX_TENT;
  } else if (d.modo === 'NORMAL') {
    det = 'Alimentando pela rede ' + NOMES[d.rede];
  } else if (d.modo === 'FALHA') {
    det = 'Todos os reles abertos. Verifique a ligacao.';
  }
  document.getElementById('detalhe').innerText = det;

  // Tensoes
  document.getElementById('v0').innerText = d.v1.toFixed(2) + ' Vrms';
  document.getElementById('v1').innerText = d.v2.toFixed(2) + ' Vrms';

  // Situacao de cada rede
  const pend = [d.bloqP, d.bloqR, false];
  for (let i = 0; i < 3; i++) {
    const box = document.getElementById('rede' + i);
    const emUso = d.reles[i];
    const selecionada = IDX[d.rede] === i;
    box.className = 'rede' + (emUso ? ' ativa' : '') + (pend[i] ? ' pendente' : '');

    if (pend[i])                         tag(i, 'MANUTENCAO', 'vermelho');
    else if (emUso)                      tag(i, 'EM USO', 'verde');
    else if (selecionada && d.modo === 'TRIP') tag(i, 'TRIP', 'amarelo');
    else if (IDX[d.destino] === i)       tag(i, 'PROXIMA', 'azul');
    else                                 tag(i, i === 2 ? 'RESERVADA' : 'DISPONIVEL', 'cinza');
  }

  // Botao principal
  const desligado = d.modo === 'DESLIGADO';
  const falha = d.modo === 'FALHA';
  const botao = document.getElementById('botao');
  botao.innerText = desligado ? 'LIGAR' : (falha ? 'RECONHECER FALHA (DESLIGAR)' : 'DESLIGAR');
  botao.className = desligado ? 'ligar' : 'desligar';
  if (!ocupado) botao.disabled = false;

  // Alarme
  const algumBloq = d.bloqP || d.bloqR;
  const mostrarAlarme = algumBloq || falha;
  document.getElementById('alarme').style.display = mostrarAlarme ? 'block' : 'none';
  document.getElementById('btnManut1').style.display = d.bloqP ? 'block' : 'none';
  document.getElementById('btnManut2').style.display = d.bloqR ? 'block' : 'none';

  let titulo = 'ALARME';
  let texto = '';
  if (falha) {
    titulo = 'FALHA DE INTERTRAVAMENTO';
    texto = 'Rele fechado fora da sequencia. Todos foram abertos.';
  } else if (d.bloqP && d.bloqR) {
    titulo = 'PRINCIPAL E RESERVA FORA DO NOMINAL';
    texto = 'Operando pela rede emergencial. Volta a principal somente apos a manutencao das duas redes.';
  } else if (d.bloqP) {
    titulo = 'REDE PRINCIPAL FORA DO NOMINAL';
    texto = 'Bloqueada apos ' + MAX_TENT + ' religamentos. ' +
            (d.rede === 'EMERGENCIA' ? 'Operando pela emergencial.' : 'Operando pela reserva.');
  } else if (d.bloqR) {
    titulo = 'REDE RESERVA FORA DO NOMINAL';
    texto = 'Bloqueada apos ' + MAX_TENT + ' religamentos. Manutencao necessaria.';
  }
  document.getElementById('alarmeTitulo').innerText = titulo;
  document.getElementById('alarmeTexto').innerText = texto;
  document.title = mostrarAlarme ? 'ALARME - Redes' : 'Transferencia de Redes';

  if (mostrarAlarme && d.alarme !== ultimoAlarme) {
    ultimoAlarme = d.alarme;
    dispararAlarme(titulo);
  }
}

async function atualizar() {
  if (ocupado) return;
  try {
    const resposta = await fetch('/api/status');
    if (!resposta.ok) throw new Error();

    const d = await resposta.json();

    if (!d.online) {
      document.getElementById('status').innerText =
        'Falha na comunicacao com o Arduino';
      document.getElementById('botao').disabled = true;
      return;
    }

    aplicar(d);
    document.getElementById('status').innerText =
      'Arduino conectado | Atualizado';

  } catch (erro) {
    document.getElementById('status').innerText =
      'Falha na comunicacao com o ESP32';
  }
}

async function enviar(url, corpo) {
  return fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
    body: new URLSearchParams(corpo)
  });
}

async function alternar() {
  if (!dados || ocupado) return;

  const proximoEstado = (dados.modo === 'DESLIGADO') ? 1 : 0;
  if (proximoEstado === 0 &&
      !confirm('Desligar a alimentacao (todos os reles serao abertos)?')) return;

  const botao = document.getElementById('botao');
  botao.disabled = true;
  ocupado = true;

  try {
    const resposta = await enviar('/api/relay', { state: proximoEstado });
    if (!resposta.ok) throw new Error();
    ocupado = false;
    aplicar(await resposta.json());

  } catch (erro) {
    document.getElementById('status').innerText = 'Erro ao enviar comando';

  } finally {
    ocupado = false;
    botao.disabled = false;
  }
}

async function manutencao(rede) {
  const nome = rede === 1 ? 'PRINCIPAL' : 'RESERVA';
  const pin = prompt('Codigo de autorizacao do SOD:');
  if (pin === null) return;

  if (!confirm('Confirma que a rede ' + nome + ' foi manutenida?')) return;

  ocupado = true;
  try {
    const resposta = await enviar('/api/manutencao', { rede: rede, pin: pin });

    if (resposta.status === 403) {
      document.getElementById('status').innerText = 'Codigo incorreto';
      return;
    }
    if (resposta.status === 429) {
      document.getElementById('status').innerText = 'Muitas tentativas. Aguarde 1 minuto.';
      return;
    }
    if (!resposta.ok) throw new Error();

    ocupado = false;
    aplicar(await resposta.json());
    document.getElementById('status').innerText =
      'Manutencao da rede ' + nome + ' confirmada';

  } catch (erro) {
    document.getElementById('status').innerText =
      'Erro ao confirmar manutencao';
  } finally {
    ocupado = false;
  }
}

atualizar();
setInterval(atualizar, 1000);

</script>
</body>
</html>
)rawliteral";

// ---------------------------------------------------------------------
//  Rotas
// ---------------------------------------------------------------------
void paginaPrincipal() {
  server.send_P(200, "text/html", pagina);
}

// Estado atual (cache atualizado a cada 500 ms)
void consultarEstado() {
  server.send(200, "application/json", jsonEstado());
}

// Liga / desliga manualmente
void controlarRele() {
  String valor = server.arg("state");

  if (valor != "0" && valor != "1") {
    server.send(400, "text/plain", "Comando invalido");
    return;
  }

  if (comandoArduino(valor == "1" ? "SET 1" : "SET 0")) {
    server.send(200, "application/json", jsonEstado());
  } else {
    server.send(503, "application/json", "{\"erro\":\"Arduino sem resposta\"}");
  }
}

// SOD confirma manutencao da rede 1 (principal) ou 2 (reserva)
void confirmarManutencao() {

  if (errosPin >= PIN_MAX_ERROS) {
    if (millis() - tTravaPin < PIN_TRAVA_MS) {
      server.send(429, "application/json", "{\"erro\":\"Muitas tentativas\"}");
      return;
    }
    errosPin = 0;
  }

  if (strlen(PIN_SOD) > 0 && server.arg("pin") != PIN_SOD) {
    if (++errosPin >= PIN_MAX_ERROS) tTravaPin = millis();
    server.send(403, "application/json", "{\"erro\":\"Codigo invalido\"}");
    return;
  }
  errosPin = 0;

  String rede = server.arg("rede");
  if (rede != "1" && rede != "2") {
    server.send(400, "application/json", "{\"erro\":\"Rede invalida\"}");
    return;
  }

  int idx = rede.toInt() - 1;
  if (!est.bloq[idx]) {
    server.send(409, "application/json", "{\"erro\":\"Rede nao esta bloqueada\"}");
    return;
  }

  if (comandoArduino(rede == "1" ? "MANUT 1" : "MANUT 2")) {
    server.send(200, "application/json", jsonEstado());
  } else {
    server.send(503, "application/json", "{\"erro\":\"Arduino sem resposta\"}");
  }
}

void setup() {
  Serial.begin(115200);

  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, senha);

  Serial.println("Access Point iniciado!");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, paginaPrincipal);
  server.on("/api/status", HTTP_GET, consultarEstado);
  server.on("/api/relay", HTTP_POST, controlarRele);
  server.on("/api/manutencao", HTTP_POST, confirmarManutencao);

  server.begin();
  Serial.println("Servidor HTTP iniciado!");
}

void loop() {
  server.handleClient();

  // Resposta do GET chega sem travar o servidor
  if (lerLinhaArduino()) aguardandoGet = false;
  if (aguardandoGet && millis() - tEnvioGet > TIMEOUT_MS) aguardandoGet = false;

  // Consulta o Arduino a cada 500 ms
  if (!aguardandoGet && millis() - ultimaConsulta >= INTERVALO_GET_MS) {
    ultimaConsulta = millis();
    Serial2.println("GET");
    aguardandoGet = true;
    tEnvioGet = millis();
  }

  if (millis() - est.ultimaResposta > 3000) est.online = false;
}
