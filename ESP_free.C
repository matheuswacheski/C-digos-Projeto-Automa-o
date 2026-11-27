#include <WiFi.h>
#include <WebServer.h>

// Configuracao do Access Point
const char* ssid = "Controle_Rele_ESP32";
const char* senha = "12345678";

// Codigo que o SOD digita para confirmar a manutencao de uma rede
const char* PIN_SOD = "1234";

// Comunicacao UART
#define RXD2 16
#define TXD2 17

WebServer server(80);

struct EstadoArduino {
  String modo = "DESLIGADO";
  int fonte = 0;          // rele fechado (0 = nenhum)
  int alvo = 0;           // rede em trip saver / teste / manobra
  int tentativas = 0;
  bool falhaP = false;    // rede principal aguardando manutencao
  bool falhaR = false;    // rede reserva aguardando manutencao
  float v1 = 0;
  float v2 = 0;
  bool online = false;
  unsigned long ultimaResposta = 0;
};

EstadoArduino est;
unsigned long alarmeId = 0;          // incrementa a cada nova falha de rede
unsigned long ultimaConsulta = 0;

// Interpreta "STATE <modo> <fonte> <alvo> <tent> <falhaP> <falhaR> <v1> <v2>"
bool interpretarResposta(String r) {
  r.trim();
  if (!r.startsWith("STATE ")) return false;

  char modoBuf[16];
  int fonte, alvo, tent, fp, fr;
  float v1, v2;

  if (sscanf(r.c_str(), "STATE %15s %d %d %d %d %d %f %f",
             modoBuf, &fonte, &alvo, &tent, &fp, &fr, &v1, &v2) != 8) {
    return false;
  }

  // Nova rede em falha => novo alarme para o SOD
  if ((fp == 1 && !est.falhaP) || (fr == 1 && !est.falhaR)) {
    alarmeId++;
  }

  est.modo = String(modoBuf);
  est.fonte = fonte;
  est.alvo = alvo;
  est.tentativas = tent;
  est.falhaP = (fp == 1);
  est.falhaR = (fr == 1);
  est.v1 = v1;
  est.v2 = v2;
  return true;
}

// Envia comando ao Arduino e aguarda resposta
bool comunicarArduino(const char* comando) {

  // Limpa respostas antigas
  while (Serial2.available()) {
    Serial2.read();
  }

  Serial2.println(comando);

  String resposta = Serial2.readStringUntil('\n');

  if (interpretarResposta(resposta)) {
    est.online = true;
    est.ultimaResposta = millis();
    return true;
  }

  if (millis() - est.ultimaResposta > 3000) {
    est.online = false;
  }
  return false;
}

String jsonEstado() {
  String j = "{\"online\":";
  j += est.online ? "true" : "false";
  j += ",\"modo\":\"" + est.modo + "\"";
  j += ",\"fonte\":" + String(est.fonte);
  j += ",\"alvo\":" + String(est.alvo);
  j += ",\"tent\":" + String(est.tentativas);
  j += ",\"fp\":";
  j += est.falhaP ? "true" : "false";
  j += ",\"fr\":";
  j += est.falhaR ? "true" : "false";
  j += ",\"v1\":" + String(est.v1, 2);
  j += ",\"v2\":" + String(est.v2, 2);
  j += ",\"alarme\":" + String(alarmeId);
  j += "}";
  return j;
}

// Pagina HTML
const char pagina[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Controle de Redes</title>

<style>
body {
  font-family: Arial, sans-serif;
  background: #172033;
  color: white;
  text-align: center;
  padding: 30px 15px;
}

.card {
  background: #26344d;
  padding: 25px;
  border-radius: 18px;
  max-width: 380px;
  margin: auto;
}

h1 { font-size: 25px; margin-top: 0; }

.badge {
  display: inline-block;
  padding: 8px 14px;
  border-radius: 20px;
  font-weight: bold;
  font-size: 15px;
  margin: 8px 0;
}

.verde { background: #16a34a; }
.amarelo { background: #d97706; }
.vermelho { background: #dc2626; }
.cinza { background: #64748b; }

.info {
  font-size: 15px;
  margin: 8px 0;
  color: #d5deec;
}

.rede {
  display: flex;
  justify-content: space-between;
  align-items: center;
  background: #1c2a40;
  border-radius: 12px;
  padding: 12px;
  margin: 10px 0;
  text-align: left;
}

.rede .nome { font-weight: bold; font-size: 15px; }
.rede .tensao { font-size: 13px; color: #b9c5d8; margin-top: 3px; }
.rede .sel {
  padding: 6px 10px;
  border-radius: 14px;
  font-size: 12px;
  font-weight: bold;
}

button {
  border: none;
  border-radius: 12px;
  padding: 16px 35px;
  font-size: 19px;
  color: white;
  cursor: pointer;
  width: 100%;
  margin-top: 15px;
}

button:disabled { opacity: 0.45; }

.ligar { background: #16a34a; }
.desligar { background: #dc2626; }
.manut { background: #2563eb; font-size: 15px; padding: 12px; margin-top: 10px; }

.alarme {
  background: #7f1d1d;
  border: 2px solid #dc2626;
  border-radius: 12px;
  padding: 14px;
  margin: 15px 0 5px;
  font-size: 14px;
  animation: piscar 1s infinite;
}

@keyframes piscar { 50% { background: #b91c1c; } }

#status {
  font-size: 13px;
  color: #b9c5d8;
  margin-top: 20px;
}
</style>
</head>

<body>

<div class="card">

<h1>Controle de Redes</h1>

<div id="modo" class="badge cinza">Consultando...</div>
<div id="tent" class="info"></div>

<div id="redes"></div>
<div id="alarmes"></div>
<div id="aviso" class="info"></div>

<button id="botao" onclick="alternar()">Aguarde...</button>

<div id="status">Conectando...</div>

</div>

<script>

const MAX_TENT = 3;  // igual ao MAX_TENTATIVAS do Arduino

const NOMES = {1: 'Rede principal', 2: 'Rede reserva', 3: 'Rede emergencial'};

const MODOS = {
  DESLIGADO: ['Desligado manualmente', 'cinza'],
  NORMAL: ['Operacao normal', 'verde'],
  TRIP: ['Trip saver: aguardando religamento', 'amarelo'],
  TESTE: ['Verificando a rede', 'amarelo'],
  MANOBRA: ['Manobra: aguardando tempo de seguranca', 'amarelo']
};

let dados = null;
let ultimoAlarme = -1;
let audioCtx = null;

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

function dispararAlarme() {
  if (navigator.vibrate) navigator.vibrate([500, 200, 500, 200, 500]);
  beep();
  try {
    if (window.Notification && Notification.permission === 'granted') {
      new Notification('SOD: rede indisponivel', {
        body: 'Uma rede precisa de manutencao.'
      });
    }
  } catch (e) {}
}

// Pede permissao de notificacao no primeiro toque na tela
document.addEventListener('click', () => {
  try {
    if (window.Notification && Notification.permission === 'default') {
      Notification.requestPermission();
    }
  } catch (e) {}
}, { once: true });

function estadoRede(n, d) {
  if (d.alvo === n && d.modo === 'TRIP') return ['TRIP SAVER', 'amarelo'];
  if (d.alvo === n && d.modo === 'TESTE') return ['VERIFICANDO', 'amarelo'];
  if (d.alvo === n && d.modo === 'MANOBRA') return ['MANOBRANDO', 'amarelo'];
  if (d.fonte === n) return ['EM USO', 'verde'];
  if (n === 1 && d.fp) return ['EM FALHA', 'vermelho'];
  if (n === 2 && d.fr) return ['EM FALHA', 'vermelho'];
  return [n === 3 ? 'STAND-BY' : 'DISPONIVEL', 'cinza'];
}

function tensaoRede(n, d) {
  if (n === 1) return d.v1.toFixed(2) + ' Vrms';
  if (n === 2) return d.v2.toFixed(2) + ' Vrms';
  return 'sem sensor';
}

function aplicar(d) {
  dados = d;

  const info = MODOS[d.modo] || [d.modo, 'cinza'];
  const badge = document.getElementById('modo');
  badge.innerText = info[0];
  badge.className = 'badge ' + info[1];

  const mostrarTent = ['TRIP', 'TESTE'].includes(d.modo) && d.tent > 0;
  document.getElementById('tent').innerText = mostrarTent
    ? 'Religamentos sem sucesso: ' + d.tent + ' de ' + MAX_TENT
    : '';

  // Cartoes das tres redes
  let html = '';
  for (let n = 1; n <= 3; n++) {
    const e = estadoRede(n, d);
    html += '<div class="rede"><div>' +
            '<div class="nome">' + NOMES[n] + '</div>' +
            '<div class="tensao">' + tensaoRede(n, d) + '</div></div>' +
            '<div class="sel ' + e[1] + '">' + e[0] + '</div></div>';
  }
  document.getElementById('redes').innerHTML = html;

  // Alarmes: uma caixa por rede que aguarda manutencao
  let alm = '';
  if (d.fp) {
    alm += '<div class="alarme"><strong>ALARME: REDE PRINCIPAL INDISPONIVEL</strong><br>' +
           'Trip saver esgotado. Notificacao enviada ao SOD.' +
           '<button class="manut" onclick="manutencao(1)">Confirmar manutencao da rede principal</button></div>';
  }
  if (d.fr) {
    alm += '<div class="alarme"><strong>ALARME: REDE RESERVA INDISPONIVEL</strong><br>' +
           'Trip saver esgotado. Notificacao enviada ao SOD.' +
           '<button class="manut" onclick="manutencao(2)">Confirmar manutencao da rede reserva</button></div>';
  }
  document.getElementById('alarmes').innerHTML = alm;

  let aviso = '';
  if (d.fonte === 3) {
    aviso = 'Carga alimentada pela rede emergencial. O retorno a rede principal exige a manutencao da principal e da reserva.';
  } else if (d.fonte === 2 && d.fp) {
    aviso = 'Carga alimentada pela rede reserva.';
  }
  document.getElementById('aviso').innerText = aviso;

  const desligado = d.modo === 'DESLIGADO';
  const botao = document.getElementById('botao');
  botao.innerText = desligado ? 'LIGAR' : 'DESLIGAR TUDO';
  botao.className = desligado ? 'ligar' : 'desligar';

  const emFalha = d.fp || d.fr;
  document.title = emFalha ? 'ALARME - Redes' : 'Controle de Redes';

  if (d.alarme > 0 && d.alarme !== ultimoAlarme) {
    ultimoAlarme = d.alarme;
    dispararAlarme();
  }
}

async function atualizar() {
  try {
    const resposta = await fetch('/api/status');
    if (!resposta.ok) throw new Error();

    const d = await resposta.json();

    if (!d.online) {
      document.getElementById('status').innerText =
        'Falha na comunicacao com o Arduino';
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

async function alternar() {
  if (!dados) return;

  const proximoEstado = (dados.modo === 'DESLIGADO') ? 1 : 0;
  const botao = document.getElementById('botao');
  botao.disabled = true;

  try {
    const resposta = await fetch('/api/relay?state=' + proximoEstado);
    if (!resposta.ok) throw new Error();
    aplicar(await resposta.json());

  } catch (erro) {
    document.getElementById('status').innerText = 'Erro ao enviar comando';

  } finally {
    botao.disabled = false;
  }
}

async function manutencao(rede) {
  const pin = prompt('Codigo de autorizacao do SOD:');
  if (pin === null) return;

  if (!confirm('Confirma que a ' + NOMES[rede].toLowerCase() + ' foi manutenida?')) {
    return;
  }

  try {
    const resposta = await fetch(
      '/api/manutencao?rede=' + rede + '&pin=' + encodeURIComponent(pin)
    );

    if (resposta.status === 403) {
      document.getElementById('status').innerText = 'Codigo incorreto';
      return;
    }
    if (!resposta.ok) throw new Error();

    aplicar(await resposta.json());
    document.getElementById('status').innerText =
      'Manutencao confirmada | Aguardando manobra de seguranca';

  } catch (erro) {
    document.getElementById('status').innerText =
      'Erro ao confirmar manutencao';
  }
}

atualizar();
setInterval(atualizar, 1000);

</script>
</body>
</html>
)rawliteral";

// Pagina principal
void paginaPrincipal() {
  server.send_P(200, "text/html", pagina);
}

// Estado atual (vem do cache, atualizado a cada 500 ms)
void consultarEstado() {
  server.send(200, "application/json", jsonEstado());
}

// Liga/desliga manualmente (liga na melhor rede disponivel)
void controlarRele() {

  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "Parametro ausente");
    return;
  }

  String valor = server.arg("state");

  if (valor != "0" && valor != "1") {
    server.send(400, "text/plain", "Comando invalido");
    return;
  }

  bool sucesso = comunicarArduino(valor == "1" ? "SET 1" : "SET 0");

  if (sucesso) {
    server.send(200, "application/json", jsonEstado());
  } else {
    server.send(503, "application/json",
      "{\"erro\":\"Arduino sem resposta\"}");
  }
}

// SOD confirma que uma rede foi manutenida (rede=1 principal, rede=2 reserva)
void confirmarManutencao() {

  if (strlen(PIN_SOD) > 0 && server.arg("pin") != PIN_SOD) {
    server.send(403, "application/json", "{\"erro\":\"Codigo invalido\"}");
    return;
  }

  String rede = server.arg("rede");

  if (rede != "1" && rede != "2") {
    server.send(400, "text/plain", "Rede invalida");
    return;
  }

  bool pendente = (rede == "1") ? est.falhaP : est.falhaR;

  if (!pendente) {
    server.send(409, "application/json",
      "{\"erro\":\"Rede nao esta aguardando manutencao\"}");
    return;
  }

  bool sucesso = comunicarArduino(rede == "1" ? "MANUT 1" : "MANUT 2");

  if (sucesso) {
    server.send(200, "application/json", jsonEstado());
  } else {
    server.send(503, "application/json",
      "{\"erro\":\"Arduino sem resposta\"}");
  }
}

void setup() {

  Serial.begin(115200);

  // UART
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.setTimeout(300);

  // Cria rede Wi-Fi
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, senha);

  Serial.println("Access Point iniciado!");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  // Rotas HTTP
  server.on("/", paginaPrincipal);
  server.on("/api/status", consultarEstado);
  server.on("/api/relay", controlarRele);
  server.on("/api/manutencao", confirmarManutencao);

  server.begin();

  Serial.println("Servidor HTTP iniciado!");
}

void loop() {
  server.handleClient();

  // Consulta o Arduino a cada 500 ms para manter o estado atualizado
  // e detectar as falhas mesmo sem comando do usuario
  if (millis() - ultimaConsulta >= 500) {
    ultimaConsulta = millis();
    comunicarArduino("GET");
  }
}
