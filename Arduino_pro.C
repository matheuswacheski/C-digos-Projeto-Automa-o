/*
  Trip saver + transferencia automatica entre 3 redes

  Rele 1 (pino 7) = rede PRINCIPAL   -> sensor 1 (A0)
  Rele 2 (pino 8) = rede RESERVA     -> sensor 2 (A1)
  Rele 3 (pino 9) = rede EMERGENCIAL -> sem sensor

  Sequencia:
   1. Principal falha -> trip saver (3 religamentos). Sem sucesso: bloqueia a
      principal, espera T_MANOBRA_MS e manobra para a reserva.
   2. SOD confirma manutencao da principal (MANUT 1) -> espera T_MANOBRA_MS,
      abre a reserva, tempo morto e fecha a principal.
   3. Reserva falha -> trip saver. Sem sucesso: bloqueia a reserva, espera
      T_MANOBRA_MS e manobra para a emergencial.
   4. Na emergencial so volta para a principal quando as DUAS redes tiverem
      manutencao confirmada (MANUT 1 e MANUT 2).

  Intertravamento logico:
   - fecharRele() e o UNICO ponto do codigo que fecha um rele, e so fecha se
     todos os outros estiverem abertos;
   - entre abrir uma rede e fechar OUTRA ha sempre o tempo morto T_MORTO_MS;
   - a cada ciclo verificarIntertravamento() le o estado real dos pinos; se
     houver rele fechado fora da rede ativa, abre tudo e entra em FALHA.
   Recomenda-se tambem intertravamento fisico (ver comentario no final).

  Protocolo (9600 baud, linhas terminadas em \n):
   ESP32 -> Arduino: GET | SET 1 | SET 0 | MANUT 1 | MANUT 2
   Arduino -> ESP32:
   STATE <modo> <rede_ativa> <destino> <r1r2r3> <tentativas> <bloqP> <bloqR> <V1> <V2> <seg_restantes>
*/

#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <avr/wdt.h>

SoftwareSerial comunicacao(10, 11);
// RX = 10 | TX = 11  (TX 5 V -> divisor resistivo -> GPIO16 do ESP32)

// ---------- Redes e reles ----------
enum Rede : uint8_t { PRINCIPAL, RESERVA, EMERGENCIA, NENHUMA };
const char* const NOMES_REDE[] = {"PRINCIPAL", "RESERVA", "EMERGENCIA", "NENHUMA"};

const uint8_t N_RELES = 3;
const uint8_t PINO_RELE[N_RELES] = {7, 8, 9};
// Muitos modulos rele sao acionados em nivel LOW.
#define RELE_LIGADO LOW
#define RELE_DESLIGADO HIGH

// ---------- Medicao de tensao ----------
const uint8_t N_SENSORES = 2;                         // principal e reserva
const uint8_t PINO_V[N_SENSORES] = {A0, A1};
const float GANHO[N_SENSORES] = {11.0, 11.0};          // (R1 + R2) / R2 de cada condicionador
const float CALIBRACAO[N_SENSORES] = {1.00, 1.00};     // Vrms multimetro / Vrms Arduino
const float BANDGAP_V = 1.100;                         // referencia interna (calibrar por chip)
const unsigned long JANELA_US = 100000UL;              // 100 ms = 5 ciclos (50 Hz) ou 6 (60 Hz)

// ---------- Trip saver (iguais a versao anterior) ----------
const float V_MIN = 5.5;                       // limite inferior aceitavel (Vrms)
const float V_MAX = 7.5;                       // limite superior aceitavel (Vrms)
const float V_FOLGA_RETORNO = 0.2;             // histerese para confirmar retorno
const uint8_t JANELAS_PARA_TRIP = 5;           // 5 janelas seguidas fora = problema
const unsigned long T_TRIP_MS = 3000UL;        // tempo com o rele aberto
const unsigned long T_ASSENTAMENTO_MS = 1000UL;// ignora a partida apos fechar
const uint8_t JANELAS_CONFIRMA = 10;           // 10 janelas seguidas ok = rede ok
const unsigned long T_TESTE_MAX_MS = 4000UL;   // tempo maximo do teste de retorno
const uint8_t MAX_TENTATIVAS = 3;              // religamentos antes de bloquear

// ---------- Manobra ----------
const unsigned long T_MANOBRA_MS = 5000UL;     // tempo de seguranca antes de manobrar
const unsigned long T_MORTO_MS = 500UL;        // todos abertos entre abrir uma rede e fechar outra
// false: da emergencial so sai quando principal E reserva forem manutenidas (especificacao).
// true : sai da emergencial para a reserva se so a reserva for manutenida.
const bool EMERG_VOLTA_RESERVA = false;

// ---------- Estado persistente ----------
const int EE_BLOQ[N_SENSORES] = {0, 1};        // endereco 0 = mesmo da versao anterior
const uint8_t BLOQ_MAGICO = 0xB1;

enum Modo : uint8_t { DESLIGADO, NORMAL, TRIP, TESTE, ESPERA, COMUTANDO, FALHA };
const char* const NOMES_MODO[] = {"DESLIGADO", "NORMAL", "TRIP", "TESTE",
                                  "ESPERA", "COMUTANDO", "FALHA"};

Modo modo = DESLIGADO;
Rede ativa = NENHUMA;        // rede selecionada (o rele pode estar aberto em TRIP)
Rede destino = NENHUMA;      // rede para a qual se vai manobrar (ESPERA / COMUTANDO)
bool bloq[N_SENSORES] = {false, false};   // rede aguardando manutencao

uint8_t tentativas = 0;
uint8_t janelasFora = 0;
uint8_t janelasOk = 0;
unsigned long tMarca = 0;

// Intertravamento
Rede ultimaFechada = NENHUMA;
unsigned long tAbertura = 0;  // instante em que o ultimo rele fechado foi aberto

// Ultima janela valida
float vrms[N_SENSORES] = {0, 0};
float vccAtual = 5.0;
uint16_t nAmostras = 0;

char bufferCmd[16];
uint8_t idxCmd = 0;

// Prototipos
void entrarFalha(const __FlashStringHelper* msg);
void iniciarManobra(Rede alvo);

// =====================================================================
//  Registro
// =====================================================================
void registrar(const __FlashStringHelper* msg) {
  Serial.print(msg);
  Serial.print(F(" | "));
  Serial.print(NOMES_MODO[modo]);
  Serial.print(F(" rede="));
  Serial.print(NOMES_REDE[ativa]);
  Serial.print(F(" V1="));
  Serial.print(vrms[0], 2);
  Serial.print(F(" V2="));
  Serial.print(vrms[1], 2);
  Serial.print(F(" Vcc="));
  Serial.println(vccAtual, 2);
}

// =====================================================================
//  Reles e intertravamento
// =====================================================================
bool releFechado(uint8_t i) {
  // Em pino OUTPUT o digitalRead le o nivel real do pino
  return digitalRead(PINO_RELE[i]) == RELE_LIGADO;
}

uint8_t relesFechados() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < N_RELES; i++) {
    if (releFechado(i)) n++;
  }
  return n;
}

void abrirTodos() {
  if (relesFechados() > 0) tAbertura = millis();
  for (uint8_t i = 0; i < N_RELES; i++) {
    digitalWrite(PINO_RELE[i], RELE_DESLIGADO);
  }
}

// UNICO ponto do codigo que fecha um rele.
bool fecharRele(Rede r) {
  if (r >= N_RELES) return false;

  // Intertravamento: so fecha com todos os reles abertos
  if (relesFechados() > 0) {
    entrarFalha(F("INTERTRAVAMENTO: fechamento com outro rele fechado"));
    return false;
  }

  // Tempo morto ao trocar de rede
  if (r != ultimaFechada && millis() - tAbertura < T_MORTO_MS) {
    return false;
  }

  digitalWrite(PINO_RELE[r], RELE_LIGADO);
  ultimaFechada = r;
  return true;
}

void entrarFalha(const __FlashStringHelper* msg) {
  abrirTodos();
  modo = FALHA;
  ativa = NENHUMA;
  destino = NENHUMA;
  registrar(msg);
}

// Confere o estado real dos pinos a cada ciclo
void verificarIntertravamento() {
  if (modo == FALHA) return;
  for (uint8_t i = 0; i < N_RELES; i++) {
    if (releFechado(i) && i != ativa) {
      entrarFalha(F("INTERTRAVAMENTO: rele fechado fora da rede ativa"));
      return;
    }
  }
}

// =====================================================================
//  Medicao
// =====================================================================
bool dentroDaFaixa(float v, float folga) {
  return (v >= V_MIN + folga) && (v <= V_MAX - folga);
}

// Mede o Vcc real pela referencia interna de 1,1 V
float lerVcc() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);

  ADCSRA |= _BV(ADSC);                        // primeira conversao descartada
  while (bit_is_set(ADCSRA, ADSC));

  uint32_t soma = 0;
  for (uint8_t i = 0; i < 8; i++) {
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    soma += ADC;
  }

  analogRead(PINO_V[0]);                      // volta o mux ao A0 (leitura descartada)
  return BANDGAP_V * 1024.0 / (soma / 8.0);
}

// Mede os dois sensores na mesma janela, alternando A0/A1.
// Retorna false se a janela foi contaminada pela comunicacao serial.
bool medirJanela() {
  int antes = comunicacao.available();
  float vcc = lerVcc();

  int32_t soma[N_SENSORES] = {0, 0};
  uint32_t somaQuad[N_SENSORES] = {0, 0};
  uint16_t n = 0;

  unsigned long inicio = micros();
  while (micros() - inicio < JANELA_US) {
    for (uint8_t c = 0; c < N_SENSORES; c++) {
      int16_t x = (int16_t)analogRead(PINO_V[c]) - 512;   // centraliza: melhora precisao
      soma[c] += x;
      somaQuad[c] += (uint32_t)((int32_t)x * x);
    }
    n++;
  }

  if (comunicacao.available() != antes) return false;
  if (n == 0) return false;

  for (uint8_t c = 0; c < N_SENSORES; c++) {
    float media = (float)soma[c] / n;
    float variancia = (float)somaQuad[c] / n - media * media;
    if (variancia < 0) variancia = 0;
    vrms[c] = sqrt(variancia) * (vcc / 1024.0) * GANHO[c] * CALIBRACAO[c];
  }
  vccAtual = vcc;
  nAmostras = n;
  return true;
}

// =====================================================================
//  Logica de transferencia
// =====================================================================

// Rede em que o sistema deve estar, dado o estado das manutencoes
Rede redeAlvo(Rede atual) {
  bool okP = !bloq[PRINCIPAL];
  bool okR = !bloq[RESERVA];

  if (atual == EMERGENCIA) {
    if (okP && okR) return PRINCIPAL;           // so volta com as duas manutenidas
    if (EMERG_VOLTA_RESERVA && okR) return RESERVA;
    return EMERGENCIA;
  }
  if (okP) return PRINCIPAL;
  if (okR) return RESERVA;
  return EMERGENCIA;
}

void iniciarManobra(Rede alvo) {
  destino = alvo;
  modo = ESPERA;
  tMarca = millis();
  registrar(F("-> ESPERA (tempo de seguranca de manobra)"));
}

void bloquearRede(Rede r) {
  abrirTodos();
  bloq[r] = true;
  EEPROM.update(EE_BLOQ[r], BLOQ_MAGICO);       // sobrevive a reset
  registrar(r == PRINCIPAL ? F("REDE PRINCIPAL BLOQUEADA: manutencao necessaria")
                           : F("REDE RESERVA BLOQUEADA: manutencao necessaria"));
  iniciarManobra(redeAlvo(r));
}

// Trip saver da rede ativa (apenas principal e reserva, que tem sensor)
void atualizarTripSaver() {
  if (ativa > RESERVA) return;
  const float v = vrms[ativa];

  switch (modo) {

    case NORMAL:
      if (millis() - tMarca < T_ASSENTAMENTO_MS) break;

      if (dentroDaFaixa(v, 0)) {
        janelasFora = 0;
      } else {
        registrar(F("NORMAL: janela fora da faixa"));
        if (++janelasFora >= JANELAS_PARA_TRIP) {
          abrirTodos();
          modo = TRIP;
          tMarca = millis();
          registrar(F("-> TRIP"));
        }
      }
      break;

    case TRIP:
      if (millis() - tMarca >= T_TRIP_MS) {
        if (!fecharRele(ativa)) break;       // religa a mesma rede para testar
        modo = TESTE;
        tMarca = millis();
        janelasOk = 0;
        janelasFora = 0;
        registrar(F("-> TESTE (rele religado)"));
      }
      break;

    case TESTE:
      if (millis() - tMarca < T_ASSENTAMENTO_MS) break;

      if (dentroDaFaixa(v, V_FOLGA_RETORNO)) {
        janelasFora = 0;
        if (++janelasOk >= JANELAS_CONFIRMA) {
          tentativas = 0;
          modo = NORMAL;
          registrar(F("-> NORMAL (rede confirmada)"));
        }
      } else {
        janelasOk = 0;
        janelasFora++;
        registrar(F("TESTE: janela fora da faixa"));
      }

      if (modo == TESTE &&
          (janelasFora >= JANELAS_PARA_TRIP ||
           millis() - tMarca >= T_TESTE_MAX_MS)) {
        tentativas++;
        if (tentativas >= MAX_TENTATIVAS) {
          bloquearRede(ativa);               // 3 falhas: bloqueia e manobra
        } else {
          abrirTodos();
          modo = TRIP;
          tMarca = millis();
          registrar(F("TESTE falhou -> TRIP"));
        }
      }
      break;

    default:
      break;
  }
}

// Temporizacao da manobra (roda em todo ciclo)
void atualizarManobra() {
  if (modo == ESPERA) {
    if (millis() - tMarca < T_MANOBRA_MS) return;

    // Reavalia: o SOD pode ter confirmado manutencao durante a espera
    destino = redeAlvo(ativa);

    if (destino == ativa && ativa < N_RELES && releFechado(ativa)) {
      modo = NORMAL;
      tMarca = millis();
      janelasFora = 0;
      destino = NENHUMA;
      registrar(F("Manobra desnecessaria: rede ativa mantida"));
      return;
    }

    abrirTodos();                            // abre antes de fechar a outra
    modo = COMUTANDO;
    tMarca = millis();
    registrar(F("-> COMUTANDO (todos os reles abertos)"));
  }
  else if (modo == COMUTANDO) {
    if (millis() - tMarca < T_MORTO_MS) return;
    if (!fecharRele(destino)) return;

    ativa = destino;
    destino = NENHUMA;
    modo = NORMAL;
    tMarca = millis();
    tentativas = 0;
    janelasFora = 0;
    janelasOk = 0;
    registrar(F("-> NORMAL (rede comutada)"));
  }
}

// Se a rede ativa nao e a rede alvo (ex.: SOD confirmou manutencao), manobra
void verificarRedeAlvo() {
  if (modo != NORMAL && modo != TRIP && modo != TESTE) return;
  Rede alvo = redeAlvo(ativa);
  if (alvo != ativa) iniciarManobra(alvo);
}

// =====================================================================
//  Comunicacao com o ESP32
// =====================================================================
void enviarEstado() {
  unsigned long restante = 0;
  if (modo == ESPERA) {
    unsigned long dec = millis() - tMarca;
    if (dec < T_MANOBRA_MS) restante = (T_MANOBRA_MS - dec + 999) / 1000;
  }

  comunicacao.print(F("STATE "));
  comunicacao.print(NOMES_MODO[modo]);
  comunicacao.print(' ');
  comunicacao.print(NOMES_REDE[ativa]);
  comunicacao.print(' ');
  comunicacao.print(NOMES_REDE[destino]);
  comunicacao.print(' ');
  for (uint8_t i = 0; i < N_RELES; i++) {
    comunicacao.print(releFechado(i) ? '1' : '0');
  }
  comunicacao.print(' ');
  comunicacao.print(tentativas);
  comunicacao.print(' ');
  comunicacao.print(bloq[PRINCIPAL] ? 1 : 0);
  comunicacao.print(' ');
  comunicacao.print(bloq[RESERVA] ? 1 : 0);
  comunicacao.print(' ');
  comunicacao.print(vrms[0], 2);
  comunicacao.print(' ');
  comunicacao.print(vrms[1], 2);
  comunicacao.print(' ');
  comunicacao.println(restante);
}

void tratarComando(const char* cmd) {
  if (strcmp(cmd, "SET 1") == 0) {
    if (modo == DESLIGADO) {
      ativa = NENHUMA;
      destino = redeAlvo(NENHUMA);           // principal, senao reserva, senao emergencial
      abrirTodos();
      modo = COMUTANDO;
      tMarca = millis();
      registrar(F("Comando LIGAR"));
    }
  }
  else if (strcmp(cmd, "SET 0") == 0) {
    // Desliga tudo (tambem serve para reconhecer uma FALHA)
    abrirTodos();
    modo = DESLIGADO;
    ativa = NENHUMA;
    destino = NENHUMA;
    tentativas = 0;
    janelasFora = 0;
    registrar(F("Comando DESLIGAR"));
  }
  else if (strcmp(cmd, "MANUT 1") == 0 || strcmp(cmd, "MANUT 2") == 0) {
    // SOD confirmou a manutencao da rede indicada
    uint8_t r = cmd[6] - '1';
    if (bloq[r]) {
      bloq[r] = false;
      EEPROM.update(EE_BLOQ[r], 0);
      registrar(r == PRINCIPAL ? F("Manutencao da PRINCIPAL confirmada")
                               : F("Manutencao da RESERVA confirmada"));
      verificarRedeAlvo();                   // ja responde com a manobra iniciada
    }
  }
  // GET e qualquer outro comando: apenas responde o estado
  enviarEstado();
}

void lerComunicacao() {
  while (comunicacao.available()) {
    char c = comunicacao.read();
    if (c == '\n') {
      bufferCmd[idxCmd] = 0;
      if (idxCmd > 0) tratarComando(bufferCmd);
      idxCmd = 0;
    }
    else if (c != '\r' && idxCmd < sizeof(bufferCmd) - 1) {
      bufferCmd[idxCmd++] = c;
    }
  }
}

void depurar() {
  static unsigned long t = 0;

  if (millis() - t >= 1000) {
    t = millis();
    Serial.print(NOMES_MODO[modo]);
    Serial.print(F(" | rede="));
    Serial.print(NOMES_REDE[ativa]);
    Serial.print(F(" | reles="));
    for (uint8_t i = 0; i < N_RELES; i++) Serial.print(releFechado(i) ? '1' : '0');
    Serial.print(F(" | tent="));
    Serial.print(tentativas);
    Serial.print(F(" | bloq P/R="));
    Serial.print(bloq[PRINCIPAL] ? 1 : 0);
    Serial.print('/');
    Serial.print(bloq[RESERVA] ? 1 : 0);
    Serial.print(F(" | V1="));
    Serial.print(vrms[0], 2);
    Serial.print(F(" V2="));
    Serial.println(vrms[1], 2);
  }
}

// =====================================================================
void setup() {
  MCUSR = 0;
  wdt_disable();

  // Garante todos os reles abertos antes de configurar como saida
  for (uint8_t i = 0; i < N_RELES; i++) {
    digitalWrite(PINO_RELE[i], RELE_DESLIGADO);
    pinMode(PINO_RELE[i], OUTPUT);
  }

  Serial.begin(115200);         // Monitor Serial em 115200
  comunicacao.begin(9600);

  analogRead(PINO_V[0]);        // leitura descartada para estabilizar o ADC

  // Manutencoes pendentes sobrevivem a reset
  for (uint8_t r = 0; r < N_SENSORES; r++) {
    bloq[r] = (EEPROM.read(EE_BLOQ[r]) == BLOQ_MAGICO);
  }
  tAbertura = millis();

  Serial.print(F("Arduino iniciado! Bloqueios P/R = "));
  Serial.print(bloq[PRINCIPAL] ? 1 : 0);
  Serial.print('/');
  Serial.println(bloq[RESERVA] ? 1 : 0);

  // Watchdog de 2 s (Arduino Nano com bootloader antigo pode entrar em
  // loop de reset com WDT: nesse caso grave o bootloader novo/Optiboot)
  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();
  bool valida = medirJanela();   // janela de 100 ms
  lerComunicacao();              // responde o ESP32 entre as janelas
  if (valida) atualizarTripSaver();
  atualizarManobra();
  verificarRedeAlvo();
  verificarIntertravamento();
  depurar();
}

/*
  INTERTRAVAMENTO FISICO RECOMENDADO (cascata de contatos NA/NF):

    Carga ---- COM R1
               NA R1 ---- Rede principal
               NF R1 ---- COM R2
                          NA R2 ---- Rede reserva
                          NF R2 ---- COM R3
                                     NA R3 ---- Rede emergencial

  Assim, mesmo que dois reles fechem ao mesmo tempo (falha de software,
  rele colado, ruido), apenas UMA rede chega a carga. Para 127/220 V use
  contatores com intertravamento mecanico.
*/
