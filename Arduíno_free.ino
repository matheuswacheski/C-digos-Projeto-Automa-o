#include <SoftwareSerial.h>
#include <EEPROM.h>

SoftwareSerial comunicacao(10, 11);
// RX = 10 | TX = 11

// ---------- Reles: 1 = principal, 2 = reserva, 3 = emergencial ----------
const uint8_t PINOS_RELE[4] = {0, 7, 8, 9};
// Muitos modulos rele sao acionados em nivel LOW.
#define RELE_LIGADO LOW
#define RELE_DESLIGADO HIGH
const unsigned long T_INTERTRAVA_MS = 100UL;   // tempo de queda do contator

// ---------- Sensores de tensao (um condicionador por rede) ----------
// Sensor 1 (rede principal) no A0 | Sensor 2 (rede reserva) no A1
// A rede emergencial nao possui sensor.
const uint8_t PINOS_V[3] = {0, A0, A1};
const float GANHO = 11.0;                      // (R1 + R2) / R2
const float CALIBRACAO[3] = {0, 1.00, 1.00};   // Vrms multimetro / Vrms Arduino
const float BANDGAP_V = 1.100;                 // referencia interna
const unsigned long JANELA_US = 100000UL;      // 100 ms = 5 ciclos (50 Hz) ou 6 (60 Hz)

// ---------- Trip saver e manobra ----------
const float V_MIN = 10.8;                       // 12 V -10%
const float V_MAX = 13.2;                       // 12 V +10%
const float V_FOLGA_RETORNO = 0.2;              // histerese para confirmar retorno
const uint8_t JANELAS_PARA_TRIP = 5;            // 5 janelas seguidas fora = problema
const unsigned long T_TRIP_MS = 3000UL;         // tempo com o rele desligado
const unsigned long T_ASSENTAMENTO_MS = 1000UL; // ignora a partida apos fechar o rele
const uint8_t JANELAS_CONFIRMA = 10;            // 10 janelas seguidas ok = rede ok
const unsigned long T_TESTE_MAX_MS = 4000UL;    // tempo maximo do teste de retorno
const uint8_t MAX_TENTATIVAS = 3;               // religamentos antes de declarar falha
const unsigned long T_SEG_MANOBRA_MS = 3000UL;  // tempo de seguranca entre abrir uma rede e fechar outra
const bool EMERGENCIA_PARA_RESERVA = true;      // sai da emergencial para a reserva quando so ela for manutenida

// ---------- Estado persistente (falhas) ----------
const int EE_FALHAS = 0;

enum Modo : uint8_t { DESLIGADO, NORMAL, TRIP, TESTE, MANOBRA };
const char* const NOMES[] = {"DESLIGADO", "NORMAL", "TRIP", "TESTE", "MANOBRA"};

Modo modo = DESLIGADO;
uint8_t fonte = 0;            // rele fechado agora (0 = nenhum)
uint8_t alvo = 0;             // rede em trip saver, em teste ou destino da manobra
uint8_t tentativas = 0;
bool falha[4] = {false, false, false, false};  // true = rede precisa de manutencao
bool testeInicial = false;    // 1o teste apos manobra: se falhar, inicia o trip saver
bool retorno = false;         // manobra de retorno (rede principal) apos manutencao
uint8_t janelasFora = 0;
uint8_t janelasOk = 0;
unsigned long tMarca = 0;

float vrms[3] = {0, 0, 0};
float vccAtual = 5.0;
uint16_t contMin = 0, contMax = 0;
uint32_t nAmostras = 0;

char bufferCmd[16];
uint8_t idxCmd = 0;

// =====================================================================
//  Reles e intertravamento
// =====================================================================

void desligarTodos() {
  for (uint8_t i = 1; i <= 3; i++) digitalWrite(PINOS_RELE[i], RELE_DESLIGADO);
  fonte = 0;
}

// Unica funcao que fecha um rele. Garante que nenhum outro esteja fechado.
void energizar(uint8_t n) {
  bool outraLigada = false;
  for (uint8_t i = 1; i <= 3; i++) {
    if (i != n && digitalRead(PINOS_RELE[i]) == RELE_LIGADO) {
      outraLigada = true;
      digitalWrite(PINOS_RELE[i], RELE_DESLIGADO);
    }
  }
  if (outraLigada) delay(T_INTERTRAVA_MS);
  digitalWrite(PINOS_RELE[n], RELE_LIGADO);
  fonte = n;
}

// Rede de seguranca: se por algum motivo houver 2 reles fechados, desliga tudo.
void verificarIntertravamento() {
  uint8_t ligados = 0;
  for (uint8_t i = 1; i <= 3; i++) {
    if (digitalRead(PINOS_RELE[i]) == RELE_LIGADO) ligados++;
  }
  if (ligados > 1) {
    desligarTodos();
    modo = DESLIGADO;
    alvo = 0;
    Serial.println(F("ERRO: dois reles ligados! Tudo desligado."));
  }
}

// =====================================================================
//  Falhas persistentes
// =====================================================================

void salvarFalhas() {
  EEPROM.update(EE_FALHAS, 0xA0 | (falha[1] ? 1 : 0) | (falha[2] ? 2 : 0));
}

void carregarFalhas() {
  uint8_t b = EEPROM.read(EE_FALHAS);
  if ((b & 0xF0) == 0xA0) {
    falha[1] = b & 1;
    falha[2] = b & 2;
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

  analogRead(A0);                             // volta o canal ao A0 (leitura descartada)
  return BANDGAP_V * 1024.0 / (soma / 8.0);
}

// Mede uma janela de 100 ms no sensor s (1 ou 2).
// Retorna false se a janela foi contaminada pela comunicacao serial.
bool medirJanela(uint8_t s) {
  int antes = comunicacao.available();
  float vcc = lerVcc();

  uint32_t soma = 0, somaQuad = 0, n = 0;
  uint16_t mn = 1023, mx = 0;

  unsigned long inicio = micros();
  while (micros() - inicio < JANELA_US) {
    uint16_t v = analogRead(PINOS_V[s]);
    soma += v;
    somaQuad += (uint32_t)v * v;
    n++;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }

  if (comunicacao.available() != antes) return false;

  float media = (float)soma / n;
  float variancia = (float)somaQuad / n - media * media;
  if (variancia < 0) variancia = 0;

  vrms[s] = sqrt(variancia) * (vcc / 1024.0) * GANHO * CALIBRACAO[s];
  vccAtual = vcc;
  contMin = mn;
  contMax = mx;
  nAmostras = n;
  return true;
}

// Sensor que protege a rede em uso (0 = nenhum precisa ser avaliado)
uint8_t sensorDaProtecao() {
  if ((modo == NORMAL || modo == TESTE) && (fonte == 1 || fonte == 2)) return fonte;
  return 0;
}

void registrar(const __FlashStringHelper* msg, uint8_t s) {
  Serial.print(msg);
  Serial.print(F(" | rede="));
  Serial.print(s);
  Serial.print(F(" V="));
  Serial.print(vrms[s], 2);
  Serial.print(F(" Vcc="));
  Serial.print(vccAtual, 2);
  Serial.print(F(" min/max="));
  Serial.print(contMin);
  Serial.print('/');
  Serial.print(contMax);
  Serial.print(F(" n="));
  Serial.println(nAmostras);
}

// =====================================================================
//  Logica de redes
// =====================================================================

// Rede que deveria alimentar a carga, dadas as falhas e a rede atual.
// atual = 3: so volta para a principal se principal E reserva foram manutenidas.
uint8_t escolherFonte(uint8_t atual) {
  if (atual == 3) {
    if (!falha[1] && !falha[2]) return 1;
    if (EMERGENCIA_PARA_RESERVA && !falha[2]) return 2;
    return 3;
  }
  if (!falha[1]) return 1;
  if (!falha[2]) return 2;
  return 3;
}

void iniciarNormal(uint8_t n) {
  energizar(n);
  alvo = n;
  modo = NORMAL;
  tentativas = 0;
  janelasFora = 0;
  retorno = false;
  testeInicial = false;
  tMarca = millis();
  Serial.print(F("-> NORMAL na rede "));
  Serial.println(n);
}

// Abre tudo, espera o tempo de seguranca e depois fecha a rede d.
void iniciarManobra(uint8_t d, bool ehRetorno) {
  desligarTodos();
  modo = MANOBRA;
  alvo = d;
  retorno = ehRetorno;
  testeInicial = false;
  tentativas = 0;
  tMarca = millis();
  Serial.print(F("-> MANOBRA para a rede "));
  Serial.println(d);
}

void entrarTrip() {
  desligarTodos();
  modo = TRIP;
  tMarca = millis();
  Serial.print(F("-> TRIP na rede "));
  Serial.println(alvo);
}

// A rede 'rede' esgotou as tentativas (ou falhou no retorno): precisa de manutencao
void declararFalha(uint8_t rede) {
  falha[rede] = true;
  salvarFalhas();
  Serial.print(F("FALHA: rede "));
  Serial.print(rede);
  Serial.println(F(" requer manutencao"));
  uint8_t d = escolherFonte(0);
  iniciarManobra(d, d == 1);
}

void atualizarProtecao(uint8_t sensorMedido) {
  switch (modo) {

    case NORMAL:
      if (fonte != 1 && fonte != 2) break;           // emergencial: sem sensor
      if (sensorMedido != fonte) break;
      if (millis() - tMarca < T_ASSENTAMENTO_MS) break;

      if (dentroDaFaixa(vrms[fonte], 0)) {
        janelasFora = 0;
      } else {
        registrar(F("NORMAL: janela fora da faixa"), fonte);
        if (++janelasFora >= JANELAS_PARA_TRIP) {
          alvo = fonte;
          tentativas = 0;
          entrarTrip();
        }
      }
      break;

    case TRIP:
      if (millis() - tMarca >= T_TRIP_MS) {
        energizar(alvo);                             // religa para testar a rede
        modo = TESTE;
        tMarca = millis();
        janelasOk = 0;
        janelasFora = 0;
        Serial.println(F("-> TESTE (rele religado)"));
      }
      break;

    case TESTE: {
      if (millis() - tMarca >= T_ASSENTAMENTO_MS && sensorMedido == fonte) {
        if (dentroDaFaixa(vrms[fonte], V_FOLGA_RETORNO)) {
          janelasFora = 0;
          janelasOk++;
        } else {
          janelasOk = 0;
          janelasFora++;
          registrar(F("TESTE: janela fora da faixa"), fonte);
        }
      }

      bool ok = janelasOk >= JANELAS_CONFIRMA;
      bool falhou = janelasFora >= JANELAS_PARA_TRIP ||
                    millis() - tMarca >= T_TESTE_MAX_MS;

      if (ok) {
        tentativas = 0;                              // rede voltou: zera a contagem
        testeInicial = false;
        retorno = false;
        janelasFora = 0;
        modo = NORMAL;
        Serial.print(F("-> NORMAL (rede "));
        Serial.print(alvo);
        Serial.println(F(" confirmada)"));
      }
      else if (falhou) {
        if (retorno) {
          // Retorno apos manutencao falhou: rede continua com problema
          retorno = false;
          declararFalha(alvo);
        }
        else if (testeInicial) {
          // Rede de destino sem tensao: inicia o trip saver nela
          testeInicial = false;
          entrarTrip();
        }
        else {
          tentativas++;
          if (tentativas >= MAX_TENTATIVAS) {
            declararFalha(alvo);                     // 3 falhas: rede indisponivel
          } else {
            entrarTrip();
          }
        }
      }
      break;
    }

    case MANOBRA:
      if (millis() - tMarca >= T_SEG_MANOBRA_MS) {
        if (alvo == 3) {
          iniciarNormal(3);                          // emergencial: sem verificacao
        } else {
          energizar(alvo);
          modo = TESTE;
          tMarca = millis();
          janelasOk = 0;
          janelasFora = 0;
          testeInicial = !retorno;
          Serial.print(F("-> TESTE da rede "));
          Serial.println(alvo);
        }
      }
      break;

    default:   // DESLIGADO
      break;
  }
}

// SOD informou que a rede 'rede' (1 ou 2) foi manutenida
void tratarManutencao(uint8_t rede) {
  if (rede < 1 || rede > 2 || !falha[rede]) return;

  falha[rede] = false;
  salvarFalhas();
  Serial.print(F("Manutencao confirmada: rede "));
  Serial.println(rede);

  uint8_t d;
  switch (modo) {
    case NORMAL:
      d = escolherFonte(fonte);
      if (d != fonte) iniciarManobra(d, d == 1);
      break;

    case TRIP:
    case TESTE:
      if (alvo == 2 && !falha[1]) iniciarManobra(1, true);
      break;

    case MANOBRA:
      d = escolherFonte(alvo);
      if (d != alvo) {
        alvo = d;
        retorno = (d == 1);
      }
      break;

    default:
      break;
  }
}

// =====================================================================
//  Comunicacao com o ESP32
// =====================================================================

// STATE <modo> <fonte> <alvo> <tentativas> <falhaP> <falhaR> <v1> <v2>
void enviarEstado() {
  comunicacao.print(F("STATE "));
  comunicacao.print(NOMES[modo]);
  comunicacao.print(' ');
  comunicacao.print(fonte);
  comunicacao.print(' ');
  comunicacao.print(alvo);
  comunicacao.print(' ');
  comunicacao.print(tentativas);
  comunicacao.print(' ');
  comunicacao.print(falha[1] ? 1 : 0);
  comunicacao.print(' ');
  comunicacao.print(falha[2] ? 1 : 0);
  comunicacao.print(' ');
  comunicacao.print(vrms[1], 2);
  comunicacao.print(' ');
  comunicacao.println(vrms[2], 2);
}

void tratarComando(const char* cmd) {
  if (strcmp(cmd, "SET 1") == 0) {
    if (modo == DESLIGADO) {
      iniciarNormal(escolherFonte(0));
    }
  }
  else if (strcmp(cmd, "SET 0") == 0) {
    desligarTodos();
    modo = DESLIGADO;
    alvo = 0;
    tentativas = 0;
    retorno = false;
    testeInicial = false;
  }
  else if (strcmp(cmd, "MANUT 1") == 0) {
    tratarManutencao(1);
  }
  else if (strcmp(cmd, "MANUT 2") == 0) {
    tratarManutencao(2);
  }
  // GET e qualquer outro comando: apenas responde o estado
  enviarEstado();
}

void lerComunicacao() {
  while (comunicacao.available()) {
    char c = comunicacao.read();
    if (c == '\n') {
      bufferCmd[idxCmd] = 0;
      if (idxCmd > 0) {
        Serial.println(bufferCmd);
        tratarComando(bufferCmd);
      }
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
    Serial.print(NOMES[modo]);
    Serial.print(F(" | fonte="));
    Serial.print(fonte);
    Serial.print(F(" alvo="));
    Serial.print(alvo);
    Serial.print(F(" | tentativas="));
    Serial.print(tentativas);
    Serial.print(F(" | falhas P/R="));
    Serial.print(falha[1] ? 1 : 0);
    Serial.print('/');
    Serial.print(falha[2] ? 1 : 0);
    Serial.print(F(" | V1="));
    Serial.print(vrms[1], 2);
    Serial.print(F(" V2="));
    Serial.print(vrms[2], 2);
    Serial.print(F(" | Vcc="));
    Serial.println(vccAtual, 2);
  }
}

void setup() {
  // Garante que todos os reles comecem desligados.
  for (uint8_t i = 1; i <= 3; i++) {
    digitalWrite(PINOS_RELE[i], RELE_DESLIGADO);
    pinMode(PINOS_RELE[i], OUTPUT);
  }

  Serial.begin(9600);
  comunicacao.begin(9600);

  analogRead(A0);  // leitura descartada para estabilizar o ADC

  carregarFalhas();   // lembra quais redes aguardam manutencao

  Serial.println("Arduino iniciado!");
}

void loop() {
  static uint8_t ciclo = 0, altern = 0;

  uint8_t s = sensorDaProtecao();
  uint8_t sensorLogica = s;

  if (s == 0) {
    altern ^= 1;                       // so exibicao: alterna entre os sensores
    s = 1 + altern;
  } else if (++ciclo >= 6) {
    ciclo = 0;
    s = (s == 1) ? 2 : 1;              // a cada 6 janelas, le o outro sensor (exibicao)
    sensorLogica = 0;
  }

  bool valida = medirJanela(s);
  lerComunicacao();                    // responde o ESP32 entre as janelas
  atualizarProtecao(valida ? sensorLogica : 0);
  verificarIntertravamento();
  depurar();
}
