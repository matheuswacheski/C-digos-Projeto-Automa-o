#include <Arduino.h>

// ==========================================
// DEFINIÇÕES DE PINOS
// ==========================================
// Sensores da Rede Principal
#define PINO_ZMPT_PRINCIPAL A0
#define PINO_ACS_PRINCIPAL A1

// Sensores da Rede Reserva
#define PINO_ZMPT_RESERVA A2
#define PINO_ACS_RESERVA A3

// Relés/Contatores
#define PINO_REDE_PRINCIPAL 2
#define PINO_REDE_RESERVA 3
#define PINO_REDE_EMERGENCIA 4

// ==========================================
// PARÂMETROS E LIMITES (AJUSTE CONFORME NECESSÁRIO)
// ==========================================
const float TENSAO_MAX = 240.0; // Sobretensão (V)
const float TENSAO_MIN = 190.0; // Subtensão (V)
const float CORRENTE_MAX = 0.5; // Sobrecorrente / Curto-circuito (A) - 500mA

// Fatores de calibração dos sensores
const float ACS_SENSIBILIDADE = 0.185; // 185mV/A para o ACS712 de 5A
const float ZMPT_FATOR = 250.0; // Fator de multiplicação empírico do trimpot do ZMPT101B

const unsigned long TEMPO_REARME_TRIPSAVER = 3000; // Tempo que o TripSaver aguarda para fechar de novo (ms)
const unsigned long DEAD_TIME = 1000; // Tempo de segurança do intertravamento lógico (ms)

// ==========================================
// VARIÁVEIS DE ESTADO
// ==========================================
enum TipoRede { PRINCIPAL, RESERVA, EMERGENCIA };
TipoRede rede_atual = PRINCIPAL;

int falhas_consecutivas = 0;
bool bloqueio_principal = false;
bool bloqueio_reserva = false;

// ==========================================
// PROTÓTIPOS DAS FUNÇÕES
// ==========================================
float lerCorrenteRMS(int pino);
float lerTensaoRMS(int pino);
void manobraRede(TipoRede nova_rede);
void processarFalha();
void verificarComandosESP();

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(9600); // Comunicação com o ESP32
  
  pinMode(PINO_REDE_PRINCIPAL, OUTPUT);
  pinMode(PINO_REDE_RESERVA, OUTPUT);
  pinMode(PINO_REDE_EMERGENCIA, OUTPUT);

  // Inicializa tudo desligado por segurança
  digitalWrite(PINO_REDE_PRINCIPAL, LOW);
  digitalWrite(PINO_REDE_RESERVA, LOW);
  digitalWrite(PINO_REDE_EMERGENCIA, LOW);

  // Inicia na rede principal
  manobraRede(PRINCIPAL);
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop() {
  verificarComandosESP();

  float tensao = 0.0;
  float corrente = 0.0;
  bool monitorar = false;

  // Seleciona quais sensores ler dependendo da rede conectada
  if (rede_atual == PRINCIPAL) {
    tensao = lerTensaoRMS(PINO_ZMPT_PRINCIPAL);
    corrente = lerCorrenteRMS(PINO_ACS_PRINCIPAL);
    monitorar = true;
  } 
  else if (rede_atual == RESERVA) {
    tensao = lerTensaoRMS(PINO_ZMPT_RESERVA);
    corrente = lerCorrenteRMS(PINO_ACS_RESERVA);
    monitorar = true;
  }
  // Se estiver na rede de EMERGÊNCIA, não há sensores definidos, apenas aguarda reparos.

  if (monitorar) {
    bool anomalia = (tensao > TENSAO_MAX) || (tensao < TENSAO_MIN) || (corrente > CORRENTE_MAX);

    if (anomalia) {
      processarFalha();
    }
  }
}

// ==========================================
// FUNÇÃO: PROCESSAR FALHA (TRIPSAVER)
// ==========================================
void processarFalha() {
  falhas_consecutivas++;
  
  // ABRIR TRIPSAVER DA REDE ATUAL (Trip) - Interrompe todas por segurança
  digitalWrite(PINO_REDE_PRINCIPAL, LOW);
  digitalWrite(PINO_REDE_RESERVA, LOW);
  digitalWrite(PINO_REDE_EMERGENCIA, LOW);

  if (falhas_consecutivas < 3) {
    // Modo TripSaver: Aguarda e tenta religar
    delay(TEMPO_REARME_TRIPSAVER);
    manobraRede(rede_atual); 
  } 
  else {
    // Falha definitiva após 3 tentativas
    falhas_consecutivas = 0; // Zera contagem para a próxima rede assumir limpa

    if (rede_atual == PRINCIPAL && !bloqueio_principal) {
      bloqueio_principal = true;
      Serial.print("[FP]"); // Informa ESP32: Falha Principal
      manobraRede(RESERVA);
    } 
    else if (rede_atual == RESERVA && !bloqueio_reserva) {
      bloqueio_reserva = true;
      Serial.print("[FR]"); // Informa ESP32: Falha Reserva
      manobraRede(EMERGENCIA);
    }
  }
}

// ==========================================
// FUNÇÃO: INTERTRAVAMENTO E MANOBRA DE REDE
// ==========================================
void manobraRede(TipoRede nova_rede) {
  // 1. Break-Before-Make (Abre antes de fechar)
  digitalWrite(PINO_REDE_PRINCIPAL, LOW);
  digitalWrite(PINO_REDE_RESERVA, LOW);
  digitalWrite(PINO_REDE_EMERGENCIA, LOW);
  
  // 2. Tempo de segurança (Dead-time para extinção de arco voltaico)
  delay(DEAD_TIME);
  
  // 3. Verifica bloqueios lógicos e aciona a rede correspondente
  if (nova_rede == PRINCIPAL && !bloqueio_principal) {
    rede_atual = PRINCIPAL;
    digitalWrite(PINO_REDE_PRINCIPAL, HIGH);
  } 
  else if (nova_rede == RESERVA && !bloqueio_reserva) {
    rede_atual = RESERVA;
    digitalWrite(PINO_REDE_RESERVA, HIGH);
  } 
  else if (nova_rede == EMERGENCIA) {
    rede_atual = EMERGENCIA;
    digitalWrite(PINO_REDE_EMERGENCIA, HIGH);
  }
}

// ==========================================
// FUNÇÃO: COMUNICAÇÃO COM O ESP32
// ==========================================
void verificarComandosESP() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil(']'); 
    comando += ']'; 

    if (comando.indexOf("[RP]") >= 0) {
      bloqueio_principal = false; // Confirmação de manutenção da Principal
    } 
    if (comando.indexOf("[RR]") >= 0) {
      bloqueio_reserva = false; // Confirmação de manutenção da Reserva
    }

    // REGRA DE RETORNO: 
    // Só volta para a Principal se AMBAS as redes (Principal e Reserva) estiverem sem bloqueio (manutenidas).
    if (!bloqueio_principal && !bloqueio_reserva && rede_atual != PRINCIPAL) {
      falhas_consecutivas = 0;
      manobraRede(PRINCIPAL);
    } 
    // Caso a reserva tenha sido manutenida, mas a principal ainda não, e o sistema esteja na Emergência:
    else if (!bloqueio_reserva && bloqueio_principal && rede_atual == EMERGENCIA) {
      falhas_consecutivas = 0;
      manobraRede(RESERVA);
    }
  }
}

// ==========================================
// FUNÇÕES DOS SENSORES (TRUE RMS)
// ==========================================
float lerCorrenteRMS(int pino) {
  uint32_t start_time = millis();
  int max_val = 0;
  int min_val = 1023;
  
  while (millis() - start_time < 25) {
    int val = analogRead(pino);
    if (val > max_val) max_val = val;
    if (val < min_val) min_val = val;
  }
  
  float tensao_pp = (max_val - min_val) * (5.0 / 1023.0);
  float tensao_rms = (tensao_pp / 2.0) * 0.707;
  float corrente_rms = tensao_rms / ACS_SENSIBILIDADE;
  
  if (corrente_rms < 0.05) corrente_rms = 0; 
  return corrente_rms;
}

float lerTensaoRMS(int pino) {
  uint32_t start_time = millis();
  int max_val = 0;
  int min_val = 1023;
  
  while (millis() - start_time < 25) {
    int val = analogRead(pino);
    if (val > max_val) max_val = val;
    if (val < min_val) min_val = val;
  }
  
  float tensao_pp = (max_val - min_val) * (5.0 / 1023.0);
  float tensao_rms_sensor = (tensao_pp / 2.0) * 0.707;
  float tensao_final_rms = tensao_rms_sensor * ZMPT_FATOR; 
  
  if(tensao_final_rms < 10.0) tensao_final_rms = 0;
  return tensao_final_rms;
}