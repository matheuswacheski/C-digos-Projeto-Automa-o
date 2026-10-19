#include <WiFi.h>
#include <WebServer.h>

// ==========================================
// CONFIGURAÇÕES DE REDE (ACCESS POINT)
// ==========================================
const char* ssid = "SOD_Self_Healing"; // Nome da rede WiFi que o ESP32 vai criar
const char* password = "admin_energia"; // Senha da rede

WebServer server(80); // Servidor web na porta 80

// ==========================================
// VARIÁVEIS DE ESTADO (SOD)
// ==========================================
bool alerta_principal = false;
bool alerta_reserva = false;

// ==========================================
// PROTÓTIPOS DAS FUNÇÕES
// ==========================================
void setupWiFi();
void setupRoutes();
void handleRoot();
void handleRestaurarPrincipal();
void handleRestaurarReserva();
void lerDadosArduino();
String gerarHTML();

// ==========================================
// SETUP
// ==========================================
void setup() {
  // Comunicação Serial com o PC (para debug) e com o Arduino
  // Usamos a Serial2 do ESP32 (Pinos 16=RX2, 17=TX2) para evitar conflitos de upload
  Serial.begin(115200); 
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Comunicação com Arduino (RX no pino 16, TX no 17)
  
  setupWiFi();
  setupRoutes();
  
  server.begin();
  Serial.println("Servidor SOD Iniciado.");
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop() {
  server.handleClient(); // Escuta requisições web do celular
  lerDadosArduino();     // Escuta alertas do Arduino
}

// ==========================================
// CONFIGURAÇÃO DO ACCESS POINT
// ==========================================
void setupWiFi() {
  Serial.println("Configurando Access Point...");
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Endereço IP do SOD: ");
  Serial.println(IP); // Digite este IP no navegador do celular (geralmente 192.168.4.1)
}

// ==========================================
// ROTAS DO SERVIDOR WEB
// ==========================================
void setupRoutes() {
  // Página principal
  server.on("/", HTTP_GET, handleRoot);
  
  // Rotas chamadas pelos botões da página web
  server.on("/restaurar_principal", HTTP_POST, handleRestaurarPrincipal);
  server.on("/restaurar_reserva", HTTP_POST, handleRestaurarReserva);
}

// ==========================================
// TRATAMENTO DA COMUNICAÇÃO SERIAL (ARDUINO)
// ==========================================
void lerDadosArduino() {
  if (Serial2.available() > 0) {
    String mensagem = Serial2.readStringUntil(']');
    mensagem += ']';

    if (mensagem.indexOf("[FP]") >= 0) {
      alerta_principal = true;
      Serial.println("ALERTA SOD: TripSaver da Rede Principal falhou. Equipe despachada.");
    } 
    else if (mensagem.indexOf("[FR]") >= 0) {
      alerta_reserva = true;
      Serial.println("ALERTA SOD: TripSaver da Rede Reserva falhou. Equipe despachada.");
    }
  }
}

// ==========================================
// HANDLERS DAS ROTAS WEB
// ==========================================
void handleRoot() {
  server.send(200, "text/html", gerarHTML());
}

void handleRestaurarPrincipal() {
  // Envia comando para o Arduino liberar a rede principal
  Serial2.print("[RP]"); 
  alerta_principal = false;
  Serial.println("SOD Informa: Problema na Rede Principal resolvido.");
  
  // Recarrega a página principal
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleRestaurarReserva() {
  // Envia comando para o Arduino liberar a rede reserva
  Serial2.print("[RR]");
  alerta_reserva = false;
  Serial.println("SOD Informa: Problema na Rede Reserva resolvido.");
  
  // Recarrega a página principal
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

// ==========================================
// INTERFACE WEB (PÁGINA DO SOD)
// ==========================================
String gerarHTML() {
  String html = "<!DOCTYPE html><html lang='pt-BR'><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>SOD - Self Healing</title>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; background-color: #f4f4f4;}";
  html += "h1 { color: #333; }";
  html += ".card { background: white; padding: 20px; margin: 20px auto; width: 80%; border-radius: 10px; box-shadow: 0px 0px 10px rgba(0,0,0,0.1); }";
  html += ".alerta { color: red; font-weight: bold; }";
  html += ".normal { color: green; font-weight: bold; }";
  html += "button { padding: 15px 25px; font-size: 16px; background-color: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; }";
  html += "button:hover { background-color: #0056b3; }";
  html += "</style></head><body>";
  
  html += "<h1>Sistema de Operação de Distribuição</h1>";

  // Painel da Rede Principal
  html += "<div class='card'>";
  html += "<h2>Rede Principal</h2>";
  if (alerta_principal) {
    html += "<p class='alerta'>STATUS: FALHA DEFINTIVA (Aguardando Equipe)</p>";
    html += "<form action='/restaurar_principal' method='POST'><button type='submit'>Confirmar Reparo Concluído</button></form>";
  } else {
    html += "<p class='normal'>STATUS: OPERAÇÃO NORMAL</p>";
  }
  html += "</div>";

  // Painel da Rede Reserva
  html += "<div class='card'>";
  html += "<h2>Rede Reserva</h2>";
  if (alerta_reserva) {
    html += "<p class='alerta'>STATUS: FALHA DEFINTIVA (Aguardando Equipe)</p>";
    html += "<form action='/restaurar_reserva' method='POST'><button type='submit'>Confirmar Reparo Concluído</button></form>";
  } else {
    html += "<p class='normal'>STATUS: OPERAÇÃO NORMAL</p>";
  }
  html += "</div>";

  html += "<p><i>Atualize a página para ver novos alertas.</i></p>";
  html += "</body></html>";

  return html;
}
