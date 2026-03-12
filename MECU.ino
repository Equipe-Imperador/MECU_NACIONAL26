/*
    MECU
*/
#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <esp_task_wdt.h>

// --- Bibliotecas MQTT e GSM ---
#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ====================================================================
// 1. DEFINIÇÕES DE PINOS E ESTRUTURAS
// ====================================================================
#define PIN_BOTAO_PAINEL 32
#define DWIN_TX 26
#define DWIN_RX 25 
#define MODEM_TX 17
#define MODEM_RX 16 
#define CAN_CS 15
#define CAN_INT 27
#define CAN_SCK 14
#define CAN_MISO 12
#define CAN_MOSI 13
#define SD_CS 5
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23

#define TELA_PRINCIPAL  0x00  
#define TELA_SECUNDARIA 0x02  
#define TELA_BOX        0x01  

volatile uint8_t telaAtual = TELA_PRINCIPAL;
volatile bool forcarMudancaTela = true; 

struct TelemetriaGlobal {
    uint32_t timestamp;
    uint16_t rpm;
    float velocidade, tempCVT, v_LF, v_RF;
    float vBat, presTras, tempBat, perT, perF;
    float pedalFreio, presDiant, presCM, accX, accY, accZ; 
    bool correnteDif; 
} dados;

QueueHandle_t filaSD;
SemaphoreHandle_t mutexDados;
File dataFile;
char nomeArquivo[30];

TinyGsm modem(Serial1);
TinyGsmClient gsmClient(modem);
PubSubClient mqttClient(gsmClient);
MCP_CAN CAN0(CAN_CS);
SPIClass sdSPI(HSPI); 

const char apn[] = "claro.com.br";
const char* mqtt_server = "72.60.141.159";
const int mqtt_port = 1883;
const char* mqtt_user = "imperador_mqtt";
const char* mqtt_pass = "imperador25";
const char* topic_telemetry = "imperador/telemetria";
const char* topic_command = "imperador/comandos/box";

void vTaskCAN(void *pvParameters);
void vTaskModem(void *pvParameters);
void vTaskSD(void *pvParameters);
void vTaskDWIN(void *pvParameters);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void enviarMsgCAN(uint32_t id, float valor);

// ====================================================================
// 2. SETUP
// ====================================================================
void setup() {
    Serial.begin(921600);
    Serial.println("\n [MECU] MODO DEBUG CAN ATIVADO ");

    pinMode(PIN_BOTAO_PAINEL, INPUT_PULLUP);
    mutexDados = xSemaphoreCreateMutex();
    
    Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX); 
    Serial2.begin(9600, SERIAL_8N1, DWIN_RX, DWIN_TX);     

    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println(" !!! ERRO: MCP2515 NÃO INICIALIZOU !!!");
    } else {
        CAN0.setMode(MCP_NORMAL);
        Serial.println(" >>> CAN Inicializada e em modo NORMAL.");
    }

    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, sdSPI)) {
        Serial.println(" Erro: SD Card falhou.");
    } else {
        int n = 1;
        while (n < 1000) {
            sprintf(nomeArquivo, "/MECU_%d.csv", n);
            if (!SD.exists(nomeArquivo)) break;
            n++;
        }
        dataFile = SD.open(nomeArquivo, FILE_WRITE);
        if (dataFile) {
            dataFile.println("ms;rpm;vel;tCVT;vBat;pTras;tBat;perT;perF;pedF;pDiant;pCM;accX;accY;accZ;vLF;vRF;corrDif");
            dataFile.flush();
        }
    }

    esp_task_wdt_config_t twdt_config = { .timeout_ms = 8000, .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, .trigger_panic = true };
    esp_task_wdt_deinit(); 
    esp_task_wdt_init(&twdt_config);

    filaSD = xQueueCreate(100, sizeof(TelemetriaGlobal));
    if (filaSD != NULL) {
        xTaskCreatePinnedToCore(vTaskCAN, "CAN", 4096, NULL, 3, NULL, 1);
        xTaskCreatePinnedToCore(vTaskDWIN, "DWIN", 2048, NULL, 2, NULL, 1);
        xTaskCreatePinnedToCore(vTaskModem, "GSM", 10240, NULL, 0, NULL, 0);
        xTaskCreatePinnedToCore(vTaskSD, "SD", 4096, NULL, 2, NULL, 0);
    }
}

void loop() { vTaskDelete(NULL); }

void enviarMsgCAN(uint32_t id, float valor) {
    int16_t val = (int16_t)(valor * 100.0f);
    byte b[2] = {(byte)(val >> 8), (byte)(val & 0xFF)};
    CAN0.sendMsgBuf(id, 0, 2, b);
}

// ====================================================================
// 3. TAREFAS (TASKS)
// ====================================================================

// --- TAREFA CAN ---
void vTaskCAN(void *pvParameters) {
    long unsigned int rxId;
    unsigned char len, rxBuf[8];
    uint32_t lastLogTime = 0;
    
    esp_task_wdt_add(NULL);

    for (;;) {
        esp_task_wdt_reset();
        
        // Usando o 'while' para esvaziar completamente o buffer do MCP2515 a cada ciclo
        while (CAN0.checkReceive() == CAN_MSGAVAIL) {
            CAN0.readMsgBuf(&rxId, &len, rxBuf);
            
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            dados.timestamp = millis();
            
            if (len == 2) {
                // Junta os 2 bytes em um inteiro
                int16_t valorInt = (rxBuf[0] << 8) | rxBuf[1];
                // Transforma em float (dividindo por 100) para os demais sensores
                float valorFloat = (float)valorInt / 100.0f;

               /* // --- DEBUG CAN COM VALORES REAIS ---
                if (rxId == 0x200) {
                    Serial.printf("[CAN RX] ID: 0x200 | RPM recebido: %d\n", valorInt);
                } else {
                    Serial.printf("[CAN RX] ID: 0x%03X | Valor recebido: %.2f\n", rxId, valorFloat);
                }
                // ------------------------------------
                */
                switch (rxId) {
                    case 0x200: dados.rpm = (uint16_t)valorInt; break; 
                    case 0x201: dados.velocidade = valorFloat; break;
                    case 0x202: dados.tempCVT = valorFloat; break;
                    case 0x203: dados.v_LF = valorFloat; break; 
                    case 0x204: dados.v_RF = valorFloat; break; 
                    case 0x300: dados.vBat = valorFloat; break;
                    case 0x301: dados.presTras = valorFloat; break;
                    case 0x303: dados.tempBat = valorFloat; break;
                    case 0x304: dados.perT = valorFloat; break;
                    case 0x305: dados.perF = valorFloat; break;
                    case 0x400: dados.pedalFreio = valorFloat; break;
                    case 0x402: dados.presDiant = valorFloat; break;
                    case 0x403: dados.presCM = valorFloat; break;
                    case 0x404: dados.accX = valorFloat; break;
                    case 0x405: dados.accY = valorFloat; break;
                    case 0x406: dados.accZ = valorFloat; break; 
                    case 0x407: dados.correnteDif = (valorInt > 0); break; 
                    default:
                        Serial.printf("   (!) ID desconhecido na lógica: 0x%X\n", rxId);
                        break;
                }
            }
            xSemaphoreGive(mutexDados);
        }

        // Envio para a fila do SD movido para fora do while
        // Garante que o SD vai receber o frame mais atualizado a cada 10ms cravados
        if (millis() - lastLogTime >= 10) {
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            TelemetriaGlobal copiaDados = dados;
            xSemaphoreGive(mutexDados);
            
            xQueueSend(filaSD, &copiaDados, 0);
            lastLogTime = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// --- TAREFA DWIN ---
void vTaskDWIN(void *pvParameters) {
    uint32_t ultimoTempoBotao = 0;
    for (;;) {
        if (digitalRead(PIN_BOTAO_PAINEL) == LOW && (millis() - ultimoTempoBotao > 300)) {
            ultimoTempoBotao = millis();
            if (telaAtual == TELA_PRINCIPAL) telaAtual = TELA_SECUNDARIA;
            else if (telaAtual == TELA_SECUNDARIA) telaAtual = TELA_PRINCIPAL;
            else if (telaAtual == TELA_BOX) telaAtual = TELA_PRINCIPAL;
            forcarMudancaTela = true;
        }
        
        if (forcarMudancaTela) {
            byte frameTela[10] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, telaAtual};
            Serial2.write(frameTela, 10);
            forcarMudancaTela = false;
        }
        
        if (telaAtual != TELA_BOX) {
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            uint16_t rpmTela = dados.rpm;
            uint16_t velTela = (uint16_t)dados.velocidade; // Convertendo para inteiro para enviar ao DWIN
            uint16_t tempCvtTela = (uint16_t)dados.tempCVT; // Convertendo para inteiro para enviar ao DWIN
            xSemaphoreGive(mutexDados);

            // 1. Envia RPM (VP: 0x31 0x00)
            byte frameRpm[8] = {0x5A, 0xA5, 0x05, 0x82, 0x31, 0x00, (byte)(rpmTela >> 8), (byte)(rpmTela & 0xFF)};
            Serial2.write(frameRpm, 8);
            vTaskDelay(pdMS_TO_TICKS(10)); 
            
            // 2. Envia Velocidade (ATENÇÃO: Altere o 0x32 0x00 para o VP configurado no seu DGUS)
            byte frameVel[8] = {0x5A, 0xA5, 0x05, 0x82, 0x11, 0x00, (byte)(velTela >> 8), (byte)(velTela & 0xFF)};
            Serial2.write(frameVel, 8);
            vTaskDelay(pdMS_TO_TICKS(10));

            // 3. Envia Temp CVT (ATENÇÃO: Altere o 0x33 0x00 para o VP configurado no seu DGUS)
            byte frameTemp[8] = {0x5A, 0xA5, 0x05, 0x82, 0x21, 0x00, (byte)(tempCvtTela >> 8), (byte)(tempCvtTela & 0xFF)};
            Serial2.write(frameTemp, 8);
        }
        vTaskDelay(pdMS_TO_TICKS(80)); // Fechando ciclo total em ~100ms
    }
}

// --- TAREFA SD ---
void vTaskSD(void *pvParameters) {
    TelemetriaGlobal d;
    int ct = 0;
    for (;;) {
        if (xQueueReceive(filaSD, &d, portMAX_DELAY)) {
            if (dataFile) {
                dataFile.printf("%u;%u;%.1f;%.1f;%.1f;%.2f;%.1f;%.0f;%.0f;%.1f;%.2f;%.2f;%.2f;%.2f;%.2f;%.1f;%.1f;%d\n", 
                    d.timestamp, d.rpm, d.velocidade, d.tempCVT, d.vBat, 
                    d.presTras, d.tempBat, d.perT, d.perF, d.pedalFreio, 
                    d.presDiant, d.presCM, d.accX, d.accY, d.accZ, d.v_LF, d.v_RF, d.correnteDif);
                if (++ct >= 50) { dataFile.flush(); ct = 0; }
            }
        }
    }
}

// --- CALLBACK MQTT ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) message += (char)payload[i];
    
    if (String(topic) == topic_command) {
        StaticJsonDocument<200> doc;
        if (!deserializeJson(doc, message)) {
            if (doc.containsKey("command")) {
                const char* command = doc["command"];
                if (String(command) == "PIT") { telaAtual = TELA_BOX; forcarMudancaTela = true;  Serial.println("* CHAMADA PARA O BOX RECEBIDA! *");}
                if (String(command) == "PISTA") { telaAtual = TELA_PRINCIPAL; forcarMudancaTela = true;  Serial.println("* RETORNAR PISTA RECEBIDO! *");}
            }
            if (doc.containsKey("acionamentoDif")) {
                bool estadoDif = doc["acionamentoDif"];
                Serial.println("Input DIF servidor recebido ");
                enviarMsgCAN(0x500, estadoDif ? 1.0f : 0.0f);
                
            }
            if (doc.containsKey("acionamentoBuzina")) {
                bool estadoBuzina = doc["acionamentoBuzina"];
                Serial.println("Input BUZINA servidor recebido ");
                enviarMsgCAN(0x501, estadoBuzina ? 1.0f : 0.0f);
            }
        }
    }
}

// --- TAREFA MODEM ---
void vTaskModem(void *pvParameters) {
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);
    
    for (;;) {
        if (!modem.isNetworkConnected()) {
            Serial.print("[MODEM] Procurando rede... ");
            if (modem.waitForNetwork(3000)) Serial.println("CONECTADO!");
            else Serial.println("Ainda não.");
        } 
        else if (!modem.isGprsConnected()) {
            Serial.println("[MODEM] Conectando GPRS...");
            modem.gprsConnect(apn, "", "");
        } 
        else if (!mqttClient.connected()) {
            Serial.println("[MQTT] Conectando...");
            String clientId = "MECU-" + String(random(0xffff), HEX);
            if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
                mqttClient.subscribe(topic_command);
            }
        }
        
        if (mqttClient.connected()) {
            TelemetriaGlobal d;
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            d = dados;
            xSemaphoreGive(mutexDados);

            StaticJsonDocument<1024> doc; 
            doc["rpm"] = d.rpm; doc["vel"] = d.velocidade; doc["tCVT"] = d.tempCVT;
            doc["vBat"] = d.vBat; doc["pTras"] = d.presTras; doc["tBat"] = d.tempBat;
            doc["perT"] = d.perT; doc["perF"] = d.perF; doc["pedF"] = d.pedalFreio;
            doc["pDiant"] = d.presDiant; doc["pCM"] = d.presCM; doc["accX"] = d.accX;
            doc["accY"] = d.accY; doc["accZ"] = d.accZ; doc["vLF"] = d.v_LF;
            doc["vRF"] = d.v_RF; doc["corrDif"] = d.correnteDif;

            char buffer[1024];
            size_t n = serializeJson(doc, buffer);
            mqttClient.publish(topic_telemetry, buffer, n);
            mqttClient.loop();
        }
        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
}
