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
// 1. DEFINIÇÕES DE PINOS (Hardware Final)
// ====================================================================
#define PIN_BOTAO_PAINEL 32

// DWIN (Serial 2)
#define DWIN_TX 25 
#define DWIN_RX 26 

// MODEM SIM7600 (Serial 1)
#define MODEM_TX 16 
#define MODEM_RX 17 

// CAN (Pinos customizados para SPI)
#define CAN_CS 15
#define CAN_INT 27
#define CAN_SCK 14
#define CAN_MISO 12
#define CAN_MOSI 13

// SD CARD (Pinos VSPI)
#define SD_CS 5
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23

// ====================================================================
// 2. CONTROLE DE TELAS (DWIN) - Altere para bater com seu DGUS
// ====================================================================
#define TELA_PRINCIPAL  0  
#define TELA_SECUNDARIA 1  
#define TELA_BOX        2  

volatile uint8_t telaAtual = TELA_PRINCIPAL;
volatile bool forcarMudancaTela = true; 

// ====================================================================
// 3. ESTRUTURA DE DADOS UNIFICADA 
// ====================================================================
struct TelemetriaGlobal {
    uint32_t timestamp;
    
    // TX 1 (CVT)
    uint16_t rpm;
    float velocidade, tempCVT;

    // RECU
    float vBat, presTras, tempBat, perT, perF;

    // FECU
    float pedalFreio, presDiant, presCM;
    float accX, accY, accZ; // Eixo Z adicionado aqui
    float v_LF, v_RF, acionamentoDif;
} dados;

QueueHandle_t filaSD;
SemaphoreHandle_t mutexDados;
File dataFile;
char nomeArquivo[30];

// Instâncias de Hardware
TinyGsm modem(Serial1);
TinyGsmClient gsmClient(modem);
PubSubClient mqttClient(gsmClient);
MCP_CAN CAN0(CAN_CS);
SPIClass sdSPI(VSPI);

// Configurações MQTT e Rede
const char apn[] = "claro.com.br";
const char* mqtt_server = "72.60.141.159";
const int mqtt_port = 1883;
const char* mqtt_user = "imperador_mqtt";
const char* mqtt_pass = "imperador25";
const char* topic_telemetry = "imperador/telemetria";
const char* topic_command = "imperador/comandos/box";

// ====================================================================
// 4. PROTÓTIPOS DE FUNÇÕES
// ====================================================================
void vTaskCAN(void *pvParameters);
void vTaskModem(void *pvParameters);
void vTaskSD(void *pvParameters);
void vTaskDWIN(void *pvParameters);
void mqttCallback(char* topic, byte* payload, unsigned int length);

// ====================================================================
// SETUP
// ====================================================================
void setup() {
    Serial.begin(921600);
    Serial.println("\n🔥 MECU INICIALIZANDO (FreeRTOS) 🔥");

    pinMode(PIN_BOTAO_PAINEL, INPUT_PULLUP);
    mutexDados = xSemaphoreCreateMutex();
    
    // Inicia Seriais
    Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX); 
    Serial2.begin(9600, SERIAL_8N1, DWIN_RX, DWIN_TX);     

    // Inicia CAN
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println("❌ Erro Crítico: Falha no MCP2515 (CAN)!");
        delay(3000);
    } else {
        CAN0.setMode(MCP_NORMAL);
        Serial.println("✅ CAN Inicializada.");
    }

    // Inicia SD Incremental
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, sdSPI)) {
        Serial.println("❌ Erro Crítico: Falha no Cartão SD!");
    } else {
        int n = 1;
        while (n < 1000) {
            sprintf(nomeArquivo, "/MECU_%d.csv", n);
            if (!SD.exists(nomeArquivo)) break;
            n++;
        }
        dataFile = SD.open(nomeArquivo, FILE_WRITE);
        if (dataFile) {
            // Cabeçalho completo
            dataFile.println("ms;rpm;vel;tCVT;vBat;pTras;tBat;perT;perF;pedF;pDiant;pCM;accX;accY;accZ;vLF;vRF;dif");
            dataFile.flush();
            Serial.printf("✅ SD Inicializado. Arquivo: %s\n", nomeArquivo);
        }
    }

    // Configura Watchdog (8 segundos)
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 8000, 
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
        .trigger_panic = true
    };
    esp_task_wdt_init(&twdt_config);

    // Criação da Fila do SD
    filaSD = xQueueCreate(100, sizeof(TelemetriaGlobal));

    if (filaSD != NULL) {
        // CORE 1: Hardware em Tempo Real
        xTaskCreatePinnedToCore(vTaskCAN, "CAN", 4096, NULL, 3, NULL, 1);
        xTaskCreatePinnedToCore(vTaskDWIN, "DWIN", 2048, NULL, 2, NULL, 1);

        // CORE 0: Comunicação Nuvem e SD
        xTaskCreatePinnedToCore(vTaskModem, "GSM", 8192, NULL, 1, NULL, 0);
        xTaskCreatePinnedToCore(vTaskSD, "SD", 4096, NULL, 2, NULL, 0);
    }
}

void loop() { vTaskDelete(NULL); }

// ====================================================================
// TAREFA CAN (Core 1)
// ====================================================================
void vTaskCAN(void *pvParameters) {
    long unsigned int rxId;
    unsigned char len, rxBuf[8];
    uint32_t lastLogTime = 0;
    
    esp_task_wdt_add(NULL);

    for (;;) {
        esp_task_wdt_reset();
        
        if (CAN0.checkReceive() == CAN_MSGAVAIL) {
            CAN0.readMsgBuf(&rxId, &len, rxBuf);
            
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            dados.timestamp = millis();
            
            if (len == 2) {
                int16_t valorInt = (rxBuf[0] << 8) | rxBuf[1];
                float valorFloat = (float)valorInt / 100.0f;

                switch (rxId) {
                    // TX 1
                    case 0x200: dados.rpm = (uint16_t)valorFloat; break; 
                    case 0x201: dados.velocidade = valorFloat; break;
                    case 0x202: dados.tempCVT = valorFloat; break;
                    
                    // RECU
                    case 0x300: dados.vBat = valorFloat; break;
                    case 0x301: dados.presTras = valorFloat; break;
                    case 0x303: dados.tempBat = valorFloat; break;
                    case 0x304: dados.perT = valorFloat; break;
                    case 0x305: dados.perF = valorFloat; break;

                    // FECU
                    case 0x400: dados.pedalFreio = valorFloat; break;
                    case 0x402: dados.presDiant = valorFloat; break;
                    case 0x403: dados.presCM = valorFloat; break;
                    case 0x404: dados.accX = valorFloat; break;
                    case 0x405: dados.accY = valorFloat; break;
                    case 0x406: dados.v_LF = valorFloat; break;
                    case 0x407: dados.v_RF = valorFloat; break;
                    case 0x408: dados.acionamentoDif = valorFloat; break;
                    case 0x409: dados.accZ = valorFloat; break; // <--- Eixo Z Adicionado!
                }
            }
            xSemaphoreGive(mutexDados);

            // Grava a 100Hz na fila
            if (millis() - lastLogTime >= 10) {
                xQueueSend(filaSD, &dados, 0);
                lastLogTime = millis();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// ====================================================================
// TAREFA PAINEL DWIN (Core 1)
// ====================================================================
void vTaskDWIN(void *pvParameters) {
    uint32_t ultimoTempoBotao = 0;

    for (;;) {
        // --- 1. Lógica do Botão com Debounce ---
        if (digitalRead(PIN_BOTAO_PAINEL) == LOW && (millis() - ultimoTempoBotao > 300)) {
            ultimoTempoBotao = millis();
            
            if (telaAtual == TELA_PRINCIPAL) telaAtual = TELA_SECUNDARIA;
            else if (telaAtual == TELA_SECUNDARIA) telaAtual = TELA_PRINCIPAL;
            else if (telaAtual == TELA_BOX) telaAtual = TELA_PRINCIPAL; // Piloto fecha aviso do BOX
            
            forcarMudancaTela = true;
        }

        // --- 2. Troca de Tela ---
        if (forcarMudancaTela) {
            byte frameTela[10] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, telaAtual};
            Serial2.write(frameTela, 10);
            forcarMudancaTela = false;
        }

        // --- 3. Atualiza Dados ---
        if (telaAtual != TELA_BOX) {
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            uint16_t rpmTela = dados.rpm;
            xSemaphoreGive(mutexDados);

            // Exemplo envio RPM (Endereço 0x3100)
            byte frameRpm[8] = {0x5A, 0xA5, 0x05, 0x82, 0x31, 0x00, (byte)(rpmTela >> 8), (byte)(rpmTela & 0xFF)};
            Serial2.write(frameRpm, 8);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}

// ====================================================================
// TAREFA SD CARD (Core 0)
// ====================================================================
void vTaskSD(void *pvParameters) {
    TelemetriaGlobal d;
    int ct = 0;
    
    for (;;) {
        if (xQueueReceive(filaSD, &d, portMAX_DELAY)) {
            if (dataFile) {
                dataFile.printf("%u;%u;%.1f;%.1f;%.1f;%.2f;%.1f;%.0f;%.0f;%.1f;%.2f;%.2f;%.2f;%.2f;%.2f;%.1f;%.1f;%.0f\n", 
                    d.timestamp, d.rpm, d.velocidade, d.tempCVT, d.vBat, 
                    d.presTras, d.tempBat, d.perT, d.perF, d.pedalFreio, 
                    d.presDiant, d.presCM, d.accX, d.accY, d.accZ, d.v_LF, d.v_RF, d.acionamentoDif);
                
                if (++ct >= 50) { // Flush a cada meio segundo (garante n perder dados na vibração)
                    dataFile.flush();
                    ct = 0;
                }
            }
        }
    }
}

// ====================================================================
// CALLBACK MQTT (Recebe comandos)
// ====================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) message += (char)payload[i];
    
    if (String(topic) == topic_command) {
        StaticJsonDocument<200> doc;
        if (!deserializeJson(doc, message)) {
            const char* command = doc["command"];
            if (String(command) == "PIT") {
                telaAtual = TELA_BOX;
                forcarMudancaTela = true;
            }
        }
    }
}

// ====================================================================
// TAREFA MODEM E MQTT (Core 0)
// ====================================================================
void vTaskModem(void *pvParameters) {
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);

    for (;;) {
        if (!modem.isNetworkConnected()) modem.waitForNetwork(10000);
        if (modem.isNetworkConnected() && !modem.isGprsConnected()) modem.gprsConnect(apn, "", "");

        if (modem.isGprsConnected() && !mqttClient.connected()) {
            String clientId = "MECU-" + String(random(0xffff), HEX);
            if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
                mqttClient.subscribe(topic_command);
            }
        } 
        
        if (mqttClient.connected()) {
            StaticJsonDocument<1024> doc; 
            
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            doc["rpm"] = dados.rpm;
            doc["vel"] = dados.velocidade;
            doc["tCVT"] = dados.tempCVT;
            doc["vBat"] = dados.vBat;
            doc["pTras"] = dados.presTras;
            doc["tBat"] = dados.tempBat;
            doc["perT"] = dados.perT;
            doc["perF"] = dados.perF;
            doc["pedF"] = dados.pedalFreio;
            doc["pDiant"] = dados.presDiant;
            doc["pCM"] = dados.presCM;
            doc["accX"] = dados.accX;
            doc["accY"] = dados.accY;
            doc["accZ"] = dados.accZ; // <--- Eixo Z Adicionado no JSON
            doc["vLF"] = dados.v_LF;
            doc["vRF"] = dados.v_RF;
            doc["dif"] = dados.acionamentoDif;
            xSemaphoreGive(mutexDados);

            char buffer[1024];
            size_t n = serializeJson(doc, buffer);
            mqttClient.publish(topic_telemetry, buffer, n);
            
            mqttClient.loop();
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // Envio a 5Hz
    }
}
