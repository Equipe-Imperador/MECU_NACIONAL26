#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>

// --- Bibliotecas MQTT e GSM ---
#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ====================================================================
// 1. DEFINIÇÕES DE PINOS
// ====================================================================
#define PIN_BOTAO_PAINEL 32
#define DWIN_TX 26 
#define DWIN_RX 25 
#define MODEM_TX 17
#define MODEM_RX 16
#define CAN_CS 15
#define CAN_SCK 14
#define CAN_MISO 12
#define CAN_MOSI 13
#define SD_CS 5
#define SD_SCK 18
#define SD_MISO 19
#define SD_MOSI 23

#define TELA_PRINCIPAL  0  
#define TELA_SECUNDARIA 1  
#define TELA_BOX        2  

// ====================================================================
// 2. VARIÁVEIS GLOBAIS E ESTRUTURAS
// ====================================================================
struct TelemetriaGlobal {
    uint32_t timestamp;
    uint16_t rpm;
    float velocidade, tempCVT, vBat, presTras, tempBat, perT, perF, pedalFreio, presDiant, presCM, accX, accY, accZ, v_LF, v_RF, acionamentoDif;
} dados;

volatile uint8_t telaAtual = TELA_PRINCIPAL;
volatile bool forcarMudancaTela = true; 
bool sdOk = false;
bool canOk = false;

QueueHandle_t filaSD;
SemaphoreHandle_t mutexDados;
File dataFile;
char nomeArquivo[30];

// Instâncias
TinyGsm modem(Serial1);
TinyGsmClient gsmClient(modem);
PubSubClient mqttClient(gsmClient);
MCP_CAN CAN0(CAN_CS);
SPIClass sdSPI(HSPI); // Barramento isolado para o SD não travar a CAN

const char apn[] = "claro.com.br";
const char* mqtt_server = "72.60.141.159";
const int mqtt_port = 1883;
const char* mqtt_user = "imperador_mqtt";
const char* mqtt_pass = "imperador25";
const char* topic_telemetry = "imperador/telemetria";
const char* topic_command = "imperador/comandos/box";

// Protótipos
void vTaskCAN(void *pvParameters);
void vTaskModem(void *pvParameters);
void vTaskSD(void *pvParameters);
void vTaskDWIN(void *pvParameters);
void simularDadosFicticios();

// ====================================================================
// SETUP
// ====================================================================
void setup() {
    Serial.begin(921600);
    delay(2000); 
    Serial.println("\n--- MECU INICIALIZANDO (Software Estável) ---");

    pinMode(PIN_BOTAO_PAINEL, INPUT_PULLUP);
    mutexDados = xSemaphoreCreateMutex();
    
    // Inicia Seriais
    Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX); 
    Serial2.begin(9600, SERIAL_8N1, DWIN_RX, DWIN_TX);     

    // 1. Inicia CAN
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        CAN0.setMode(MCP_NORMAL);
        canOk = true;
        Serial.println("[OK] CAN Conectada.");
    } else {
        Serial.println("[AVISO] CAN não detetada.");
    }

    // 2. Inicia SD
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (SD.begin(SD_CS, sdSPI)) {
        sdOk = true;
        int n = 1;
        while (n < 1000) {
            sprintf(nomeArquivo, "/MECU_%d.csv", n);
            if (!SD.exists(nomeArquivo)) break;
            n++;
        }
        dataFile = SD.open(nomeArquivo, FILE_WRITE);
        if (dataFile) {
            dataFile.println("ms;rpm;vel;tCVT;vBat;pTras;tBat;perT;perF;pedF;pDiant;pCM;accX;accY;accZ;vLF;vRF;dif");
            dataFile.flush();
            Serial.printf("[OK] SD pronto: %s\n", nomeArquivo);
        }
    } else {
        Serial.println("[AVISO] SD Card não detetado.");
    }

    // Cria as filas e tarefas
    filaSD = xQueueCreate(100, sizeof(TelemetriaGlobal));

    if (filaSD != NULL) {
        // CORE 1 (Sensores e Tela)
        xTaskCreatePinnedToCore(vTaskCAN, "CAN", 4096, NULL, 3, NULL, 1);
        xTaskCreatePinnedToCore(vTaskDWIN, "DWIN", 2048, NULL, 2, NULL, 1);
        
        // CORE 0 (Nuvem e SD)
        // -> O SEGREDO AQUI: Prioridade do Modem agora é 0!
        xTaskCreatePinnedToCore(vTaskModem, "GSM", 10240, NULL, 0, NULL, 0); 
        xTaskCreatePinnedToCore(vTaskSD, "SD", 4096, NULL, 2, NULL, 0);
        
        Serial.println("[SISTEMA] FreeRTOS em execução.");
    }
}

// O loop principal agora só descansa (para não acionar o WDT do Core 1)
void loop() { vTaskDelay(portMAX_DELAY); }

// ====================================================================
// FUNÇÃO DE SIMULAÇÃO (Dados fictícios a 1Hz)
// ====================================================================
void simularDadosFicticios() {
    xSemaphoreTake(mutexDados, portMAX_DELAY);
    dados.timestamp = millis();
    dados.rpm = random(1800, 3600);
    dados.velocidade = (float)random(150, 500) / 10.0f;
    dados.tempCVT = (float)random(400, 950) / 10.0f;
    dados.vBat = (float)random(115, 138) / 10.0f;
    dados.pedalFreio = (float)random(0, 100);
    xSemaphoreGive(mutexDados);
}

// ====================================================================
// TAREFA MODEM E MQTT (Core 0)
// ====================================================================
void vTaskModem(void *pvParameters) {
    mqttClient.setServer(mqtt_server, mqtt_port);
    uint32_t lastSimTime = 0;

    for (;;) {
        // Simulação rodando perfeitamente independente da internet
        if (millis() - lastSimTime >= 1000) {
            simularDadosFicticios();
            lastSimTime = millis();
        }

        // Lógica Não-Bloqueante
        if (!modem.isNetworkConnected()) {
            Serial.print("[MODEM] Procurando rede... ");
            // Tenta se comunicar por no máximo 1 segundo
            if (modem.waitForNetwork(1000)) {
                Serial.println("CONECTADO!");
            } else {
                Serial.println("Ainda não.");
            }
        } 
        else if (!modem.isGprsConnected()) {
            Serial.println("[MODEM] Conectando GPRS (Internet)...");
            modem.gprsConnect(apn, "", "");
        } 
        else if (!mqttClient.connected()) {
            Serial.println("[MQTT] Conectando ao Servidor...");
            String clientId = "MECU_BAJA_" + String(random(0xffff), HEX);
            mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass);
        }

        // Envia os dados caso a conexão seja bem-sucedida
        if (mqttClient.connected()) {
            StaticJsonDocument<512> doc;
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            doc["rpm"] = dados.rpm;
            doc["vel"] = dados.velocidade;
            doc["vBat"] = dados.vBat;
            doc["tCVT"] = dados.tempCVT;
            xSemaphoreGive(mutexDados);

            char buffer[512];
            size_t n = serializeJson(doc, buffer);
            mqttClient.publish(topic_telemetry, buffer, n);
            mqttClient.loop();
        }

        // Descanso vital para o FreeRTOS rodar a tarefa oculta (IDLE)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ====================================================================
// TAREFA CAN (Core 1)
// ====================================================================
void vTaskCAN(void *pvParameters) {
    long unsigned int rxId;
    unsigned char len, rxBuf[8];
    uint32_t lastLogTime = 0;

    for (;;) {
        if (canOk && CAN0.checkReceive() == CAN_MSGAVAIL) {
            CAN0.readMsgBuf(&rxId, &len, rxBuf);
            // Processamento CAN
        }

        if (millis() - lastLogTime >= 100) {
            xQueueSend(filaSD, &dados, 0);
            lastLogTime = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// ====================================================================
// TAREFA PAINEL DWIN (Core 1)
// ====================================================================
void vTaskDWIN(void *pvParameters) {
    for (;;) {
        // Lógica do Botão
        if (digitalRead(PIN_BOTAO_PAINEL) == LOW) {
            telaAtual = (telaAtual + 1) % 3;
            forcarMudancaTela = true;
            vTaskDelay(pdMS_TO_TICKS(300));
        }

        // Muda a Tela
        if (forcarMudancaTela) {
            byte frameTela[10] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, telaAtual};
            Serial2.write(frameTela, 10);
            forcarMudancaTela = false;
        }

        // Envia RPM Simulada para a Tela
        xSemaphoreTake(mutexDados, portMAX_DELAY);
        uint16_t rpmParaTela = dados.rpm;
        xSemaphoreGive(mutexDados);

        byte frameRpm[8] = {0x5A, 0xA5, 0x05, 0x82, 0x31, 0x00, (byte)(rpmParaTela >> 8), (byte)(rpmParaTela & 0xFF)};
        Serial2.write(frameRpm, 8);

        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
}

// ====================================================================
// TAREFA SD CARD (Core 0)
// ====================================================================
void vTaskSD(void *pvParameters) {
    TelemetriaGlobal d;
    for (;;) {
        if (xQueueReceive(filaSD, &d, portMAX_DELAY)) {
            if (sdOk && dataFile) {
                dataFile.printf("%u;%u;%.1f;%.1f;%.1f\n", d.timestamp, d.rpm, d.velocidade, d.tempCVT, d.vBat);
                static int flushCt = 0;
                if (++flushCt >= 20) {
                    dataFile.flush();
                    flushCt = 0;
                }
            }
        }
    }
}
