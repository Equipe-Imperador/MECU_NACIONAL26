/*
    MECU - Gerenciamento de Telemetria, CAN e DWIN
    Versão Unificada: Lógica CAN V2 + Dual Button V1 + DWIN 32-bit
*/

#include <mcp_can.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <esp_task_wdt.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ====================================================================
// 1. DEFINIÇÕES DE PINOS E ESTRUTURAS
// ====================================================================
#define PIN_BOTAO_PAINEL 32
#define PIN_VCC_BOTAO 33     
#define PIN_BOTAO_TELA3 39   

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
#define TELA_SECUNDARIA 0x02  // Tela de Debug
#define TELA_BOX        0x01  

volatile uint8_t telaAtual = TELA_PRINCIPAL;
volatile bool forcarMudancaTela = true; 

volatile uint8_t horaAtual = 0;
volatile uint8_t minutoAtual = 0;
uint32_t ultimoSincronismoRelogio = 0;

struct TelemetriaGlobal {
    uint32_t timestamp;
    uint16_t rpm;
    float velocidade, tempCVT, v_LF, v_RF;
    float vBat, presTras, tempBat, perT, perF;
    float pedalFreio, presDiant, estercamento, accX, accY, accZ; 
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

// Protótipos
void vTaskCAN(void *pvParameters);
void vTaskModem(void *pvParameters);
void vTaskSD(void *pvParameters);
void vTaskDWIN(void *pvParameters);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void enviarMsgCAN(uint32_t id, float valor);

// ====================================================================
// FUNÇÃO DWIN (32-bit / 4 bytes) - Compatível com VPs de Longa Distância
// ====================================================================
void enviarValorDWIN(uint16_t vp, int32_t valor) {
    uint32_t v = (uint32_t)valor;
    byte frame[10] = {
        0x5A, 0xA5, 0x07, 0x82, 
        (byte)(vp >> 8), (byte)(vp & 0xFF), 
        (byte)(v >> 24), (byte)(v >> 16),
        (byte)(v >> 8), (byte)(v & 0xFF)
    };
    Serial2.write(frame, 10);
    vTaskDelay(pdMS_TO_TICKS(2));
}

// ====================================================================
// 2. SETUP
// ====================================================================
void setup() {
    Serial.begin(921600);
    delay(2000);
    Serial.println("\n [MECU] INICIANDO SISTEMA UNIFICADO...");

    pinMode(PIN_BOTAO_PAINEL, INPUT_PULLUP);
    pinMode(PIN_VCC_BOTAO, OUTPUT);
    digitalWrite(PIN_VCC_BOTAO, HIGH); 
    pinMode(PIN_BOTAO_TELA3, INPUT); // Pull-down externo necessário

    mutexDados = xSemaphoreCreateMutex();
    
    Serial1.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX); 
    Serial2.begin(9600, SERIAL_8N1, DWIN_RX, DWIN_TX);     

    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println(" !!! ERRO CAN !!!");
    } else {
        CAN0.setMode(MCP_NORMAL);
    }

    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (SD.begin(SD_CS, sdSPI)) {
        int n = 1;
        while (n < 1000) {
            sprintf(nomeArquivo, "/MECU_%d.csv", n);
            if (!SD.exists(nomeArquivo)) break;
            n++;
        }
        dataFile = SD.open(nomeArquivo, FILE_WRITE);
        if (dataFile) {
            dataFile.println("ms;rpm;vel;tCVT;vBat;pTras;tBat;perT;perF;pedF;pDiant;estrc;accX;accY;accZ;vLF;vRF;corrDif");
            dataFile.flush();
        }
    }

    esp_task_wdt_config_t twdt_config = { .timeout_ms = 8000, .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, .trigger_panic = true };
    esp_task_wdt_deinit(); 
    esp_task_wdt_init(&twdt_config);

    filaSD = xQueueCreate(200, sizeof(TelemetriaGlobal));
    if (filaSD != NULL) {
        xTaskCreatePinnedToCore(vTaskCAN, "CAN", 4096, NULL, 3, NULL, 1);
        xTaskCreatePinnedToCore(vTaskDWIN, "DWIN", 4096, NULL, 2, NULL, 1);
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
// 3. TAREFAS
// ====================================================================

void vTaskCAN(void *pvParameters) {
    long unsigned int rxId;
    unsigned char len, rxBuf[8];
    uint32_t lastLogTime = 0;
    esp_task_wdt_add(NULL);

    for (;;) {
        esp_task_wdt_reset();
        while (CAN0.checkReceive() == CAN_MSGAVAIL) {
            CAN0.readMsgBuf(&rxId, &len, rxBuf);
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            dados.timestamp = millis();
            if (len == 2) {
                int16_t valorInt = (rxBuf[0] << 8) | rxBuf[1];
                float valorFloat = (float)valorInt / 100.0f;
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
                    case 0x403: dados.estercamento = valorFloat; break;
                    case 0x404: dados.accX = valorFloat; break;
                    case 0x405: dados.accY = valorFloat; break;
                    case 0x406: dados.accZ = valorFloat; break; 
                    case 0x407: dados.correnteDif = (valorInt > 0); break; 
                }
            }
            xSemaphoreGive(mutexDados);
        }

        if (millis() - lastLogTime >= 20) {
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            TelemetriaGlobal copia = dados;
            xSemaphoreGive(mutexDados);
            xQueueSend(filaSD, &copia, 0);
            lastLogTime = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

void vTaskDWIN(void *pvParameters) {
    uint32_t ultBotao1 = 0, ultBotao2 = 0;
    bool estadoAnt1 = HIGH, estadoAnt2 = LOW;
    const uint32_t DEBOUNCE = 150;

    for (;;) {
        // Lógica Botão 1 (Troca Principal/Debug)
        bool leit1 = digitalRead(PIN_BOTAO_PAINEL);
        if (leit1 == LOW && estadoAnt1 == HIGH && (millis() - ultBotao1 > DEBOUNCE)) {
            ultBotao1 = millis();
            telaAtual = (telaAtual == TELA_PRINCIPAL) ? TELA_SECUNDARIA : TELA_PRINCIPAL;
            forcarMudancaTela = true;
        }
        estadoAnt1 = leit1;

        // Lógica Botão 2 (Tela 3)
        bool leit2 = digitalRead(PIN_BOTAO_TELA3);
        if (leit2 == HIGH && estadoAnt2 == LOW && (millis() - ultBotao2 > DEBOUNCE)) {
            ultBotao2 = millis();
            telaAtual = (telaAtual == TELA_BOX) ? TELA_PRINCIPAL : TELA_BOX;
            forcarMudancaTela = true;
        }
        estadoAnt2 = leit2;

        if (forcarMudancaTela) {
            byte frameTela[10] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, telaAtual};
            Serial2.write(frameTela, 10);
            forcarMudancaTela = false;
        }

        xSemaphoreTake(mutexDados, portMAX_DELAY);
        TelemetriaGlobal d = dados; 
        xSemaphoreGive(mutexDados);

        // Envio Comum (RPM e VEL)
        enviarValorDWIN(0x3000, (int32_t)d.velocidade);
        enviarValorDWIN(0x3010, (int32_t)d.rpm);

        if (telaAtual == TELA_PRINCIPAL || telaAtual == TELA_BOX) {
            enviarValorDWIN(0x4600, (int32_t)horaAtual);
            enviarValorDWIN(0x4610, (int32_t)minutoAtual);
            enviarValorDWIN(0x4500, (int32_t)(d.correnteDif ? 1 : 0));
            enviarValorDWIN(0x3020, (int32_t)d.tempCVT);
        }

        if (telaAtual == TELA_SECUNDARIA) {
            enviarValorDWIN(0x3020, (int32_t)d.tempCVT);
            enviarValorDWIN(0x3030, (int32_t)(d.vBat * 10)); 
            enviarValorDWIN(0x3040, (int32_t)d.tempBat);
            enviarValorDWIN(0x3060, (int32_t)d.pedalFreio);
            enviarValorDWIN(0x3070, (int32_t)d.presDiant);
            enviarValorDWIN(0x3080, (int32_t)d.presTras);
            
            // Perinhas (VPs solicitados: 0x3140 e 0x3150)
            enviarValorDWIN(0x3140, (int32_t)((d.perT > 0.5f) ? 1 : 0));
            enviarValorDWIN(0x3150, (int32_t)((d.perF > 0.5f) ? 1 : 0));
            
            // Esterçamento (VP 0x3160)
            enviarValorDWIN(0x3160, (int32_t)(d.estercamento * 10));
            
            enviarValorDWIN(0x3090, (int32_t)(d.accX * 100)); 
            enviarValorDWIN(0x3100, (int32_t)(d.accY * 100));
            enviarValorDWIN(0x3110, (int32_t)(d.accZ * 100));
        }
        vTaskDelay(pdMS_TO_TICKS(150)); 
    }
}

void vTaskSD(void *pvParameters) {
    TelemetriaGlobal d;
    int ct = 0;
    for (;;) {
        if (xQueueReceive(filaSD, &d, portMAX_DELAY)) {
            if (dataFile) {
                dataFile.printf("%u;%u;%.1f;%.1f;%.1f;%.1f;%.1f;%.0f;%.0f;%.1f;%.1f;%.1f;%.2f;%.2f;%.2f;%.1f;%.1f;%d\n", 
                    d.timestamp, d.rpm, d.velocidade, d.tempCVT, d.vBat, d.presTras, d.tempBat, d.perT, d.perF, d.pedalFreio, d.presDiant, d.estercamento, d.accX, d.accY, d.accZ, d.v_LF, d.v_RF, d.correnteDif);
                if (++ct >= 50) { dataFile.flush(); ct = 0; }
            }
        }
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) message += (char)payload[i];
    if (String(topic) == topic_command) {
        StaticJsonDocument<200> doc;
        if (!deserializeJson(doc, message)) {
            if (doc.containsKey("command")) {
                const char* cmd = doc["command"];
                if (String(cmd) == "PIT") { telaAtual = TELA_BOX; forcarMudancaTela = true; }
                if (String(cmd) == "PISTA") { telaAtual = TELA_PRINCIPAL; forcarMudancaTela = true; }
            }
            if (doc.containsKey("acionamentoDif")) enviarMsgCAN(0x500, doc["acionamentoDif"] ? 1.0f : 0.0f);
        }
    }
}

void vTaskModem(void *pvParameters) {
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);
    for (;;) {
        if (!modem.isNetworkConnected()) modem.waitForNetwork(3000);
        else if (!modem.isGprsConnected()) modem.gprsConnect(apn, "", "");
        else if (!mqttClient.connected()) {
            String id = "MECU-" + String(random(0xffff), HEX);
            if (mqttClient.connect(id.c_str(), mqtt_user, mqtt_pass)) mqttClient.subscribe(topic_command);
        }
        
        if (mqttClient.connected()) {
            if (millis() - ultimoSincronismoRelogio > 60000 || ultimoSincronismoRelogio == 0) {
                int a, m, d, h, mi, s; float f;
                if (modem.getNetworkTime(&a, &m, &d, &h, &mi, &s, &f)) {
                    int hc = h - 3; if (hc < 0) hc += 24;
                    horaAtual = hc; minutoAtual = mi;
                    ultimoSincronismoRelogio = millis();
                }
            }
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            TelemetriaGlobal d = dados;
            xSemaphoreGive(mutexDados);
            StaticJsonDocument<1024> doc;
            doc["rpm"] = d.rpm; doc["vel"] = d.velocidade; doc["tCVT"] = d.tempCVT;
            doc["vBat"] = d.vBat; doc["pTras"] = d.presTras;
            char buffer[1024];
            size_t n = serializeJson(doc, buffer);
            mqttClient.publish(topic_telemetry, buffer, n);
            mqttClient.loop();
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
