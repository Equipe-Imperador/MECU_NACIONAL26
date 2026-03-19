/*
    MECU - Gerenciamento de Telemetria e DWIN
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
#define TELA_SECUNDARIA 0x02  // Tela de Debug
#define TELA_BOX        0x01  

volatile uint8_t telaAtual = TELA_PRINCIPAL;
volatile bool forcarMudancaTela = true; 

// Variáveis Globais do Relógio (Extraídas do Modem)
volatile uint8_t horaAtual = 0;
volatile uint8_t minutoAtual = 0;
uint32_t ultimoSincronismoRelogio = 0;

struct TelemetriaGlobal {
    uint32_t timestamp;
    uint16_t rpm;
    float velocidade, tempCVT, v_LF, v_RF;
    float vBat, presTras, tempBat, perT, perF;
    float pedalFreio, presDiant, estercamento, accX, accY, accZ; // presCM substituida por estercamento
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
// FUNÇÃO AUXILIAR PARA O PAINEL DWIN
// ====================================================================
void enviarValorDWIN(uint16_t vp, int16_t valor) {
    byte frame[8] = {
        0x5A, 0xA5, 0x05, 0x82, 
        (byte)(vp >> 8), (byte)(vp & 0xFF), 
        (byte)(valor >> 8), (byte)(valor & 0xFF)
    };
    Serial2.write(frame, 8);
    vTaskDelay(pdMS_TO_TICKS(5)); // Respiro para não saturar o display
}

// ====================================================================
// 2. SETUP
// ====================================================================
void setup() {
    Serial.begin(921600);
    Serial.println("\n [MECU] INICIANDO SISTEMA...");

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
            // Cabeçalho atualizado: pTras mantida, estrc no lugar de pCM
            dataFile.println("ms;rpm;vel;tCVT;vBat;pTras;tBat;perT;perF;pedF;pDiant;estrc;accX;accY;accZ;vLF;vRF;corrDif");
            dataFile.flush();
        }
    }

    esp_task_wdt_config_t twdt_config = { .timeout_ms = 8000, .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, .trigger_panic = true };
    esp_task_wdt_deinit(); 
    esp_task_wdt_init(&twdt_config);

    // Fila do SD aumentada para 200 posições (proteção contra write latency do SD)
    filaSD = xQueueCreate(200, sizeof(TelemetriaGlobal));
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
                    case 0x403: dados.estercamento = valorFloat; break; // Substituiu a presCM
                    case 0x404: dados.accX = valorFloat; break;
                    case 0x405: dados.accY = valorFloat; break;
                    case 0x406: dados.accZ = valorFloat; break; 
                    case 0x407: dados.correnteDif = (valorInt > 0); break; 
                }
            }
            xSemaphoreGive(mutexDados);
        }

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
        // Lógica do botão
        if (digitalRead(PIN_BOTAO_PAINEL) == LOW && (millis() - ultimoTempoBotao > 300)) {
            ultimoTempoBotao = millis();
            if (telaAtual == TELA_PRINCIPAL) telaAtual = TELA_SECUNDARIA;
            else if (telaAtual == TELA_SECUNDARIA) telaAtual = TELA_PRINCIPAL;
            else if (telaAtual == TELA_BOX) telaAtual = TELA_PRINCIPAL;
            forcarMudancaTela = true;
        }
        
        // Força a mudança de tela
        if (forcarMudancaTela) {
            byte frameTela[10] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, 0x00, telaAtual};
            Serial2.write(frameTela, 10);
            forcarMudancaTela = false;
        }
        
        // Copia os dados
        xSemaphoreTake(mutexDados, portMAX_DELAY);
        TelemetriaGlobal d = dados; 
        xSemaphoreGive(mutexDados);

        // 1. DADOS GERAIS (Todas as telas)
        enviarValorDWIN(0x3000, (int16_t)d.velocidade);
        enviarValorDWIN(0x3010, (int16_t)d.rpm);
        
        // 2. HORÁRIO E DIFERENCIAL (Principal e Box)
        if (telaAtual == TELA_PRINCIPAL || telaAtual == TELA_BOX) {
            enviarValorDWIN(0x4600, horaAtual);
            enviarValorDWIN(0x4610, minutoAtual);
        }

        // 3. APENAS TELA PRINCIPAL (0x00)
        if (telaAtual == TELA_PRINCIPAL) {
            enviarValorDWIN(0x4500, d.correnteDif ? 1 : 0); 
            enviarValorDWIN(0x3020, (int16_t)d.tempCVT);    
        }

        // 4. APENAS TELA DEBUG (0x02)
        if (telaAtual == TELA_SECUNDARIA) {
            enviarValorDWIN(0x3020, (int16_t)d.tempCVT);
            enviarValorDWIN(0x3030, (int16_t)(d.vBat * 10)); 
            enviarValorDWIN(0x3040, (int16_t)d.tempBat);
            enviarValorDWIN(0x3050, d.correnteDif ? 1 : 0);
            enviarValorDWIN(0x3060, (int16_t)d.pedalFreio);
            enviarValorDWIN(0x3070, (int16_t)d.presDiant);
            
            // Aqui pTras assumiu o lugar de pCM e esterçamento ganhou um novo VP
            enviarValorDWIN(0x3080, (int16_t)d.presTras); 
            enviarValorDWIN(0x3140, (int16_t)(d.estercamento * 10)); // Multiplicado por 10 p/ 1 casa decimal
            
            enviarValorDWIN(0x3090, (int16_t)(d.accX * 100)); 
            enviarValorDWIN(0x3100, (int16_t)(d.accY * 100));
            enviarValorDWIN(0x3110, (int16_t)(d.accZ * 100));
            enviarValorDWIN(0x3120, (int16_t)d.v_RF); 
            enviarValorDWIN(0x3130, (int16_t)d.v_LF); 
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

// --- TAREFA SD ---
void vTaskSD(void *pvParameters) {
    TelemetriaGlobal d;
    int ct = 0;
    for (;;) {
        if (xQueueReceive(filaSD, &d, portMAX_DELAY)) {
            if (dataFile) {
                // Atualizado: presTras e estercamento no lugar correto
                dataFile.printf("%u;%u;%.1f;%.1f;%.1f;%.2f;%.1f;%.0f;%.0f;%.1f;%.2f;%.1f;%.2f;%.2f;%.2f;%.1f;%.1f;%d\n", 
                    d.timestamp, d.rpm, d.velocidade, d.tempCVT, d.vBat, 
                    d.presTras, d.tempBat, d.perT, d.perF, d.pedalFreio, 
                    d.presDiant, d.estercamento, d.accX, d.accY, d.accZ, d.v_LF, d.v_RF, d.correnteDif);
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
                if (String(command) == "PIT") { telaAtual = TELA_BOX; forcarMudancaTela = true; }
                if (String(command) == "PISTA") { telaAtual = TELA_PRINCIPAL; forcarMudancaTela = true; }
            }
            if (doc.containsKey("acionamentoDif")) {
                bool estadoDif = doc["acionamentoDif"];
                enviarMsgCAN(0x500, estadoDif ? 1.0f : 0.0f);
            }
            if (doc.containsKey("acionamentoBuzina")) {
                bool estadoBuzina = doc["acionamentoBuzina"];
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
            // Sincroniza o relógio a cada 60 segundos
            if (millis() - ultimoSincronismoRelogio > 60000 || ultimoSincronismoRelogio == 0) {
                String tempoRede = modem.getNetworkTime(); 
                int indiceVirgula = tempoRede.indexOf(',');
                
                if (indiceVirgula != -1 && tempoRede.length() >= indiceVirgula + 6) {
                    String strHora = tempoRede.substring(indiceVirgula + 1, indiceVirgula + 3);
                    String strMinuto = tempoRede.substring(indiceVirgula + 4, indiceVirgula + 6);
                    
                    int horaCorrigida = strHora.toInt() - 3; // Ajuste para UTC-3 (Horário de Brasília)
                    if (horaCorrigida < 0) horaCorrigida += 24; 
                    
                    horaAtual = (uint8_t)horaCorrigida;
                    minutoAtual = (uint8_t)strMinuto.toInt();
                }
                ultimoSincronismoRelogio = millis();
            }

            // Envia Telemetria MQTT
            TelemetriaGlobal d;
            xSemaphoreTake(mutexDados, portMAX_DELAY);
            d = dados;
            xSemaphoreGive(mutexDados);

            StaticJsonDocument<1024> doc; 
            doc["rpm"] = d.rpm; doc["vel"] = d.velocidade; doc["tCVT"] = d.tempCVT;
            doc["vBat"] = d.vBat; doc["pTras"] = d.presTras; doc["tBat"] = d.tempBat;
            doc["perT"] = d.perT; doc["perF"] = d.perF; doc["pedF"] = d.pedalFreio;
            doc["pDiant"] = d.presDiant; 
            doc["estrc"] = d.estercamento; // Substituiu doc["pCM"]
            doc["accX"] = d.accX; doc["accY"] = d.accY; doc["accZ"] = d.accZ; 
            doc["vLF"] = d.v_LF; doc["vRF"] = d.v_RF; doc["corrDif"] = d.correnteDif;

            char buffer[1024];
            size_t n = serializeJson(doc, buffer);
            mqttClient.publish(topic_telemetry, buffer, n);
            mqttClient.loop();
        }
        vTaskDelay(pdMS_TO_TICKS(200)); 
    }
}
