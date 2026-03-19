# MECU - Unidade de Controle e Telemetria

A **MECU** (Main Electronic Control Unit) é o cérebro de telemetria e interface do veículo. Baseada em um **ESP32** operando sob o sistema operacional de tempo real **FreeRTOS**, a MECU consolida dados vitais do barramento CAN, registra logs físicos, transmite telemetria via rede celular (MQTT) e gerencia um painel industrial inteligente DWIN.

## Funcionalidades Principais

* **Leitura CAN Bus:** Coleta em tempo real de sensores dinâmicos (RPM, velocidades, aceleração de 3 eixos, pressões, esterçamento e temperaturas) via módulo MCP2515.
* **Datalogger de Alta Performance:** Gravação contínua em Cartão SD (formato `.csv`) protegida por filas do FreeRTOS contra latência de escrita (*write latency*).
* **Telemetria Remota (IoT):** Transmissão de dados via 4G/GPRS utilizando modem SIM7600 e protocolo MQTT.
* **Comunicação Bidirecional (Box):** Recebimento de comandos remotos via servidor para acionamento do Diferencial, Buzina e troca de telas de aviso (Pit/Pista).
* **Interface Homem-Máquina (HMI):** Controle dinâmico de um display inteligente DWIN, com otimização de barramento Serial e renderização de múltiplas telas.
* **Sincronismo de Tempo Real (NTP):** Extração de horário da rede de telefonia celular para atualização do relógio do painel, dispensando bateria de RTC dedicada.

---

## Arquitetura de Hardware (Pinout ESP32)

| Componente | Pino ESP32 | Função |
| :--- | :---: | :--- |
| **CAN (MCP2515)** | 14, 12, 13, 15 | SCK, MISO, MOSI, CS |
| **CAN Interrupt** | 27 | INT |
| **Cartão SD** | 18, 19, 23, 5 | SCK, MISO, MOSI, CS (HSPI) |
| **Modem SIM7600** | 16, 17 | RX, TX (Serial1) |
| **Painel DWIN** | 25, 26 | RX, TX (Serial2) |
| **Botão de Painel** | 32 | Troca física de telas (Input Pullup) |

---

## Arquitetura de Software (FreeRTOS)

O firmware foi projetado em tarefas (Tasks) paralelas isoladas para evitar código bloqueante:

1. **vTaskCAN** *(Prioridade 3 - Crítica)*: Varre o buffer do controlador CAN. Converte os 2 bytes recebidos em `float` (dividindo por 100) e abastece a `struct` global de dados. Envia snapshots para a fila do SD a cada 10ms.
2. **vTaskDWIN** *(Prioridade 2 - Alta)*: Gerencia a interface gráfica. Monitora o botão físico, força atualizações de página e envia dados aos registradores (VPs) apenas quando necessários para a tela ativa, evitando saturar o barramento UART a 9600 bps.
3. **vTaskSD** *(Prioridade 2 - Alta)*: Consome a `filaSD` e grava no cartão a cada 50 leituras.
4. **vTaskModem** *(Prioridade 0 - Background)*: Gerencia a máquina de estados do GPRS e MQTT. Formata os dados em JSON, publica no tópico de telemetria a cada 200ms e assina tópicos de comando de box. Requisita e calibra o horário do relógio a cada 60s.

*O projeto utiliza Mutex (`xSemaphoreCreateMutex`) para proteger a leitura/escrita da variável global de dados entre as diferentes tarefas.*

---

## Mapeamento de Variáveis DWIN (DGUS)

Para manter a precisão dos dados em um display que lê números inteiros de 16-bits (`int16_t`), os dados fracionados são multiplicados no ESP32. Configure as **Data Variables** no software DGUS conforme a tabela abaixo:

| Variável | VP (Hex) | Telas | Ajuste no DGUS (Inteiro / Decimal) | Fator MECU |
| :--- | :---: | :--- | :---: | :---: |
| **Velocidade** | `0x3000` | Todas | 3 / 0 | x1 |
| **RPM** | `0x3010` | Todas | 4 / 0 | x1 |
| **Temp CVT** | `0x3020` | Princ, Debug | 3 / 0 | x1 |
| **Tensão (vBat)** | `0x3030` | Debug | 2 / 1 | x10 |
| **Temp Bateria** | `0x3040` | Debug | 3 / 0 | x1 |
| **Status Diferencial** | `0x3050` | Debug | 1 / 0 | x1 |
| **Ícone Diferencial** | `0x4500` | Principal | 1 / 0 | 0 ou 1 |
| **Pedal Freio** | `0x3060` | Debug | 3 / 0 | x1 |
| **Pressão Dianteira** | `0x3070` | Debug | 3 / 0 | x1 |
| **Pressão Traseira** | `0x3080` | Debug | 3 / 0 | x1 |
| **Acelerômetro X** | `0x3090` | Debug | 1 / 2 | x100 |
| **Acelerômetro Y** | `0x3100` | Debug | 1 / 2 | x100 |
| **Acelerômetro Z** | `0x3110` | Debug | 1 / 2 | x100 |
| **Vel. Direita (RF)**| `0x3120` | Debug | 3 / 0 | x1 |
| **Vel. Esquerda (LF)**| `0x3130` | Debug | 3 / 0 | x1 |
| **Esterçamento** | `0x3140` | Debug | 2 / 1 | x10 |
| **Relógio (Hora)** | `0x4600` | Princ, Box | 2 / 0 | x1 |
| **Relógio (Minuto)**| `0x4610` | Princ, Box | 2 / 0 | x1 |

---

## Comunicação MQTT

### Tópicos
* **Publicação (Telemetria):** `imperador/telemetria`
* **Assinatura (Comandos Box):** `imperador/comandos/box`

### Payload JSON de Envio (Exemplo)

    {
      "rpm": 4500,
      "vel": 65.5,
      "tCVT": 80.0,
      "vBat": 12.4,
      "pTras": 15.2,
      "tBat": 35.0,
      "perT": 0,
      "perF": 0,
      "pedF": 10.0,
      "pDiant": 14.8,
      "estrc": 15.5,
      "accX": 1.25,
      "accY": -0.50,
      "accZ": 0.98,
      "vLF": 65.1,
      "vRF": 65.8,
      "corrDif": true
    }

### Payload JSON de Recebimento (Comandos)
* Chamar para o Box: `{"command": "PIT"}`
* Liberar Pista: `{"command": "PISTA"}`
* Travar Diferencial: `{"acionamentoDif": true}`
* Acionar Buzina: `{"acionamentoBuzina": true}`

---

## Datalogger (Cartão SD)

O sistema de gravação física foi projetado para suportar as vibrações e as latências típicas de cartões SD sem perder pacotes de telemetria da rede CAN.

### Dinâmica de Gravação
1. **Fila do FreeRTOS (`filaSD`):** Foi alocada uma fila com capacidade para **200 posições** na memória RAM do ESP32. Se o cartão SD engasgar (Write Latency) para trocar de setor de memória, a fila absorve o pico de dados enviados pela CAN a cada 10ms, evitando a perda de pacotes.
2. **Bufferização e Flush:** A tarefa `vTaskSD` puxa os dados da fila, escreve no arquivo e executa o comando `dataFile.flush()` a cada **50 leituras** (~500ms). Isso garante que, caso o carro sofra um corte abrupto de energia, no máximo 0,5 segundos de dados não serão salvos.
3. **Nomenclatura Automática:** A cada inicialização, a MECU busca o próximo número disponível na raiz do SD, criando arquivos sequenciais no formato `MECU_1.csv`, `MECU_2.csv`, etc.

### Formato do Arquivo (.csv)
O arquivo é gerado com separador ponto-e-vírgula (`;`), ideal para importação nativa no Excel e em softwares de análise de telemetria.

**Cabeçalho e Exemplo de Linha:**

    ms;rpm;vel;tCVT;vBat;pTras;tBat;perT;perF;pedF;pDiant;estrc;accX;accY;accZ;vLF;vRF;corrDif
    10450;4500;65.5;80.0;12.4;15.2;35.0;0;0;10.0;14.8;15.5;1.25;-0.50;0.98;65.1;65.8;1
