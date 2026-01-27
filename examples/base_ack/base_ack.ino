#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <stdlib.h>
#include <string.h>

#include <AcousticDeforestCore.h>

#define LORA_SS   18
#define LORA_RST  14
#define LORA_DIO0 26

#define LORA_FREQ 915E6

uint8_t current_sf = 7;

void handleSerial();
void printMenu();
void printStatus();

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!lora_link::init_radio(LORA_FREQ, LORA_SS, LORA_RST, LORA_DIO0, current_sf, 125E3, 5, 14,
                             true)) {
    Serial.println("LoRa.begin() falhou. Verifique pinos/placa.");
    while (true) {}
  }

  // MESMOS parâmetros do TX
  LoRa.receive();

  delay(1500);
  Serial.print("# BOOT base freq=");
  Serial.print((uint32_t)LORA_FREQ);
  Serial.print(" sf=");
  Serial.print(current_sf);
  Serial.print(" bw=125000 cr=4/5 crc=on");
  Serial.println();
  Serial.println("# RX pronto. Aguardando pacotes...");
  printMenu();
}

void loop() {
  handleSerial();

  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  lora_link::ReceivedPacket packet{};
  lora_link::ReceiveStatus status = lora_link::receive_status_alert(packet, packetSize);
  if (status == lora_link::ReceiveStatus::kSizeMismatch) {
    Serial.print("# Pacote descartado. Tamanho=");
    Serial.println(packetSize);
    return;
  }
  if (status != lora_link::ReceiveStatus::kOk) {
    Serial.println("# Pacote descartado. Erro ao decodificar.");
    return;
  }

  // 3) decodificar campos
  uint8_t devId = packet.message.dev_id;
  uint8_t msgType = packet.message.msg_type;
  uint32_t uptime_ms = packet.message.uptime_ms;
  uint16_t seq = packet.message.seq;
  uint8_t field8 = packet.message.field8;
  uint16_t battery_mV = packet.message.battery_mv;

  // 4) métricas do rádio
  int rssi_int = packet.rssi_int;
  float snr_f = packet.snr_f;

  // 5) imprimir HEX + campos (para auditoria)
  Serial.print("HEX: ");
  for (size_t i = 0; i < protocol::STATUS_ALERT_SIZE; i++) {
    if (packet.raw[i] < 16) Serial.print('0');
    Serial.print(packet.raw[i], HEX);
    Serial.print(' ');
  }

  // Validação  do tipo
  if (msgType != protocol::MSG_STATUS && msgType != protocol::MSG_ALERT) {
    Serial.print("# MsgType inesperado=0x");
    Serial.println(msgType, HEX);
    while (LoRa.available()) LoRa.read(); // drena
    return;
  }

  Serial.print("| DevID=");
  Serial.print(devId);
  Serial.print(" Seq=");
  Serial.print(seq);
  Serial.print(" Uptime_ms=");
  Serial.print(uptime_ms);

  if (msgType == protocol::MSG_STATUS) {
    Serial.print("STATUS ");
    Serial.print("Flags=0x");
    Serial.print(field8, HEX);
  } else {
    Serial.print("ALERT ");
    Serial.print("EventClass=");
    Serial.print(field8);
  }

  Serial.print(" Battery_mV=");
  Serial.print(battery_mV);
  Serial.print(" | RSSI=");
  Serial.print(rssi_int);
  Serial.print(" dBm | SNR=");
  Serial.print(snr_f);
  Serial.print(" dB -> ACK(5B) enviado: ");

  uint8_t ack_raw[protocol::ACK_SIZE];
  lora_link::send_ack_for_seq(seq, rssi_int, snr_f, ack_raw);

  for (size_t i = 0; i < protocol::ACK_SIZE; i++) {
    if (ack_raw[i] < 16) Serial.print('0');
    Serial.print(ack_raw[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

void handleSerial() {
  static char cmd_buf[64];
  static uint8_t cmd_len = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd_len > 0) {
        cmd_buf[cmd_len] = '\0';
        char *line = cmd_buf;
        while (*line == ' ') line++;
        char *token = strtok(line, " ");
        if (token) {
          if (strcmp(token, "sf") == 0) {
            char *value = strtok(nullptr, " ");
            if (!value) {
              Serial.println("# usage: sf <7..12>");
            } else {
              int sf = atoi(value);
              if (sf >= 7 && sf <= 12) {
                current_sf = (uint8_t)sf;
                LoRa.setSpreadingFactor(current_sf);
                LoRa.receive();
                Serial.print("# sf ");
                Serial.println(current_sf);
              } else {
                Serial.println("# sf invalido");
              }
            }
          } else if (strcmp(token, "help") == 0) {
            printMenu();
          } else if (strcmp(token, "status") == 0) {
            printStatus();
          } else {
            Serial.println("# comando desconhecido");
          }
        }
        cmd_len = 0;
      }
      continue;
    }

    if (cmd_len < (sizeof(cmd_buf) - 1)) {
      cmd_buf[cmd_len++] = c;
    }
  }
}

void printMenu() {
  Serial.println("# MENU base");
  Serial.println("# comandos:");
  Serial.println("#  help         -> mostrar menu");
  Serial.println("#  status       -> mostrar configuracao");
  Serial.println("#  sf <7..12>   -> alterar spreading factor");
}

void printStatus() {
  Serial.print("# STATUS sf=");
  Serial.print(current_sf);
  Serial.print(" freq=");
  Serial.print((uint32_t)LORA_FREQ);
  Serial.println(" bw=125000 cr=4/5 crc=on");
}
