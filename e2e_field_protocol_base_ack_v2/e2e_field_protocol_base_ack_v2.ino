#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#include <AcousticDeforestCore.h>

#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

constexpr long LORA_FREQ = 915E6;
constexpr long LORA_BW = 125E3;
constexpr int LORA_CR = 5;
constexpr int LORA_TX_POWER = 14;
constexpr bool LORA_CRC = true;

uint8_t current_sf = 7;

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!lora_link::init_radio(LORA_FREQ, LORA_SS, LORA_RST, LORA_DIO0, current_sf, LORA_BW, LORA_CR,
                             LORA_TX_POWER, LORA_CRC)) {
    Serial.println("[ERR] LoRa.begin falhou. Verifique pinos/placa.");
    while (true) {
      delay(1000);
    }
  }

  LoRa.receive();

  Serial.print("# BOOT base_ack_v2 freq=");
  Serial.print(static_cast<uint32_t>(LORA_FREQ));
  Serial.print(" sf=");
  Serial.print(current_sf);
  Serial.print(" bw=");
  Serial.print(static_cast<uint32_t>(LORA_BW));
  Serial.print(" cr=4/");
  Serial.print(LORA_CR);
  Serial.print(" tx=");
  Serial.print(LORA_TX_POWER);
  Serial.print(" crc=");
  Serial.println(LORA_CRC ? "on" : "off");
}

void loop() {
  int packet_size = LoRa.parsePacket();
  if (!packet_size) {
    return;
  }

  if (packet_size != static_cast<int>(protocol::STATUS_ALERT_SIZE)) {
    while (LoRa.available()) {
      LoRa.read();
    }
    return;
  }

  uint8_t raw[protocol::STATUS_ALERT_SIZE];
  for (size_t i = 0; i < protocol::STATUS_ALERT_SIZE; i++) {
    raw[i] = static_cast<uint8_t>(LoRa.read());
  }

  protocol::StatusAlert alert{};
  if (!protocol::unpack_status_alert(raw, protocol::STATUS_ALERT_SIZE, alert)) {
    return;
  }

  const int rssi_int = LoRa.packetRssi();
  const float snr_f = LoRa.packetSnr();
  const int8_t rssi_dbm = static_cast<int8_t>(rssi_int);
  const int snr_round = static_cast<int>(snr_f >= 0 ? (snr_f + 0.5f) : (snr_f - 0.5f));
  const int8_t snr_db = static_cast<int8_t>(snr_round);

  protocol::Ack ack{};
  ack.msg_type = protocol::MSG_ACK;
  ack.seq = alert.seq;
  ack.rssi_dbm = rssi_dbm;
  ack.snr_db = snr_db;

  uint8_t ack_raw[protocol::ACK_SIZE];
  protocol::pack_ack(ack, ack_raw);

  LoRa.beginPacket();
  LoRa.write(ack_raw, protocol::ACK_SIZE);
  LoRa.endPacket();
  LoRa.receive();

  Serial.print("# ACK ms=");
  Serial.print(millis());
  Serial.print(" seq=");
  Serial.print(alert.seq);
  Serial.print(" rssi=");
  Serial.print(rssi_int);
  Serial.print(" snr=");
  Serial.println(snr_f, 1);
}

