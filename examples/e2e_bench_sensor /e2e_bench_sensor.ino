#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

#include <AcousticDeforestCore.h>

#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

constexpr uint8_t DEV_ID = 1;
constexpr uint8_t EVENT_CLASS_DEFAULT = 1;

constexpr long LORA_FREQ = 915E6;
constexpr int LORA_SF = 7;
constexpr long LORA_BW = 125E3;
constexpr int LORA_CR = 5;
constexpr int LORA_TX_POWER = 14;
constexpr bool LORA_CRC = true;

constexpr uint32_t ACK_TIMEOUT_MS = 950;
constexpr uint8_t MAX_RETRIES = 2;
constexpr uint32_t COOLDOWN_MS = 2000;

constexpr uint32_t VR_BAUD = 9600;
constexpr int VR_RX_PIN = 16;
constexpr int VR_TX_PIN = 17;
constexpr uint16_t VR_POLL_TIMEOUT_MS = 30;

constexpr size_t kMaxIds = 20;

vr_link::VrLink vr;
bool run_mode = false;
uint8_t event_class = EVENT_CLASS_DEFAULT;
uint16_t seq = 0;
uint32_t last_tx_ms = 0;

bool parse_uint8(const char *token, uint8_t &out) {
  if (token == nullptr || *token == '\0') {
    return false;
  }
  char *end = nullptr;
  const long value = strtol(token, &end, 10);
  if (end == token || *end != '\0' || value < 0 || value > 255) {
    return false;
  }
  out = static_cast<uint8_t>(value);
  return true;
}

void print_help() {
  Serial.println("[CMD] comandos:");
  Serial.println("[CMD] help");
  Serial.println("[CMD] load <id>");
  Serial.println("[CMD] loadlist <id1> <id2> ...");
  Serial.println("[CMD] run");
  Serial.println("[CMD] stop");
  Serial.println("[CMD] class <eventClass>");
  Serial.println("[CMD] status");
}

void print_status() {
  Serial.println("[CFG] e2e_bench_sensor");
  Serial.print("[CFG] dev_id=");
  Serial.println(DEV_ID);
  Serial.print("[CFG] run_mode=");
  Serial.println(run_mode ? "on" : "off");
  Serial.print("[CFG] event_class=");
  Serial.println(event_class);
  Serial.print("[CFG] cooldown_ms=");
  Serial.println(COOLDOWN_MS);
  Serial.print("[CFG] ack_timeout_ms=");
  Serial.println(ACK_TIMEOUT_MS);
  Serial.print("[CFG] max_retries=");
  Serial.println(MAX_RETRIES);
  Serial.print("[CFG] lora freq=");
  Serial.print(static_cast<uint32_t>(LORA_FREQ));
  Serial.print(" sf=");
  Serial.print(LORA_SF);
  Serial.print(" bw=");
  Serial.print(static_cast<uint32_t>(LORA_BW));
  Serial.print(" cr=4/");
  Serial.print(LORA_CR);
  Serial.print(" tx=");
  Serial.println(LORA_TX_POWER);
}

void handle_command(const String &line) {
  String trimmed = line;
  trimmed.trim();
  if (trimmed.isEmpty()) {
    return;
  }

  char buffer[160] = {0};
  trimmed.toCharArray(buffer, sizeof(buffer));
  char *token = strtok(buffer, " ");
  if (token == nullptr) {
    return;
  }

  if (strcmp(token, "help") == 0) {
    print_help();
    return;
  }

  if (strcmp(token, "load") == 0) {
    char *id_token = strtok(nullptr, " ");
    uint8_t id = 0;
    if (!parse_uint8(id_token, id)) {
      Serial.println("[ERR] load: id invalido");
      return;
    }
    Serial.print("[CMD] load ");
    Serial.println(id);
    if (vr.load_record(id)) {
      Serial.println("[CMD] load ok");
    } else {
      Serial.println("[ERR] load failed");
    }
    return;
  }

  if (strcmp(token, "loadlist") == 0) {
    uint8_t ids[kMaxIds] = {0};
    size_t count = 0;
    char *id_token = strtok(nullptr, " ");
    while (id_token != nullptr && count < kMaxIds) {
      uint8_t id = 0;
      if (!parse_uint8(id_token, id)) {
        Serial.println("[ERR] loadlist: id invalido");
        return;
      }
      ids[count++] = id;
      id_token = strtok(nullptr, " ");
    }
    if (count == 0) {
      Serial.println("[ERR] loadlist: informe ids");
      return;
    }
    Serial.print("[CMD] loadlist ");
    Serial.println(count);
    if (vr.load_records(ids, count)) {
      Serial.println("[CMD] loadlist ok");
    } else {
      Serial.println("[ERR] loadlist failed");
    }
    return;
  }

  if (strcmp(token, "run") == 0) {
    run_mode = true;
    Serial.println("[CMD] run");
    return;
  }

  if (strcmp(token, "stop") == 0) {
    run_mode = false;
    Serial.println("[CMD] stop");
    return;
  }

  if (strcmp(token, "class") == 0) {
    char *class_token = strtok(nullptr, " ");
    uint8_t class_value = 0;
    if (!parse_uint8(class_token, class_value)) {
      Serial.println("[ERR] class: valor invalido");
      return;
    }
    event_class = class_value;
    Serial.print("[CMD] class ");
    Serial.println(event_class);
    return;
  }

  if (strcmp(token, "status") == 0) {
    print_status();
    return;
  }

  Serial.print("[ERR] comando desconhecido: ");
  Serial.println(token);
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n[BOOT] e2e_bench_sensor");
  Serial.print("[BOOT] dev_id=");
  Serial.print(DEV_ID);
  Serial.print(" event_class=");
  Serial.print(event_class);
  Serial.print(" freq=");
  Serial.print(static_cast<uint32_t>(LORA_FREQ));
  Serial.print(" sf=");
  Serial.print(LORA_SF);
  Serial.print(" bw=");
  Serial.print(static_cast<uint32_t>(LORA_BW));
  Serial.print(" cr=4/");
  Serial.print(LORA_CR);
  Serial.print(" tx=");
  Serial.print(LORA_TX_POWER);
  Serial.print(" ack_timeout=");
  Serial.print(ACK_TIMEOUT_MS);
  Serial.print(" max_retries=");
  Serial.print(MAX_RETRIES);
  Serial.print(" cooldown=");
  Serial.println(COOLDOWN_MS);

  if (!lora_link::init_radio(LORA_FREQ, LORA_SS, LORA_RST, LORA_DIO0, LORA_SF, LORA_BW, LORA_CR,
                             LORA_TX_POWER, LORA_CRC)) {
    Serial.println("[ERR] LoRa.begin falhou. Verifique pinos/placa.");
    while (true) {
      delay(1000);
    }
  }

  vr_link::Config cfg;
  cfg.serial = &Serial2;
  cfg.baud = VR_BAUD;
  cfg.rx_pin = VR_RX_PIN;
  cfg.tx_pin = VR_TX_PIN;
  cfg.poll_timeout_ms = VR_POLL_TIMEOUT_MS;

  if (vr.begin(cfg)) {
    Serial.println("[BOOT] VR ready");
  } else {
    Serial.println("[ERR] VR init failed");
  }

  print_help();
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    handle_command(line);
  }

  if (!run_mode) {
    delay(10);
    return;
  }

  event_model::DetectionEvent event;
  const vr_link::Status status = vr.poll(event);
  if (status == vr_link::Status::kOk && event.recognized) {
    const uint32_t now_ms = millis();
    if (now_ms - last_tx_ms < COOLDOWN_MS) {
      delay(5);
      return;
    }
    last_tx_ms = now_ms;

    seq++;
    protocol::StatusAlert status_alert{};
    status_alert.dev_id = DEV_ID;
    status_alert.msg_type = protocol::MSG_ALERT;
    status_alert.uptime_ms = now_ms;
    status_alert.seq = seq;
    status_alert.field8 = event_class;
    status_alert.battery_mv = 3700;

    uint8_t payload[protocol::STATUS_ALERT_SIZE];
    protocol::pack_status_alert(status_alert, payload);

    lora_link::AckMetrics metrics{};
    const bool ack_ok = lora_link::send_with_ack(payload, seq, ACK_TIMEOUT_MS, MAX_RETRIES, metrics);

    Serial.print("[E2E] ts_ms=");
    Serial.print(now_ms);
    Serial.print(" record_id=");
    Serial.print(event.record_id);
    Serial.print(" event_class=");
    Serial.print(event_class);
    Serial.print(" seq=");
    Serial.print(seq);
    Serial.print(" ack_ok=");
    Serial.print(ack_ok ? 1 : 0);
    Serial.print(" attempts=");
    Serial.print(metrics.attempts);
    Serial.print(" rtt_ms=");
    Serial.print(metrics.rtt_ms);
    Serial.print(" ack_rssi=");
    Serial.print(metrics.ack_rssi);
    Serial.print(" ack_snr=");
    Serial.println(metrics.ack_snr);
  } else if (status == vr_link::Status::kError) {
    Serial.println("[ERR] vr poll");
  }

  delay(10);
}
