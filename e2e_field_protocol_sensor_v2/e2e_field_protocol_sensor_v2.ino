#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

#include <AcousticDeforestCore.h>

// E2E Field Protocol v2 (SENSOR)
//
// Este sketch executa blocos de detecção acústica local (VR3.1) com entrega
// fim-a-fim via LoRa+ACK. Cada bloco pode gerar 0..N ocorrências (EVENT).
//
// Sucesso por bloco:
//  - Local (recog): sucesso se n_recog_total >= 1
//  - Fim-a-fim (e2e): sucesso se n_ackok_total >= 1
//
// Timestamps no CSV são absolutos (millis()) e incluem janela de guarda.
// Definições:
//  - TTR = first_recog_ms - t_start_cmd_ms
//  - TTA_total = first_ackok_ms - t_start_cmd_ms
//
// Anti-vazamento entre blocos:
//  - guard_ms=500: reconhecimentos dentro da janela inicial são descartados
//  - antes de iniciar o bloco, o buffer UART do VR é drenado

#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

constexpr uint8_t DEV_ID = 1;

constexpr long LORA_FREQ = 915E6;
constexpr long LORA_BW = 125E3;
constexpr int LORA_CR = 5;
constexpr int LORA_TX_POWER = 14;
constexpr bool LORA_CRC = true;

constexpr uint32_t ACK_TIMEOUT_MS = 950;
constexpr uint8_t MAX_RETRIES = 2;

constexpr uint32_t VR_BAUD = 9600;
constexpr int VR_RX_PIN = 16;
constexpr int VR_TX_PIN = 17;
constexpr uint16_t VR_POLL_TIMEOUT_MS = 30;

constexpr uint32_t GUARD_MS = 500;

constexpr size_t kMaxIds = 20;

struct BlockState {
  bool active = false;
  String block_type;
  float distance_m = 0.0f;
  uint32_t planned_duration_s = 20;
  uint32_t planned_duration_ms = 20000;

  uint32_t t_start_cmd_ms = 0;
  uint32_t t_block_start_ms = 0;
  uint32_t t_block_end_ms = 0;
  uint32_t t_block_end_planned_ms = 0;

  uint32_t n_guard_discard = 0;
  uint32_t n_recog_total = 0;
  uint32_t n_ackok_total = 0;
  int32_t first_recog_ms = -1;
  int32_t first_ackok_ms = -1;
  uint16_t event_idx = 0;

  String notes;
};

vr_link::VrLink vr;

bool vr_ready = false;

String test_id = "TEST";
String config_id = "P40";
String phase_id = "VAR";
String notes_next;

uint32_t block_duration_s = 20;

uint16_t seq = 0;
uint32_t block_id = 0;

uint32_t event_planned_sum_p40 = 0;
uint32_t event_planned_sum_p20 = 0;
uint32_t event_planned_sum_sem = 0;

uint8_t current_sf = 7;

bool loaded_flags[256] = {false};
uint8_t loaded_count = 0;

BlockState current_block;

void print_help();
void print_status();
void print_vr_status();
void handle_command(const String &line);
bool parse_uint8(const char *token, uint8_t &out);
bool parse_uint32(const char *token, uint32_t &out);
bool parse_float(const char *token, float &out);
void start_block(const String &block_type, float distance_m, uint32_t planned_duration_s);
void finish_block();
void poll_block();
void print_csv_header();
void print_block_row(const BlockState &block);
void print_event_row(const BlockState &block, uint16_t event_idx, const event_model::DetectionEvent &event,
                     uint16_t seq_id, bool ack_ok, const lora_link::AckMetrics &metrics);
String csv_escape(const String &value);
void set_loaded_ids(const uint8_t *ids, size_t count);
void add_loaded_id(uint8_t id);
uint32_t sum_event_planned_for_config();

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n[BOOT] e2e_field_protocol_sensor_v2");

  if (!lora_link::init_radio(LORA_FREQ, LORA_SS, LORA_RST, LORA_DIO0, current_sf, LORA_BW, LORA_CR,
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

  vr_ready = vr.begin(cfg);
  if (vr_ready) {
    Serial.println("[BOOT] VR ready");
  } else {
    Serial.println("[ERR] VR init failed");
  }

  print_csv_header();
  print_help();
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    handle_command(line);
  }

  if (current_block.active) {
    poll_block();
  }

  delay(5);
}

void print_csv_header() {
  Serial.println("# CSV: timestamps absolutos em millis(); guard_ms=500 aplicado no inicio do bloco");
  Serial.println(
      "row_type,test_id,config_id,phase,block_type,block_id,distance_m,block_duration_s,t_start_cmd_ms,"
      "t_block_start_ms,t_block_end_ms,block_actual_duration_ms,guard_ms,n_guard_discard,n_recog_total,"
      "first_recog_ms,n_ackok_total,first_ackok_ms,lora_sf,lora_bw,lora_cr,lora_tx_power,lora_freq_hz,"
      "ack_timeout_ms,max_retries,notes");
  Serial.println(
      "row_type,test_id,config_id,phase,block_type,block_id,distance_m,event_idx,t_recog_ms,record_id,seq,"
      "ack_ok,attempts,tx_first_ms,tx_last_ms,tx_end_ms,ack_rx_ms,rtt_ms,ack_rssi_dbm,ack_snr_db");
}

void print_help() {
  Serial.println("[CMD] help");
  Serial.println("[CMD] status");
  Serial.println("[CMD] vr_status");
  Serial.println("[CMD] set_test_id <string>");
  Serial.println("[CMD] set_config <P40|P20|SEM>");
  Serial.println("[CMD] set_phase <VAR|REF|NO_EVENT>");
  Serial.println("[CMD] set_block_dur <seconds>");
  Serial.println("[CMD] set_notes <texto>");
  Serial.println("[CMD] load <id>");
  Serial.println("[CMD] loadlist <id1> <id2> ...");
  Serial.println("[CMD] start_event <distance_m>");
  Serial.println("[CMD] start_no_event <seconds>");
  Serial.println("[CMD] start_no_event_auto");
  Serial.println("[CMD] sf <7..12>");
}

void print_status() {
  Serial.println("[STATUS] e2e_field_protocol_sensor_v2");
  Serial.print("[STATUS] test_id=");
  Serial.println(test_id);
  Serial.print("[STATUS] config_id=");
  Serial.println(config_id);
  Serial.print("[STATUS] phase=");
  Serial.println(phase_id);
  Serial.print("[STATUS] block_duration_s=");
  Serial.println(block_duration_s);
  Serial.print("[STATUS] notes_pending=");
  Serial.println(notes_next);
  Serial.print("[STATUS] lora freq=");
  Serial.print(static_cast<uint32_t>(LORA_FREQ));
  Serial.print(" sf=");
  Serial.print(current_sf);
  Serial.print(" bw=");
  Serial.print(static_cast<uint32_t>(LORA_BW));
  Serial.print(" cr=4/");
  Serial.print(LORA_CR);
  Serial.print(" tx=");
  Serial.print(LORA_TX_POWER);
  Serial.print(" ack_timeout_ms=");
  Serial.print(ACK_TIMEOUT_MS);
  Serial.print(" max_retries=");
  Serial.println(MAX_RETRIES);
}

void print_vr_status() {
  Serial.print("[VR] ready=");
  Serial.println(vr_ready ? "yes" : "no");
  Serial.print("[VR] loaded_count=");
  Serial.println(loaded_count);

  Serial.print("[VR] loaded_ids=");
  bool first = true;
  for (int i = 0; i < 256; i++) {
    if (loaded_flags[i]) {
      if (!first) {
        Serial.print(",");
      }
      Serial.print(i);
      first = false;
    }
  }
  Serial.println();

  vr_link::RecognizerStatus status;
  if (vr.check(status)) {
    Serial.print("[VR] recognizer valid_count=");
    Serial.print(status.valid_count);
    Serial.print(" total_records=");
    Serial.print(status.total_records);
    Serial.print(" bitmap=0x");
    Serial.print(status.valid_bitmap, HEX);
    Serial.print(" group_mode=");
    Serial.println(status.group_mode);
  } else {
    Serial.println("[VR] recognizer check failed");
  }
}

void handle_command(const String &line) {
  String trimmed = line;
  trimmed.trim();
  if (trimmed.isEmpty()) {
    return;
  }

  char buffer[200] = {0};
  trimmed.toCharArray(buffer, sizeof(buffer));
  char *token = strtok(buffer, " ");
  if (!token) {
    return;
  }

  for (char *p = token; *p; ++p) {
    *p = static_cast<char>(toupper(*p));
  }

  if (strcmp(token, "HELP") == 0) {
    print_help();
    return;
  }

  if (strcmp(token, "STATUS") == 0) {
    print_status();
    return;
  }

  if (strcmp(token, "VR_STATUS") == 0) {
    print_vr_status();
    return;
  }

  if (strcmp(token, "SET_TEST_ID") == 0) {
    char *value = strtok(nullptr, " ");
    if (!value) {
      Serial.println("[ERR] set_test_id: informe string");
      return;
    }
    test_id = value;
    Serial.print("[CMD] test_id=");
    Serial.println(test_id);
    return;
  }

  if (strcmp(token, "SET_CONFIG") == 0) {
    char *value = strtok(nullptr, " ");
    if (!value) {
      Serial.println("[ERR] set_config: informe P40|P20|SEM");
      return;
    }
    String upper = value;
    upper.toUpperCase();
    if (upper != "P40" && upper != "P20" && upper != "SEM") {
      Serial.println("[ERR] set_config: valor invalido");
      return;
    }
    config_id = upper;
    Serial.print("[CMD] config_id=");
    Serial.println(config_id);
    return;
  }

  if (strcmp(token, "SET_PHASE") == 0) {
    char *value = strtok(nullptr, " ");
    if (!value) {
      Serial.println("[ERR] set_phase: informe VAR|REF|NO_EVENT");
      return;
    }
    String upper = value;
    upper.toUpperCase();
    if (upper != "VAR" && upper != "REF" && upper != "NO_EVENT") {
      Serial.println("[ERR] set_phase: valor invalido");
      return;
    }
    phase_id = upper;
    Serial.print("[CMD] phase=");
    Serial.println(phase_id);
    return;
  }

  if (strcmp(token, "SET_BLOCK_DUR") == 0) {
    char *value = strtok(nullptr, " ");
    uint32_t dur_s = 0;
    if (!parse_uint32(value, dur_s) || dur_s == 0) {
      Serial.println("[ERR] set_block_dur: valor invalido");
      return;
    }
    block_duration_s = dur_s;
    Serial.print("[CMD] block_duration_s=");
    Serial.println(block_duration_s);
    return;
  }

  if (strcmp(token, "SET_NOTES") == 0) {
    char *value = strtok(nullptr, "");
    if (!value) {
      Serial.println("[ERR] set_notes: informe texto");
      return;
    }
    while (*value == ' ') {
      value++;
    }
    notes_next = value;
    Serial.print("[CMD] notes set for next block: ");
    Serial.println(notes_next);
    return;
  }

  if (strcmp(token, "LOAD") == 0) {
    char *value = strtok(nullptr, " ");
    uint8_t id = 0;
    if (!parse_uint8(value, id)) {
      Serial.println("[ERR] load: id invalido");
      return;
    }
    if (vr.load_record(id)) {
      add_loaded_id(id);
      Serial.println("[CMD] load ok");
    } else {
      Serial.println("[ERR] load failed");
    }
    return;
  }

  if (strcmp(token, "LOADLIST") == 0) {
    uint8_t ids[kMaxIds] = {0};
    size_t count = 0;
    char *value = strtok(nullptr, " ");
    while (value != nullptr && count < kMaxIds) {
      uint8_t id = 0;
      if (!parse_uint8(value, id)) {
        Serial.println("[ERR] loadlist: id invalido");
        return;
      }
      ids[count++] = id;
      value = strtok(nullptr, " ");
    }
    if (count == 0) {
      Serial.println("[ERR] loadlist: informe ids");
      return;
    }
    if (vr.load_records(ids, count)) {
      set_loaded_ids(ids, count);
      Serial.println("[CMD] loadlist ok");
    } else {
      Serial.println("[ERR] loadlist failed");
    }
    return;
  }

  if (strcmp(token, "START_EVENT") == 0) {
    char *value = strtok(nullptr, " ");
    float distance = 0.0f;
    if (!parse_float(value, distance) || distance <= 0.0f) {
      Serial.println("[ERR] start_event: distancia invalida");
      return;
    }
    start_block("EVENT", distance, block_duration_s);
    return;
  }

  if (strcmp(token, "START_NO_EVENT") == 0) {
    char *value = strtok(nullptr, " ");
    uint32_t duration_s = 0;
    if (!parse_uint32(value, duration_s) || duration_s == 0) {
      Serial.println("[ERR] start_no_event: duracao invalida");
      return;
    }
    start_block("NO_EVENT", 0.0f, duration_s);
    return;
  }

  if (strcmp(token, "START_NO_EVENT_AUTO") == 0) {
    uint32_t sum_s = sum_event_planned_for_config();
    if (sum_s == 0) {
      Serial.println("[ERR] start_no_event_auto: soma zero para config atual");
      return;
    }
    start_block("NO_EVENT", 0.0f, sum_s);
    return;
  }

  if (strcmp(token, "SF") == 0) {
    char *value = strtok(nullptr, " ");
    uint32_t sf_value = 0;
    if (!parse_uint32(value, sf_value) || sf_value < 7 || sf_value > 12) {
      Serial.println("[ERR] sf: valor invalido");
      return;
    }
    current_sf = static_cast<uint8_t>(sf_value);
    LoRa.setSpreadingFactor(current_sf);
    LoRa.receive();
    Serial.print("[CMD] sf=");
    Serial.println(current_sf);
    return;
  }

  Serial.print("[ERR] comando desconhecido: ");
  Serial.println(token);
}

bool parse_uint8(const char *token, uint8_t &out) {
  if (!token || *token == '\0') {
    return false;
  }
  char *end = nullptr;
  long value = strtol(token, &end, 10);
  if (end == token || *end != '\0' || value < 0 || value > 255) {
    return false;
  }
  out = static_cast<uint8_t>(value);
  return true;
}

bool parse_uint32(const char *token, uint32_t &out) {
  if (!token || *token == '\0') {
    return false;
  }
  char *end = nullptr;
  unsigned long value = strtoul(token, &end, 10);
  if (end == token || *end != '\0') {
    return false;
  }
  out = static_cast<uint32_t>(value);
  return true;
}

bool parse_float(const char *token, float &out) {
  if (!token || *token == '\0') {
    return false;
  }
  char *end = nullptr;
  float value = strtof(token, &end);
  if (end == token || *end != '\0') {
    return false;
  }
  out = value;
  return true;
}

void start_block(const String &block_type, float distance_m, uint32_t planned_duration_s) {
  if (current_block.active) {
    Serial.println("[ERR] bloco ja em execucao");
    return;
  }

  block_id++;

  current_block = BlockState{};
  current_block.active = true;
  current_block.block_type = block_type;
  current_block.distance_m = distance_m;
  current_block.planned_duration_s = planned_duration_s;
  current_block.planned_duration_ms = planned_duration_s * 1000UL;
  current_block.t_start_cmd_ms = millis();
  current_block.notes = notes_next;
  notes_next = "";

  while (Serial2.available() > 0) {
    Serial2.read();
  }

  current_block.t_block_start_ms = millis();
  current_block.t_block_end_planned_ms = current_block.t_block_start_ms + current_block.planned_duration_ms;

  Serial.print("[CMD] start block ");
  Serial.print(current_block.block_type);
  Serial.print(" id=");
  Serial.print(block_id);
  Serial.print(" dur_s=");
  Serial.println(current_block.planned_duration_s);
}

void poll_block() {
  const uint32_t now_ms = millis();
  if (now_ms >= current_block.t_block_end_planned_ms) {
    finish_block();
    return;
  }

  event_model::DetectionEvent event;
  vr_link::Status status = vr.poll(event);
  if (status == vr_link::Status::kOk && event.recognized) {
    const uint32_t recog_ms = event.ts_ms;
    if (recog_ms < current_block.t_block_start_ms + GUARD_MS) {
      current_block.n_guard_discard++;
      return;
    }

    current_block.n_recog_total++;
    current_block.event_idx++;
    if (current_block.first_recog_ms < 0) {
      current_block.first_recog_ms = static_cast<int32_t>(recog_ms);
    }

    seq++;

    protocol::StatusAlert status_alert{};
    status_alert.dev_id = DEV_ID;
    status_alert.msg_type = protocol::MSG_ALERT;
    status_alert.uptime_ms = recog_ms;
    status_alert.seq = seq;
    status_alert.field8 = static_cast<uint8_t>(event.record_id & 0xFF);
    status_alert.battery_mv = 3700;

    uint8_t payload[protocol::STATUS_ALERT_SIZE];
    protocol::pack_status_alert(status_alert, payload);

    lora_link::AckMetrics metrics{};
    const bool ack_ok = lora_link::send_with_ack(payload, seq, ACK_TIMEOUT_MS, MAX_RETRIES, metrics);

    if (ack_ok) {
      current_block.n_ackok_total++;
      if (current_block.first_ackok_ms < 0) {
        current_block.first_ackok_ms = static_cast<int32_t>(metrics.ack_rx_ms);
      }
    }

    print_event_row(current_block, current_block.event_idx, event, seq, ack_ok, metrics);
  } else if (status == vr_link::Status::kError) {
    Serial.println("[ERR] vr poll");
  }
}

void finish_block() {
  current_block.t_block_end_ms = millis();

  print_block_row(current_block);

  if (current_block.block_type == "EVENT") {
    if (config_id == "P40") {
      event_planned_sum_p40 += current_block.planned_duration_s;
    } else if (config_id == "P20") {
      event_planned_sum_p20 += current_block.planned_duration_s;
    } else if (config_id == "SEM") {
      event_planned_sum_sem += current_block.planned_duration_s;
    }
  }

  current_block.active = false;
}

void print_block_row(const BlockState &block) {
  uint32_t actual_duration_ms = block.t_block_end_ms - block.t_block_start_ms;

  Serial.print("BLOCK,");
  Serial.print(csv_escape(test_id));
  Serial.print(",");
  Serial.print(config_id);
  Serial.print(",");
  Serial.print(phase_id);
  Serial.print(",");
  Serial.print(block.block_type);
  Serial.print(",");
  Serial.print(block_id);
  Serial.print(",");
  Serial.print(block.distance_m, 2);
  Serial.print(",");
  Serial.print(block.planned_duration_s);
  Serial.print(",");
  Serial.print(block.t_start_cmd_ms);
  Serial.print(",");
  Serial.print(block.t_block_start_ms);
  Serial.print(",");
  Serial.print(block.t_block_end_ms);
  Serial.print(",");
  Serial.print(actual_duration_ms);
  Serial.print(",");
  Serial.print(GUARD_MS);
  Serial.print(",");
  Serial.print(block.n_guard_discard);
  Serial.print(",");
  Serial.print(block.n_recog_total);
  Serial.print(",");
  Serial.print(block.first_recog_ms >= 0 ? block.first_recog_ms : -1);
  Serial.print(",");
  Serial.print(block.n_ackok_total);
  Serial.print(",");
  Serial.print(block.first_ackok_ms >= 0 ? block.first_ackok_ms : -1);
  Serial.print(",");
  Serial.print(current_sf);
  Serial.print(",");
  Serial.print(static_cast<uint32_t>(LORA_BW));
  Serial.print(",");
  Serial.print(LORA_CR);
  Serial.print(",");
  Serial.print(LORA_TX_POWER);
  Serial.print(",");
  Serial.print(static_cast<uint32_t>(LORA_FREQ));
  Serial.print(",");
  Serial.print(ACK_TIMEOUT_MS);
  Serial.print(",");
  Serial.print(MAX_RETRIES);
  Serial.print(",");
  Serial.println(csv_escape(block.notes));
}

void print_event_row(const BlockState &block, uint16_t event_idx, const event_model::DetectionEvent &event,
                     uint16_t seq_id, bool ack_ok, const lora_link::AckMetrics &metrics) {
  const int32_t ack_rx_ms = ack_ok ? static_cast<int32_t>(metrics.ack_rx_ms) : -1;
  const int32_t rtt_ms = ack_ok ? static_cast<int32_t>(metrics.rtt_ms) : -1;
  const int32_t ack_rssi = ack_ok ? static_cast<int32_t>(metrics.ack_rssi) : -999;
  const int32_t ack_snr = ack_ok ? static_cast<int32_t>(metrics.ack_snr) : -999;

  Serial.print("EVENT,");
  Serial.print(csv_escape(test_id));
  Serial.print(",");
  Serial.print(config_id);
  Serial.print(",");
  Serial.print(phase_id);
  Serial.print(",");
  Serial.print(block.block_type);
  Serial.print(",");
  Serial.print(block_id);
  Serial.print(",");
  Serial.print(block.distance_m, 2);
  Serial.print(",");
  Serial.print(event_idx);
  Serial.print(",");
  Serial.print(event.ts_ms);
  Serial.print(",");
  Serial.print(event.record_id);
  Serial.print(",");
  Serial.print(seq_id);
  Serial.print(",");
  Serial.print(ack_ok ? 1 : 0);
  Serial.print(",");
  Serial.print(metrics.attempts);
  Serial.print(",");
  Serial.print(metrics.tx_first_ms);
  Serial.print(",");
  Serial.print(metrics.tx_last_ms);
  Serial.print(",");
  Serial.print(metrics.tx_end_ms);
  Serial.print(",");
  Serial.print(ack_rx_ms);
  Serial.print(",");
  Serial.print(rtt_ms);
  Serial.print(",");
  Serial.print(ack_rssi);
  Serial.print(",");
  Serial.println(ack_snr);
}

String csv_escape(const String &value) {
  String escaped = "\"";
  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    if (c == '"') {
      escaped += "\"\"";
    } else {
      escaped += c;
    }
  }
  escaped += "\"";
  return escaped;
}

void set_loaded_ids(const uint8_t *ids, size_t count) {
  for (int i = 0; i < 256; i++) {
    loaded_flags[i] = false;
  }
  loaded_count = 0;
  for (size_t i = 0; i < count; i++) {
    add_loaded_id(ids[i]);
  }
}

void add_loaded_id(uint8_t id) {
  if (!loaded_flags[id]) {
    loaded_flags[id] = true;
    loaded_count++;
  }
}

uint32_t sum_event_planned_for_config() {
  if (config_id == "P40") {
    return event_planned_sum_p40;
  }
  if (config_id == "P20") {
    return event_planned_sum_p20;
  }
  return event_planned_sum_sem;
}

