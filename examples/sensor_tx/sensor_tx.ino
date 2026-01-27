#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <stdlib.h>
#include <string.h>

#include <AcousticDeforestCore.h>

#define LORA_SS   18
#define LORA_RST  14
#define LORA_DIO0 26

// escolha uma frequência dentro de 915-928 MHz
#define LORA_FREQ 915E6

uint16_t seq = 0;

// Defina um DevID fixo (no topo também):
const uint8_t DEV_ID = 1;

const uint32_t ACK_TIMEOUT_MS = 950;
const uint8_t  MAX_RETRIES    = 2;   // total de tentativas = 1 + MAX_RETRIES
const uint32_t DEFAULT_PERIOD_MS = 3000;
const char FW_ID[] = "sensor";

enum SendMode {
  MODE_STATUS = 0,
  MODE_ALERT  = 1,
  MODE_MIXED  = 2
};

uint32_t period_ms = DEFAULT_PERIOD_MS;
SendMode send_mode = MODE_MIXED;
uint16_t mixed_interval = 5;
bool transmitting = false;
int current_sf = 7;
char run_id[16] = "";
char run_note[33] = "";
uint16_t dist_m = 0;
uint16_t n_msgs = 0;
uint16_t sent_count = 0;
uint16_t ack_ok_count = 0;
uint16_t fail_count = 0;

void handleSerial();
void processCommand(char *line);
void printConfig();
void printMenu();
void printChecklist();
bool missingRequired();
const char *modeLabel();

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!lora_link::init_radio(LORA_FREQ, LORA_SS, LORA_RST, LORA_DIO0, current_sf, 125E3, 5, 14,
                             true)) {
    Serial.println("# LoRa.begin() falhou. Verifique pinos/placa.");
    while (true) {}
  }

  delay(1500);
  Serial.print("# BOOT ");
  Serial.print(FW_ID);
  Serial.print(" freq=");
  Serial.print((uint32_t)LORA_FREQ);
  Serial.print(" sf=");
  Serial.print(current_sf);
  Serial.print(" bw=125000 cr=4/5 tx=14 period_ms=");
  Serial.print(period_ms);
  Serial.print(" mode=");
  Serial.print(modeLabel());
  Serial.print(" mixed_interval=");
  Serial.print(mixed_interval);
  Serial.print(" ack_timeout=");
  Serial.print(ACK_TIMEOUT_MS);
  Serial.print(" max_retries=");
  Serial.println(MAX_RETRIES);
  Serial.println("# TX pronto. Aguardando start.");
  printMenu();
}

void loop() {
  handleSerial();
  if (!transmitting) {
    delay(10);
    return;
  }
  uint32_t t_loop0 = millis();

  if (sent_count >= n_msgs) {
    transmitting = false;
    float pdr = (n_msgs > 0) ? ((float)ack_ok_count / (float)n_msgs) : 0.0f;
    Serial.print("# END run=");
    Serial.print(run_id);
    Serial.print(" dist_m=");
    Serial.print(dist_m);
    Serial.print(" sf=");
    Serial.print(current_sf);
    Serial.print(" period_ms=");
    Serial.print(period_ms);
    Serial.print(" n=");
    Serial.print(n_msgs);
    Serial.print(" ack_ok=");
    Serial.print(ack_ok_count);
    Serial.print(" fail=");
    Serial.print(fail_count);
    Serial.print(" pdr=");
    Serial.println(pdr, 3);
    return;
  }

  uint16_t msg_index = sent_count + 1;
  seq++;

  bool isAlert = false;
  if (send_mode == MODE_ALERT) {
    isAlert = true;
  } else if (send_mode == MODE_MIXED) {
    isAlert = (mixed_interval > 0) && (seq % mixed_interval == 0);
  }
  uint8_t msgType = isAlert ? protocol::MSG_ALERT : protocol::MSG_STATUS; //escolhe o MsgType certo para montar o payload.

  // Campos do STATUS/ALERT (neste passo, valores simples para teste)
  uint32_t uptime_ms = millis();
  uint8_t flags = 0x01;         // exemplo: bit0=1 (qualquer convenção por enquanto)
  uint8_t eventClass = 1;     // exemplo: 1 = motosserra
  uint16_t battery_mV = 3700;   // valor simulado por enquanto (3.7V)

  // 1) Monta STATUS (11 bytes) uma única vez (o mesmo payload será retransmitido)
  uint8_t payload[protocol::STATUS_ALERT_SIZE];
  protocol::StatusAlert status{};
  status.dev_id = DEV_ID;
  status.msg_type = msgType;
  status.uptime_ms = uptime_ms;
  status.seq = seq;
  status.field8 = isAlert ? eventClass : flags;
  status.battery_mv = battery_mV;
  protocol::pack_status_alert(status, payload);

  // 2) Variáveis de resultado final (para log)
  lora_link::AckMetrics metrics{};
  bool ack_ok = lora_link::send_with_ack(payload, seq, ACK_TIMEOUT_MS, MAX_RETRIES, metrics);
  uint8_t attempt_final = metrics.attempts;
  uint32_t rtt_ms = metrics.rtt_ms;
  int8_t ack_rssi = metrics.ack_rssi;
  int8_t ack_snr  = metrics.ack_snr;

  // MsgType numérico (STATUS = 0xB1). No CSV eu recomendo imprimir como decimal para facilitar filtro.
  // Se você preferir em HEX, eu ajusto depois.
  if (ack_ok) {
    ack_ok_count++;
  } else {
    fail_count++;
  }
  sent_count++;

  Serial.print(run_id);                      Serial.print(",");
  Serial.print(dist_m);                      Serial.print(",");
  Serial.print(current_sf);                  Serial.print(",");
  Serial.print(period_ms);                   Serial.print(",");
  Serial.print(ACK_TIMEOUT_MS);              Serial.print(",");
  Serial.print(MAX_RETRIES);                 Serial.print(",");
  Serial.print(n_msgs);                      Serial.print(",");
  Serial.print(msg_index);                   Serial.print(",");
  Serial.print(DEV_ID);                      Serial.print(",");
  Serial.print((int)msgType);                Serial.print(",");
  Serial.print(uptime_ms);                   Serial.print(",");
  Serial.print(seq);                         Serial.print(",");
  Serial.print(attempt_final);               Serial.print(",");
  Serial.print(ack_ok ? 1 : 0);              Serial.print(",");
  Serial.print(ack_ok ? (int)rtt_ms : -1);   Serial.print(",");
  Serial.print(ack_ok ? (int)ack_rssi : -1); Serial.print(",");
  Serial.print(ack_ok ? (int)ack_snr : -1);  Serial.print(",");
  Serial.print(battery_mV);                  Serial.print(",");

  // Flags e EventClass
  Serial.print(isAlert ? -1 : (int)flags);   Serial.print(",");//se for ALERT, Flags não se aplica → -1.
  Serial.println(isAlert ? (int)eventClass : -1); //se for STATUS, EventClass não se aplica → -1.

  if (sent_count >= n_msgs) {
    transmitting = false;
    float pdr = (n_msgs > 0) ? ((float)ack_ok_count / (float)n_msgs) : 0.0f;
    Serial.print("# END run=");
    Serial.print(run_id);
    Serial.print(" dist_m=");
    Serial.print(dist_m);
    Serial.print(" sf=");
    Serial.print(current_sf);
    Serial.print(" period_ms=");
    Serial.print(period_ms);
    Serial.print(" n=");
    Serial.print(n_msgs);
    Serial.print(" ack_ok=");
    Serial.print(ack_ok_count);
    Serial.print(" fail=");
    Serial.print(fail_count);
    Serial.print(" pdr=");
    Serial.println(pdr, 3);
    return;
  }

  // Período configurável (quando possível)
  uint32_t spent = millis() - t_loop0;
  if (spent < period_ms) delay(period_ms - spent);
}

void handleSerial() {
  static char cmd_buf[64];
  static uint8_t cmd_len = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmd_buf[cmd_len] = '\0';
      if (cmd_len > 0) {
        processCommand(cmd_buf);
      }
      cmd_len = 0;
      continue;
    }

    if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = c;
    }
  }
}

void processCommand(char *line) {
  while (*line == ' ') line++;
  if (*line == '\0') return;

  char *token = strtok(line, " ");
  if (!token) return;

  if (strcmp(token, "period") == 0) {
    char *value = strtok(nullptr, " ");
    if (!value) {
      Serial.println("# usage: period <ms>");
      return;
    }
    uint32_t new_period = (uint32_t)strtoul(value, nullptr, 10);
    if (new_period == 0) {
      Serial.println("# invalid period");
      return;
    }
    period_ms = new_period;
    Serial.print("# period ");
    Serial.println(period_ms);
    return;
  }

  if (strcmp(token, "sf") == 0) {
    char *value = strtok(nullptr, " ");
    if (!value) {
      Serial.println("# usage: sf <7..12>");
      return;
    }
    int sf = atoi(value);
    if (sf < 7 || sf > 12) {
      Serial.println("# invalid sf");
      return;
    }
    LoRa.setSpreadingFactor(sf);
    current_sf = sf;
    Serial.print("# sf ");
    Serial.println(sf);
    return;
  }

  if (strcmp(token, "start") == 0) {
    if (missingRequired()) {
      Serial.println("# START bloqueado: faltam parametros obrigatorios.");
      printChecklist();
      return;
    }
    transmitting = true;
    sent_count = 0;
    ack_ok_count = 0;
    fail_count = 0;
    Serial.print("# START dev_id=");
    Serial.print(DEV_ID);
    Serial.print(" run=");
    Serial.print(run_id);
    Serial.print(" dist_m=");
    Serial.print(dist_m);
    Serial.print(" n=");
    Serial.print(n_msgs);
    if (run_note[0]) {
      Serial.print(" note=");
      Serial.print(run_note);
    }
    Serial.print(" freq=");
    Serial.print((uint32_t)LORA_FREQ);
    Serial.print(" sf=");
    Serial.print(current_sf);
    Serial.print(" bw=125000 cr=4/5 tx=14 period_ms=");
    Serial.print(period_ms);
    Serial.print(" mode=");
    Serial.print(modeLabel());
    Serial.print(" mixed_interval=");
    Serial.print(mixed_interval);
    Serial.print(" ack_timeout=");
    Serial.print(ACK_TIMEOUT_MS);
    Serial.print(" max_retries=");
    Serial.println(MAX_RETRIES);
    Serial.println("# RunID,Dist_m,SF,Period_ms,AckTimeout_ms,MaxRetries,N_msgs_planned,MsgIndex,DevID,MsgType,Uptime_ms,Seq,attempt_final,ack_ok,RTT_ms,RSSI_dBm,SNR_dB,Battery_mV,Flags,EventClass");
    return;
  }

  if (strcmp(token, "stop") == 0) {
    transmitting = false;
    Serial.println("# stopped");
    return;
  }

  if (strcmp(token, "run") == 0) {
    char *value = strtok(nullptr, " ");
    if (!value) {
      Serial.println("# usage: run <id>");
      return;
    }
    strncpy(run_id, value, sizeof(run_id) - 1);
    run_id[sizeof(run_id) - 1] = '\0';
    Serial.print("# run ");
    Serial.println(run_id);
    return;
  }

  if (strcmp(token, "status") == 0) {
    printConfig();
    return;
  }

  if (strcmp(token, "check") == 0) {
    printConfig();
    return;
  }

  if (strcmp(token, "help") == 0) {
    printMenu();
    return;
  }

  if (strcmp(token, "dist") == 0) {
    char *value = strtok(nullptr, " ");
    if (!value) {
      Serial.println("# usage: dist <metros>");
      return;
    }
    uint32_t dist_value = (uint32_t)strtoul(value, nullptr, 10);
    if (dist_value == 0 || dist_value > 65535) {
      Serial.println("# invalid dist");
      return;
    }
    dist_m = (uint16_t)dist_value;
    Serial.print("# dist ");
    Serial.println(dist_m);
    return;
  }

  if (strcmp(token, "n") == 0) {
    char *value = strtok(nullptr, " ");
    if (!value) {
      Serial.println("# usage: n <quantidade>");
      return;
    }
    uint32_t n_value = (uint32_t)strtoul(value, nullptr, 10);
    if (n_value == 0 || n_value > 65535) {
      Serial.println("# invalid n");
      return;
    }
    n_msgs = (uint16_t)n_value;
    Serial.print("# n ");
    Serial.println(n_msgs);
    return;
  }

  if (strcmp(token, "note") == 0) {
    char *value = strtok(nullptr, "");
    if (!value) {
      Serial.println("# usage: note <texto_curto>");
      return;
    }
    while (*value == ' ') value++;
    strncpy(run_note, value, sizeof(run_note) - 1);
    run_note[sizeof(run_note) - 1] = '\0';
    Serial.print("# note ");
    Serial.println(run_note);
    return;
  }

  if (strcmp(token, "mode") == 0) {
    char *mode = strtok(nullptr, " ");
    if (!mode) {
      Serial.println("# usage: mode status|alert|mixed <N>");
      return;
    }
    if (strcmp(mode, "status") == 0) {
      send_mode = MODE_STATUS;
      Serial.println("# mode status");
      return;
    }
    if (strcmp(mode, "alert") == 0) {
      send_mode = MODE_ALERT;
      Serial.println("# mode alert");
      return;
    }
    if (strcmp(mode, "mixed") == 0) {
      char *value = strtok(nullptr, " ");
      if (!value) {
        Serial.println("# usage: mode mixed <N>");
        return;
      }
      uint16_t interval = (uint16_t)strtoul(value, nullptr, 10);
      if (interval == 0) {
        Serial.println("# invalid mixed interval");
        return;
      }
      send_mode = MODE_MIXED;
      mixed_interval = interval;
      Serial.print("# mode mixed ");
      Serial.println(mixed_interval);
      return;
    }
    Serial.println("# invalid mode");
    return;
  }

  Serial.println("# unknown command");
}

void printConfig() {
  Serial.print("# CONFIG run=");
  Serial.print(run_id[0] ? run_id : "-");
  Serial.print(" dist_m=");
  Serial.print(dist_m);
  Serial.print(" n=");
  Serial.print(n_msgs);
  Serial.print(" note=");
  Serial.print(run_note[0] ? run_note : "-");
  Serial.print(" period_ms=");
  Serial.print(period_ms);
  Serial.print(" sf=");
  Serial.print(current_sf);
  Serial.print(" mode=");
  Serial.print(modeLabel());
  Serial.print(" mixed_interval=");
  Serial.print(mixed_interval);
  Serial.print(" ack_timeout=");
  Serial.print(ACK_TIMEOUT_MS);
  Serial.print(" max_retries=");
  Serial.println(MAX_RETRIES);
  printChecklist();
}

const char *modeLabel() {
  switch (send_mode) {
    case MODE_STATUS:
      return "status";
    case MODE_ALERT:
      return "alert";
    case MODE_MIXED:
    default:
      return "mixed";
  }
}

void printMenu() {
  Serial.println("# MENU");
  Serial.println("# comandos:");
  Serial.println("#  help               -> mostrar menu");
  Serial.println("#  check|status       -> mostrar configuracao e pendencias");
  Serial.println("#  run <id>           -> define identificador da rodada");
  Serial.println("#  dist <metros>      -> distancia em metros (ex: 150)");
  Serial.println("#  n <quantidade>     -> numero de mensagens da rodada");
  Serial.println("#  note <texto_curto> -> observacao (opcional)");
  Serial.println("#  sf <7..12>         -> spreading factor");
  Serial.println("#  period <ms>        -> intervalo entre mensagens");
  Serial.println("#  mode status|alert|mixed <N>");
  Serial.println("#  start              -> iniciar rodada");
  Serial.println("#  stop               -> parar transmissao");
  printChecklist();
}

void printChecklist() {
  Serial.print("# checklist ");
  Serial.print("run_id=");
  Serial.print(run_id[0] ? "ok" : "missing");
  Serial.print(" dist_m=");
  Serial.print(dist_m > 0 ? "ok" : "missing");
  Serial.print(" n_msgs=");
  Serial.print(n_msgs > 0 ? "ok" : "missing");
  if (missingRequired()) {
    Serial.print(" missing:");
    if (!run_id[0]) Serial.print(" run_id");
    if (dist_m == 0) Serial.print(" dist_m");
    if (n_msgs == 0) Serial.print(" n_msgs");
  }
  Serial.println();
}

bool missingRequired() {
  return (!run_id[0] || dist_m == 0 || n_msgs == 0);
}
