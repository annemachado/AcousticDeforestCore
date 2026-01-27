#pragma once

#include <Arduino.h>

namespace protocol {

constexpr uint8_t MSG_ALERT = 0xA1;
constexpr uint8_t MSG_STATUS = 0xB1;
constexpr uint8_t MSG_ACK = 0xC1;

constexpr size_t STATUS_ALERT_SIZE = 11;
constexpr size_t ACK_SIZE = 5;

struct StatusAlert {
  uint8_t dev_id;
  uint8_t msg_type;
  uint32_t uptime_ms;
  uint16_t seq;
  uint8_t field8;
  uint16_t battery_mv;
};

struct Ack {
  uint8_t msg_type;
  uint16_t seq;
  int8_t rssi_dbm;
  int8_t snr_db;
};

void pack_status_alert(const StatusAlert &msg, uint8_t *out);
bool unpack_status_alert(const uint8_t *in, size_t len, StatusAlert &out);

void pack_ack(const Ack &ack, uint8_t *out);
bool unpack_ack(const uint8_t *in, size_t len, Ack &out);

}  // namespace protocol
