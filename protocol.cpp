#include "protocol.h"

namespace protocol {

void pack_status_alert(const StatusAlert &msg, uint8_t *out) {
  out[0] = msg.dev_id;
  out[1] = msg.msg_type;
  out[2] = static_cast<uint8_t>(msg.uptime_ms & 0xFF);
  out[3] = static_cast<uint8_t>((msg.uptime_ms >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>((msg.uptime_ms >> 16) & 0xFF);
  out[5] = static_cast<uint8_t>((msg.uptime_ms >> 24) & 0xFF);
  out[6] = static_cast<uint8_t>(msg.seq & 0xFF);
  out[7] = static_cast<uint8_t>((msg.seq >> 8) & 0xFF);
  out[8] = msg.field8;
  out[9] = static_cast<uint8_t>(msg.battery_mv & 0xFF);
  out[10] = static_cast<uint8_t>((msg.battery_mv >> 8) & 0xFF);
}

bool unpack_status_alert(const uint8_t *in, size_t len, StatusAlert &out) {
  if (len != STATUS_ALERT_SIZE) {
    return false;
  }

  out.dev_id = in[0];
  out.msg_type = in[1];
  out.uptime_ms = static_cast<uint32_t>(in[2]) |
                  (static_cast<uint32_t>(in[3]) << 8) |
                  (static_cast<uint32_t>(in[4]) << 16) |
                  (static_cast<uint32_t>(in[5]) << 24);
  out.seq = static_cast<uint16_t>(in[6]) |
            (static_cast<uint16_t>(in[7]) << 8);
  out.field8 = in[8];
  out.battery_mv = static_cast<uint16_t>(in[9]) |
                   (static_cast<uint16_t>(in[10]) << 8);

  return true;
}

void pack_ack(const Ack &ack, uint8_t *out) {
  out[0] = ack.msg_type;
  out[1] = static_cast<uint8_t>(ack.seq & 0xFF);
  out[2] = static_cast<uint8_t>((ack.seq >> 8) & 0xFF);
  out[3] = static_cast<uint8_t>(ack.rssi_dbm);
  out[4] = static_cast<uint8_t>(ack.snr_db);
}

bool unpack_ack(const uint8_t *in, size_t len, Ack &out) {
  if (len != ACK_SIZE) {
    return false;
  }

  out.msg_type = in[0];
  out.seq = static_cast<uint16_t>(in[1]) |
            (static_cast<uint16_t>(in[2]) << 8);
  out.rssi_dbm = static_cast<int8_t>(in[3]);
  out.snr_db = static_cast<int8_t>(in[4]);

  return true;
}

}  // namespace protocol
