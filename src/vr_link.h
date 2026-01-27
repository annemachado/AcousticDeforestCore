#pragma once

#include <Arduino.h>

#include "event_model.h"
#include <VoiceRecognitionV3_ESP32.h>

namespace vr_link {

constexpr size_t kCheckRawMax = 11;  // espaço seguro

enum class Status {
  kOk,
  kNoEvent,
  kError,
  kNotInitialized,
};

struct Config {
  HardwareSerial *serial;
  uint32_t baud;
  int rx_pin;
  int tx_pin;
  uint16_t poll_timeout_ms;
};

struct RecognizerStatus {
  uint8_t valid_count = 0;     // outBuf[0]
  uint8_t loaded_ids[7] = {0}; // outBuf[1..7]
  uint8_t total_records = 0;   // outBuf[8]
  uint8_t valid_bitmap = 0;    // outBuf[9]
  uint8_t group_mode = 0;      // outBuf[10]
  uint8_t raw_len = 0;         // bytes efetivamente recebidos/copiados
};

class VrLink {
 public:
  VrLink();
  ~VrLink();

  bool begin(const Config &cfg);
  bool is_ready() const;

  bool clear_records();
  bool load_record(uint8_t record_id);
  bool load_records(const uint8_t *records, size_t record_count);
  bool check(RecognizerStatus &out_status);
  bool train_record(uint8_t record_id);

  Status poll(event_model::DetectionEvent &out_event);

 private:
  void reset_event(event_model::DetectionEvent &out_event, bool recognized) const;

  Config config_{};
  HardwareSerial *serial_ = nullptr;
  VR_ESP *vr_ = nullptr;
  bool ready_ = false;
};

}  // namespace vr_link
