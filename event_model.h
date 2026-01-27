#pragma once

#include <Arduino.h>

namespace event_model {

enum class Source : uint8_t {
  kVR3 = 1,
};

struct DetectionEvent {
  Source source;
  uint32_t ts_ms;
  bool recognized;
  int16_t record_id;
  int16_t confidence;
};

}  // namespace event_model
