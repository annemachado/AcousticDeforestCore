#include "vr_link.h"

namespace vr_link {

namespace {
constexpr int kDefaultRxPin = 16;
constexpr int kDefaultTxPin = 17;
}  // namespace

VrLink::VrLink() = default;

VrLink::~VrLink() {
  if (vr_ != nullptr) {
    delete vr_;
    vr_ = nullptr;
  }
}

bool VrLink::begin(const Config &cfg) {
  if (cfg.serial == nullptr) {
    ready_ = false;
    return false;
  }

  config_ = cfg;
  serial_ = cfg.serial;

  if (vr_ != nullptr) {
    delete vr_;
    vr_ = nullptr;
  }

  vr_ = new VR_ESP(*serial_);

  const uint8_t rx_pin = (cfg.rx_pin >= 0) ? static_cast<uint8_t>(cfg.rx_pin)
                                           : static_cast<uint8_t>(kDefaultRxPin);
  const uint8_t tx_pin = (cfg.tx_pin >= 0) ? static_cast<uint8_t>(cfg.tx_pin)
                                           : static_cast<uint8_t>(kDefaultTxPin);

  vr_->begin(cfg.baud, rx_pin, tx_pin);
  ready_ = true;
  return true;
}

bool VrLink::is_ready() const {
  return ready_ && vr_ != nullptr;
}

bool VrLink::clear_records() {
  if (!is_ready()) {
    return false;
  }
  return vr_->clear() == 0;
}

bool VrLink::load_record(uint8_t record_id) {
  if (!is_ready()) {
    return false;
  }
  uint8_t record = record_id;
  return vr_->load(&record, 1) > 0; //mudei para >0 pois quando vr_->load(&record, 1) significa que não teve dados para copiar no VR_ESP::load
}

bool VrLink::load_records(const uint8_t *records, size_t record_count) {
  if (!is_ready()) {
    return false;
  }
  if (records == nullptr || record_count == 0) {
    return false;
  }
  return vr_->load(records, record_count) > 0; //mudei para >0 pois quando vr_->load(&record, 1) significa que não teve dados para copiar no VR_ESP::load
}

bool VrLink::check(RecognizerStatus &out_status) {
  // 1) valida inicialização
  if (!is_ready()) {
    out_status = RecognizerStatus{};
    return false;
  }

  // 2) lê o BSR do recognizer
  uint8_t buffer[vr_link::kCheckRawMax] = {0};  // 0..10 (11 bytes) conforme documentação do driver
  const int n = vr_->checkRecognizer(buffer, sizeof(buffer), config_.poll_timeout_ms);

  if (n < 0) {
    // -1 timeout/curto; -2 comando inesperado
    out_status = RecognizerStatus{};
    return false;
  }
 
  // 3) preenche struct com o que veio (com cuidado para não ler além do que foi copiado)
  out_status = RecognizerStatus{};
  out_status.raw_len = static_cast<uint8_t>(n);

   // Campos só se existirem (n indica quantos bytes foram copiados)
  if (n >= 1) {
    out_status.valid_count = buffer[0];
  }

  // loaded_ids: bytes 1..7
  for (int i = 0; i < 7; i++) {
    const int idx = 1 + i;
    if (n > idx) {
      out_status.loaded_ids[i] = buffer[idx];
    }
  }

  if (n >= 9) {
    out_status.total_records = buffer[8];
  }
  if (n >= 10) {
    out_status.valid_bitmap = buffer[9];
  }
  if (n >= 11) {
    out_status.group_mode = buffer[10];
  }
    return true;
}

bool VrLink::train_record(uint8_t record_id) {
  if (!is_ready()) {
    return false;
  }
  uint8_t record = record_id;
  return vr_->train(&record, 1) >= 0;
}

Status VrLink::poll(event_model::DetectionEvent &out_event) {
  if (!is_ready()) {
    reset_event(out_event, false);
    return Status::kNotInitialized;
  }

  uint8_t buffer[16] = {0};
  const int result = vr_->recognize(buffer, sizeof(buffer), config_.poll_timeout_ms);
  if (result >= 0) {
    reset_event(out_event, true);
    out_event.record_id = static_cast<int16_t>(buffer[1]);
    out_event.ts_ms = millis();
    return Status::kOk;
  }

  if (result == -2) {
    reset_event(out_event, false);
    return Status::kNoEvent;
  }

  reset_event(out_event, false);
  return Status::kError;
}

void VrLink::reset_event(event_model::DetectionEvent &out_event, bool recognized) const {
  out_event.source = event_model::Source::kVR3;
  out_event.ts_ms = 0;
  out_event.recognized = recognized;
  out_event.record_id = -1;
  out_event.confidence = -1;
}

}  // namespace vr_link
