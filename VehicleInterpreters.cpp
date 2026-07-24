#include "VehicleInterpreters.h"

// Initialize as a completely safe, empty memory pointer layout first
GlobalFrameworkContext* sys_ctx = nullptr;

bool applyBoolSignalMappings(twai_message_t &msg, const BoolSignalMapping* mappings, size_t count) {
    if (sys_ctx == nullptr || mappings == nullptr) return false;

    bool matched = false;
    for (size_t i = 0; i < count; i++) {
        const BoolSignalMapping& mapping = mappings[i];
        if (msg.identifier != mapping.frame_id || msg.data_length_code <= mapping.byte_index) continue;

        const bool is_active = (msg.data[mapping.byte_index] & mapping.bit_mask) != 0;
        sys_ctx->metrics.*(mapping.value_field) = is_active;
        sys_ctx->metrics.*(mapping.known_field) = true;
        matched = true;
    }
    return matched;
}

bool applyByteSignalMappings(twai_message_t &msg, const ByteSignalMapping* mappings, size_t count) {
    if (sys_ctx == nullptr || mappings == nullptr) return false;

    bool matched = false;
    for (size_t i = 0; i < count; i++) {
        const ByteSignalMapping& mapping = mappings[i];
        if (msg.identifier != mapping.frame_id || msg.data_length_code <= mapping.byte_index) continue;

        const uint8_t raw = (msg.data[mapping.byte_index] & mapping.bit_mask) >> mapping.bit_shift;
        sys_ctx->metrics.*(mapping.value_field) = raw;
        sys_ctx->metrics.*(mapping.known_field) = true;
        matched = true;
    }
    return matched;
}

bool applyOdometerSignalMapping(twai_message_t &msg, const OdometerSignalMapping& mapping) {
    if (sys_ctx == nullptr || msg.identifier != mapping.frame_id) return false;
    if (mapping.byte_count == 0 || mapping.byte_count > 4) return false;
    if (msg.data_length_code < (mapping.start_byte + mapping.byte_count)) return false;

    const auto mapped_byte_index = [&](uint8_t offset) -> uint8_t {
        return mapping.big_endian
            ? (mapping.start_byte + offset)
            : (mapping.start_byte + mapping.byte_count - 1 - offset);
    };

    uint32_t raw = 0;
    for (uint8_t i = 0; i < mapping.byte_count; i++) {
        raw = (raw << 8) | msg.data[mapped_byte_index(i)];
    }

    sys_ctx->metrics.odometer_km = raw * mapping.scale;
    sys_ctx->metrics.odometer_valid = true;
    return true;
}

static void decodeGearSelectorFromRaw(uint8_t raw) {
    if (sys_ctx == nullptr) return;

    sys_ctx->metrics.gear_position_known = true;
    sys_ctx->metrics.selected_gear_known = false;
    sys_ctx->metrics.selected_gear = 0;

    switch (raw) {
        case 0x0: sys_ctx->metrics.gear_position = GEAR_SELECTOR_PARK;    break;
        case 0x1: sys_ctx->metrics.gear_position = GEAR_SELECTOR_REVERSE; break;
        case 0x2: sys_ctx->metrics.gear_position = GEAR_SELECTOR_NEUTRAL; break;
        case 0x3: sys_ctx->metrics.gear_position = GEAR_SELECTOR_DRIVE;   break;
        case 0x4: sys_ctx->metrics.gear_position = GEAR_SELECTOR_SPORT;   break;
        default:
            sys_ctx->metrics.gear_position = GEAR_SELECTOR_MANUAL;
            sys_ctx->metrics.selected_gear = (raw >= 0x5) ? (raw - 0x4) : 0;
            sys_ctx->metrics.selected_gear_known = sys_ctx->metrics.selected_gear > 0;
            break;
    }
}

void applyGenericGearFrame(twai_message_t &msg, uint32_t frame_id, uint8_t gear_byte_index, uint8_t sport_byte_index, uint8_t sport_mask) {
    if (sys_ctx == nullptr || msg.identifier != frame_id) return;
    if (msg.data_length_code <= gear_byte_index) return;

    const uint8_t raw_selector = msg.data[gear_byte_index] & 0x0F;
    const uint8_t displayed_gear = (msg.data[gear_byte_index] >> 4) & 0x0F;

    decodeGearSelectorFromRaw(raw_selector);

    if (displayed_gear > 0) {
        sys_ctx->metrics.selected_gear = displayed_gear;
        sys_ctx->metrics.selected_gear_known = true;
    }

    if (msg.data_length_code > sport_byte_index) {
        sys_ctx->metrics.sport_mode_active = (msg.data[sport_byte_index] & sport_mask) != 0;
        sys_ctx->metrics.sport_mode_known = true;
    }
}

void parsePassiveDiagnosticsFrame(twai_message_t &msg) {
    if (sys_ctx == nullptr) return;
    if (msg.identifier < 0x7E8 || msg.identifier > 0x7EF || msg.data_length_code < 3) return;

    sys_ctx->metrics.diagnostics_seen = true;
    sys_ctx->metrics.diag_response_counter++;
    sys_ctx->metrics.last_diag_source = msg.identifier;

    const uint8_t pci = msg.data[0];
    if ((pci & 0xF0) != 0x00) return; // Only decode simple single-frame payloads here.

    const uint8_t payload_len = pci & 0x0F;
    if (payload_len < 1 || msg.data_length_code < 2) return;

    const uint8_t service = msg.data[1];
    sys_ctx->metrics.last_diag_service = service;
    sys_ctx->metrics.last_diag_pid = (payload_len >= 2 && msg.data_length_code >= 3) ? msg.data[2] : 0x00;

    if (service == 0x41 && payload_len >= 2) {
        const uint8_t pid = msg.data[2];
        if (pid == 0x01 && payload_len >= 6 && msg.data_length_code >= 6) {
            const uint8_t a = msg.data[3];
            sys_ctx->metrics.mil_active = (a & 0x80) != 0;
            sys_ctx->metrics.mil_status_known = true;
            sys_ctx->metrics.stored_dtc_count = a & 0x7F;
        } else if (pid == 0x42 && payload_len >= 4 && msg.data_length_code >= 5) {
            const uint16_t raw_voltage = ((uint16_t)msg.data[3] << 8) | msg.data[4];
            sys_ctx->metrics.control_module_voltage = raw_voltage / 1000.0f;
            sys_ctx->metrics.control_module_voltage_known = true;
        }
    } else if (service == 0x43) {
        uint8_t dtc_count = 0;
        for (uint8_t i = 2; (i + 1) < msg.data_length_code; i += 2) {
            if (msg.data[i] != 0x00 || msg.data[i + 1] != 0x00) dtc_count++;
        }
        sys_ctx->metrics.stored_dtc_count = dtc_count;
        sys_ctx->metrics.mil_status_known = true;
    }
}

const char* gearSelectorPositionLabel(GearSelectorPosition position) {
    switch (position) {
        case GEAR_SELECTOR_PARK:    return "P";
        case GEAR_SELECTOR_REVERSE: return "R";
        case GEAR_SELECTOR_NEUTRAL: return "N";
        case GEAR_SELECTOR_DRIVE:   return "D";
        case GEAR_SELECTOR_SPORT:   return "S";
        case GEAR_SELECTOR_MANUAL:  return "M";
        default:                    return "UNAVAILABLE";
    }
}

const char* infotainmentSourceLabel(uint8_t code) {
    switch (code) {
        case 0x01: return "RADIO";
        case 0x02: return "MEDIA";
        case 0x03: return "NAVIGATION";
        case 0x04: return "PHONE";
        case 0x05: return "BLUETOOTH";
        case 0x06: return "USB";
        case 0x07: return "AUX";
        default:   return "UNAVAILABLE";
    }
}

const char* availabilityLabel(bool known, bool active, const char* active_label, const char* inactive_label, const char* unknown_label) {
    if (!known) return unknown_label;
    return active ? active_label : inactive_label;
}

const char* openClosedLabel(bool open) {
    return open ? "OPEN" : "CLOSED";
}

void GenericVehicleInterpreter::configureUiLimits() {
    if (sys_ctx != nullptr) {
        sys_ctx->normal_green = lv_color_make(0, 180, 0);
        if (sys_ctx->rpm_meter != nullptr) lv_arc_set_range(sys_ctx->rpm_meter, 0, 6000);
        if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
    }
}
