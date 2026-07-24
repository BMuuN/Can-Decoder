#include "VehicleInterpreters.h"

// =========================================================================
//  PLATFORM 5 — PQ24 / PQ34 / PL45 EARLY GENERATION CAN DECODER
//  Vehicles: Golf Mk4, Bora, Polo 9N, Audi A3 8L, Audi A4 B5/B6
//  Years:    ~2001 – 2005
//  Bus topology:
//    Drive Train Bus 500 kbps  – Engine (01), TCM (02), ABS (03), Airbag (15)
//    Comfort Bus     100 kbps  – Convenience (46), Doors (42/52), Climatronic (08)
//    Infotainment Bus 100 kbps – Radio (37) only, basic wake signal
//
//  NOTE: PQ24 does NOT broadcast:
//    • Odometer on CAN (no kPq24OdometerMapping below)
//    • Gear selector position (no dedicated gearbox CAN frame)
//    • Interior cabin-light state
//  These fields will remain at their zero-initialised / unknown defaults so
//  the UI correctly shows UNAVAILABLE for those items on this platform.
// =========================================================================

// --- COMFORT BUS: PQ24 lighting / indicator bits (addr 0x470) ---
// Bit layout is identical to PQ35 – the 0x470 frame was shared across PQ24/PQ35.
static constexpr BoolSignalMapping kPq24ComfortSignalMappings[] = {
    {0x470, 0, 0x01, &LiveTelemetryMetrics::left_indicator_active,  &LiveTelemetryMetrics::left_indicator_known},
    {0x470, 0, 0x02, &LiveTelemetryMetrics::right_indicator_active, &LiveTelemetryMetrics::right_indicator_known},
    {0x470, 0, 0x04, &LiveTelemetryMetrics::parking_lights_active,  &LiveTelemetryMetrics::parking_lights_known},
    {0x470, 0, 0x08, &LiveTelemetryMetrics::low_beam_active,        &LiveTelemetryMetrics::low_beam_known},
    {0x470, 0, 0x10, &LiveTelemetryMetrics::high_beam_active,       &LiveTelemetryMetrics::high_beam_known},
};

// =========================================================================
//  UNIFIED PQ24 DRIVE TRAIN DECODER
// =========================================================================
static void parsePq24DriveTrainFrame(twai_message_t &msg) {
    if (sys_ctx == nullptr) return;

    switch (msg.identifier) {
        case 0x280: {
            // PQ24 Engine Speed – same CAN ID as PQ35 (shared VAG convention).
            // Encoding: raw = RPM / 0.25, little-endian 16-bit.
            if (msg.data_length_code < 2) break;
            uint16_t low_byte  = (uint16_t)(msg.data[0]);
            uint16_t high_byte = (uint16_t)(msg.data[1]);
            sys_ctx->metrics.engine_rpm = ((high_byte << 8) | low_byte) * 0.25f;
            break;
        }
        case 0x288: {
            // PQ24 Drivetrain Thermal — coolant byte 0, oil byte 1.
            // raw - 40 = °C  (same formula as all VAG platforms).
            if (msg.data_length_code < 2) break;
            sys_ctx->metrics.coolant_temp = decode_temperature_offset(msg.data[0]);
            sys_ctx->metrics.oil_temp     = decode_temperature_offset(msg.data[1]);
            break;
        }
        case 0x380: {
            // PQ24 Absolute Manifold Pressure (MAP) → boost bar.
            // Encoding: raw byte × 10 = absolute pressure in mbar.
            // Subtract 1013 mbar atmospheric and convert to bar.
            if (msg.data_length_code < 1) break;
            int absolute_mbar = (int)msg.data[0] * 10;
            float boost = (absolute_mbar - 1013) / 1000.0f;
            sys_ctx->metrics.boost_bar = (boost < 0.0f) ? 0.0f : boost;
            break;
        }
        case 0x0C0: {
            // PQ24 Vehicle Speed — 0.01 km/h per LSB, little-endian.
            if (msg.data_length_code < 2) break;
            uint16_t raw_spd = (uint16_t)(msg.data[0]) |
                               ((uint16_t)(msg.data[1]) << 8);
            sys_ctx->metrics.vehicle_speed = raw_spd * 0.01f;
            break;
        }
        case 0x088: {
            // PQ24 Accelerator Pedal Position — 0.4 % per LSB.
            if (msg.data_length_code < 1) break;
            float pct = msg.data[0] * 0.4f;
            sys_ctx->metrics.throttle_pct = (pct > 100.0f) ? 100.0f : pct;
            break;
        }
    }

    parsePassiveDiagnosticsFrame(msg);
}

// =========================================================================
//  UNIFIED PQ24 COMFORT DECODER
// =========================================================================
static void parsePq24ComfortFrame(twai_message_t &msg) {
    if (sys_ctx == nullptr) return;

    if (msg.identifier == 0x351) {
        // PQ24 Door status byte (identical encoding to PQ35 0x351).
        if (msg.data_length_code < 1) return;
        uint8_t db = msg.data[0];
        sys_ctx->metrics.driver_door_open    = (db & 0x01) != 0;
        sys_ctx->metrics.passenger_door_open = (db & 0x02) != 0;
        // PQ24 does not broadcast rear door signals – leave rear_left/right at default false.
    }
    else if (msg.identifier == 0x65D) {
        // PQ24 Exterior Ambient Temperature — raw - 40 = °C.
        if (msg.data_length_code < 1) return;
        sys_ctx->metrics.exterior_temp = decode_temperature_offset(msg.data[0]);
    }
    else if (msg.identifier == 0x3BE) {
        // PQ24 EPB / Park-Brake status – bit 4 = handbrake applied.
        if (msg.data_length_code < 1) return;
        sys_ctx->metrics.handbrake_active = (msg.data[0] & 0x10) != 0;
    }

    applyBoolSignalMappings(msg, kPq24ComfortSignalMappings,
                            sizeof(kPq24ComfortSignalMappings) / sizeof(kPq24ComfortSignalMappings[0]));
}

// =========================================================================
//  UNIFIED PQ24 INFOTAINMENT DECODER
//  PQ24 infotainment is minimal: only a basic radio wake frame (0x6C1).
//  No MMI rotary input, no phone state, no track metadata.
// =========================================================================
static void parsePq24InfotainmentFrame(twai_message_t &msg) {
    if (sys_ctx == nullptr) return;
    // 0x6C1 carries a basic source byte on early radios (RCD300/RNS300).
    // We decode the source but mark phone/track as unknown on PQ24.
    if (msg.identifier == 0x6C1 && msg.data_length_code >= 1) {
        sys_ctx->metrics.infotainment_source_code  = msg.data[0];
        sys_ctx->metrics.infotainment_source_known = true;
        // No track or phone broadcast on PQ24 infotainment bus.
    }
}

// =========================================================================
//  GROUP 5 IMPLEMENTATION MATRIX: PQ24 VEHICLES
// =========================================================================

// --- 1. AUDI A3 8L (PQ24 / PL45) ---
void AudiA38LInterpreter::interpretDriveTrain(twai_message_t &msg) { parsePq24DriveTrainFrame(msg); }
void AudiA38LInterpreter::interpretComfort(twai_message_t &msg)    { parsePq24ComfortFrame(msg); }
void AudiA38LInterpreter::interpretInfotainment(twai_message_t &msg) { parsePq24InfotainmentFrame(msg); }
void AudiA38LInterpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(220, 0, 0); // Audi instrument cluster red
    if (sys_ctx->rpm_meter  != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,  0, 7500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180); // 1.8 Bar max
}

// --- 2. AUDI A4 B6 (PQ24) ---
void AudiA4B6Interpreter::interpretDriveTrain(twai_message_t &msg) { parsePq24DriveTrainFrame(msg); }
void AudiA4B6Interpreter::interpretComfort(twai_message_t &msg)    { parsePq24ComfortFrame(msg); }
void AudiA4B6Interpreter::interpretInfotainment(twai_message_t &msg) { parsePq24InfotainmentFrame(msg); }
void AudiA4B6Interpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(200, 30, 0); // B6 warm amber-red
    if (sys_ctx->rpm_meter  != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,  0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 150);
}

// --- 3. VW GOLF MK4 / BORA (PQ24) ---
void VwGolf4Interpreter::interpretDriveTrain(twai_message_t &msg) { parsePq24DriveTrainFrame(msg); }
void VwGolf4Interpreter::interpretComfort(twai_message_t &msg)    { parsePq24ComfortFrame(msg); }
void VwGolf4Interpreter::interpretInfotainment(twai_message_t &msg) { parsePq24InfotainmentFrame(msg); }
void VwGolf4Interpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Classic VW white instrumentation
    if (sys_ctx->rpm_meter  != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,  0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180);
}

// --- 4. VW POLO 9N (PQ24 compact – cluster-integrated gateway) ---
void VwPolo9NInterpreter::interpretDriveTrain(twai_message_t &msg) { parsePq24DriveTrainFrame(msg); }
void VwPolo9NInterpreter::interpretComfort(twai_message_t &msg)    { parsePq24ComfortFrame(msg); }
void VwPolo9NInterpreter::interpretInfotainment(twai_message_t &msg) { parsePq24InfotainmentFrame(msg); }
void VwPolo9NInterpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Economy white dials
    if (sys_ctx->rpm_meter  != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,  0, 6500);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 120); // Small turbo
}

// --- 5. VW BORA (shared chassis with Golf Mk4, mapped separately for clarity) ---
void VwBoraInterpreter::interpretDriveTrain(twai_message_t &msg) { parsePq24DriveTrainFrame(msg); }
void VwBoraInterpreter::interpretComfort(twai_message_t &msg)    { parsePq24ComfortFrame(msg); }
void VwBoraInterpreter::interpretInfotainment(twai_message_t &msg) { parsePq24InfotainmentFrame(msg); }
void VwBoraInterpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(255, 255, 255); // Classic Bora white dials
    if (sys_ctx->rpm_meter  != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,  0, 7000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 180);
}
