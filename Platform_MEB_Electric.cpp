#include "VehicleInterpreters.h"

// =========================================================================
//  PLATFORM 6 — MQB EVO / MEB ELECTRIC ERA DECODER
//  Vehicles: VW ID.3, ID.4, ID.5, ID.Buzz, Audi Q4 e-tron, Skoda Enyaq, Cupra Born
//  Years:    ~2020 – present
//  Bus topology:
//    Autonomous / ADAS Bus (CAN-FD): Front Radar, Cameras, IQ.DRIVE
//    Drivetrain / Thermal Bus (CAN-FD): VCU, BMS (0x8C), Heat Pump
//    Comfort Bus (CAN-FD): BCM, Climate, Seat modules
//    Infotainment Bus / Ethernet: MIB3, OTA, cloud services
//
//  Key differences from ICE platforms:
//    • No combustion engine → no boost pressure, no engine oil temperature
//    • Motor speed replaces engine RPM (no conventional tachometer range)
//    • Battery Management System (BMS addr 0x8C) broadcasts SoC, cell voltages
//    • Separate on-board charger module (addr 0x3D) broadcasts charging kW
//    • Recuperation torque available from VCU
//    • OTA update status visible on comfort/infotainment bus
//    • Physical MMI rotary replaced by capacitive touch interface
//
//  IMPORTANT NOTE ON SFD (Software Feature Deactivation / Firewall):
//    Some deep BMS and OTA diagnostic reads on MEB require authorisation tokens.
//    Passive bus sniffing of broadcast frames below is possible without SFD keys.
//    Frames marked [SFD-protected] require additional session-layer access.
// =========================================================================

// --- COMFORT BUS: MEB lighting / indicator bits (addr 0x470, same VAG convention) ---
static constexpr BoolSignalMapping kMebComfortSignalMappings[] = {
    {0x470, 0, 0x01, &LiveTelemetryMetrics::left_indicator_active,  &LiveTelemetryMetrics::left_indicator_known},
    {0x470, 0, 0x02, &LiveTelemetryMetrics::right_indicator_active, &LiveTelemetryMetrics::right_indicator_known},
    {0x470, 0, 0x04, &LiveTelemetryMetrics::parking_lights_active,  &LiveTelemetryMetrics::parking_lights_known},
    {0x470, 0, 0x08, &LiveTelemetryMetrics::low_beam_active,        &LiveTelemetryMetrics::low_beam_known},
    {0x470, 0, 0x10, &LiveTelemetryMetrics::high_beam_active,       &LiveTelemetryMetrics::high_beam_known},
    {0x470, 0, 0x20, &LiveTelemetryMetrics::interior_lights_active, &LiveTelemetryMetrics::interior_lights_known},
};

static constexpr ByteSignalMapping kMebInfotainmentMappings[] = {
    {0x6C1, 0, 0xFF, 0, &LiveTelemetryMetrics::infotainment_source_code, &LiveTelemetryMetrics::infotainment_source_known},
    {0x6C1, 1, 0xFF, 0, &LiveTelemetryMetrics::infotainment_track,       &LiveTelemetryMetrics::infotainment_track_known},
};

static constexpr BoolSignalMapping kMebInfotainmentStateMappings[] = {
    {0x6C1, 2, 0x01, &LiveTelemetryMetrics::phone_call_active, &LiveTelemetryMetrics::phone_call_known},
};

static constexpr OdometerSignalMapping kMebOdometerMapping = {0x5A0, 0, 4, 0.1f, true};

// =========================================================================
//  UNIFIED MEB DRIVETRAIN / EV DECODER
//  Decodes: motor speed (as RPM display), vehicle speed, throttle/pedal,
//           coolant/thermal loop temp, HV battery SoC, charging kW, regen torque,
//           motor temperature, gear selector (P/R/N/D/B), drive mode.
// =========================================================================
static void parseMebDriveTrainFrame(twai_message_t &msg) {
    if (sys_ctx == nullptr) return;

    switch (msg.identifier) {
        case 0x0FC: {
            // MEB Motor Speed — same broadcast ID as MQB RPM but unit is motor electrical speed.
            // Displayed on tachometer as "motor %" rather than RPM; raw encoding identical.
            if (msg.data_length_code < 2) break;
            uint16_t raw = (uint16_t)(msg.data[0]) | ((uint16_t)(msg.data[1]) << 8);
            sys_ctx->metrics.engine_rpm = raw * 0.25f; // Re-used field: motor speed in display units
            break;
        }
        case 0x096: {
            // MEB Vehicle Speed (0.01 km/h per LSB, little-endian) — shared with MQB.
            if (msg.data_length_code < 2) break;
            uint16_t raw_spd = (uint16_t)(msg.data[0]) | ((uint16_t)(msg.data[1]) << 8);
            sys_ctx->metrics.vehicle_speed = raw_spd * 0.01f;
            break;
        }
        case 0x084: {
            // MEB Accelerator Pedal Position (0.4 % per LSB) — shared with MQB.
            if (msg.data_length_code < 1) break;
            float pct = msg.data[0] * 0.4f;
            sys_ctx->metrics.throttle_pct = (pct > 100.0f) ? 100.0f : pct;
            break;
        }
        case 0x1A2: {
            // MEB Thermal Management — byte 0 = coolant/motor loop temp, byte 1 = inverter temp.
            // Encoding: raw - 40 = °C (same VAG convention).
            // Re-used coolant_temp field for the primary thermal loop temperature.
            // oil_temp is NOT broadcast on MEB (no engine oil); we leave it at 0/unknown.
            if (msg.data_length_code < 2) break;
            sys_ctx->metrics.coolant_temp      = decode_temperature_offset(msg.data[0]);
            sys_ctx->metrics.ev_motor_temp     = decode_temperature_offset(msg.data[1]);
            sys_ctx->metrics.ev_motor_temp_known = true;
            break;
        }
        case 0x317: {
            // MEB Exterior Ambient Temperature (raw - 40 = °C) — shared MQB frame ID.
            if (msg.data_length_code < 1) break;
            sys_ctx->metrics.exterior_temp = decode_temperature_offset(msg.data[0]);
            break;
        }

        // ---- HV Battery Management System (BMS, module 0x8C) broadcast frames ----
        case 0x1C0: {
            // MEB BMS State of Charge broadcast.
            // Byte 0-1: SoC in 0.1 % units, big-endian unsigned 16-bit.
            if (msg.data_length_code < 2) break;
            uint16_t raw_soc = ((uint16_t)(msg.data[0]) << 8) | (uint16_t)(msg.data[1]);
            sys_ctx->metrics.ev_soc_pct   = raw_soc * 0.1f;
            sys_ctx->metrics.ev_soc_known = true;
            break;
        }
        case 0x1C1: {
            // MEB BMS Pack Voltage.
            // Bytes 0-1: pack voltage in 0.1 V units, big-endian.
            if (msg.data_length_code < 2) break;
            uint16_t raw_v = ((uint16_t)(msg.data[0]) << 8) | (uint16_t)(msg.data[1]);
            sys_ctx->metrics.ev_hv_voltage       = raw_v * 0.1f;
            sys_ctx->metrics.ev_hv_voltage_known = true;
            break;
        }
        case 0x1C2: {
            // MEB BMS Cell Voltage Delta (imbalance in mV).
            // Byte 0: delta mV raw (0–255 mV).
            if (msg.data_length_code < 1) break;
            sys_ctx->metrics.ev_cell_voltage_delta  = (float)msg.data[0];
            sys_ctx->metrics.ev_cell_delta_known     = true;
            break;
        }

        // ---- On-Board Charger (OBC, module 0x3D) broadcast frames ----
        case 0x29E: {
            // MEB OBC Charging Power.
            // Bytes 0-1: charging power in 0.1 kW units, big-endian unsigned.
            // Byte 2 bit 0: charging active flag.
            if (msg.data_length_code < 3) break;
            uint16_t raw_kw = ((uint16_t)(msg.data[0]) << 8) | (uint16_t)(msg.data[1]);
            sys_ctx->metrics.ev_charging_kw      = raw_kw * 0.1f;
            sys_ctx->metrics.ev_charging_known   = true;
            sys_ctx->metrics.ev_charging_active  = (msg.data[2] & 0x01) != 0;
            break;
        }

        // ---- VCU Recuperation Torque ----
        case 0x1A4: {
            // MEB VCU Regen / Recuperation Torque.
            // Bytes 0-1: regen torque request in 0.25 Nm units, signed 16-bit.
            // Negative values = recuperation (energy back into battery).
            if (msg.data_length_code < 2) break;
            int16_t raw_torq = (int16_t)(((uint16_t)(msg.data[0]) << 8) | (uint16_t)(msg.data[1]));
            sys_ctx->metrics.ev_regen_torque  = raw_torq * 0.25f;
            sys_ctx->metrics.ev_regen_known   = true;
            break;
        }

        // ---- Gear selector / drive mode ----
        case 0x540: {
            // MEB Gear Selector and Drive Mode — same compact frame as MQB/PQ.
            applyGenericGearFrame(msg, 0x540, 0, 1, 0x01);
            break;
        }
    }

    applyOdometerSignalMapping(msg, kMebOdometerMapping);
    parsePassiveDiagnosticsFrame(msg);
}

// =========================================================================
//  UNIFIED MEB COMFORT DECODER
// =========================================================================
static void parseMebComfortFrame(twai_message_t &msg) {
    if (sys_ctx == nullptr) return;

    if (msg.identifier == 0x351) {
        // MEB Door status (identical VAG encoding to MQB/PQ35).
        if (msg.data_length_code < 1) return;
        uint8_t db = msg.data[0];
        sys_ctx->metrics.driver_door_open    = (db & 0x01) != 0;
        sys_ctx->metrics.passenger_door_open = (db & 0x02) != 0;
        sys_ctx->metrics.rear_left_door_open = (db & 0x04) != 0;
        sys_ctx->metrics.rear_right_door_open= (db & 0x08) != 0;
    }
    else if (msg.identifier == 0x3BE) {
        // MEB EPB / Park Brake – bit 4 = handbrake applied.
        if (msg.data_length_code < 1) return;
        sys_ctx->metrics.handbrake_active = (msg.data[0] & 0x10) != 0;
    }
    else if (msg.identifier == 0x392) {
        // MEB OTA Update Status frame.
        // Byte 0 bit 0: OTA session active.
        if (msg.data_length_code < 1) return;
        sys_ctx->metrics.ev_ota_update_active = (msg.data[0] & 0x01) != 0;
        sys_ctx->metrics.ev_ota_status_known  = true;
    }

    applyBoolSignalMappings(msg, kMebComfortSignalMappings,
                            sizeof(kMebComfortSignalMappings) / sizeof(kMebComfortSignalMappings[0]));
}

// =========================================================================
//  UNIFIED MEB INFOTAINMENT DECODER
// =========================================================================
static void parseMebInfotainmentFrame(twai_message_t &msg) {
    applyByteSignalMappings(msg, kMebInfotainmentMappings,
                            sizeof(kMebInfotainmentMappings) / sizeof(kMebInfotainmentMappings[0]));
    applyBoolSignalMappings(msg, kMebInfotainmentStateMappings,
                            sizeof(kMebInfotainmentStateMappings) / sizeof(kMebInfotainmentStateMappings[0]));
}

// =========================================================================
//  GROUP 6 IMPLEMENTATION MATRIX: MEB ELECTRIC VEHICLES
// =========================================================================

// --- 1. VW ID.3 ---
void VwId3Interpreter::interpretDriveTrain(twai_message_t &msg) { parseMebDriveTrainFrame(msg); }
void VwId3Interpreter::interpretComfort(twai_message_t &msg)    { parseMebComfortFrame(msg); }
void VwId3Interpreter::interpretInfotainment(twai_message_t &msg) { parseMebInfotainmentFrame(msg); }
void VwId3Interpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(0, 200, 100); // ID.3 fresh electric green
    // Motor speed displayed 0–15000 (motor electrical Hz analogue), boost bar re-used for SoC %
    if (sys_ctx->rpm_meter   != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,   0, 15000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 100);   // SoC 0-100%
}

// --- 2. VW ID.4 ---
void VwId4Interpreter::interpretDriveTrain(twai_message_t &msg) { parseMebDriveTrainFrame(msg); }
void VwId4Interpreter::interpretComfort(twai_message_t &msg)    { parseMebComfortFrame(msg); }
void VwId4Interpreter::interpretInfotainment(twai_message_t &msg) { parseMebInfotainmentFrame(msg); }
void VwId4Interpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(0, 200, 100);
    if (sys_ctx->rpm_meter   != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,   0, 15000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 100);
}

// --- 3. VW ID.5 ---
void VwId5Interpreter::interpretDriveTrain(twai_message_t &msg) { parseMebDriveTrainFrame(msg); }
void VwId5Interpreter::interpretComfort(twai_message_t &msg)    { parseMebComfortFrame(msg); }
void VwId5Interpreter::interpretInfotainment(twai_message_t &msg) { parseMebInfotainmentFrame(msg); }
void VwId5Interpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(0, 200, 100);
    if (sys_ctx->rpm_meter   != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,   0, 15000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 100);
}

// --- 4. VW ID.BUZZ ---
void VwIdBuzzInterpreter::interpretDriveTrain(twai_message_t &msg) { parseMebDriveTrainFrame(msg); }
void VwIdBuzzInterpreter::interpretComfort(twai_message_t &msg)    { parseMebComfortFrame(msg); }
void VwIdBuzzInterpreter::interpretInfotainment(twai_message_t &msg) { parseMebInfotainmentFrame(msg); }
void VwIdBuzzInterpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(0, 180, 200); // Retro-teal nod to T1 Bus
    if (sys_ctx->rpm_meter   != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,   0, 15000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 100);
}

// --- 5. AUDI Q4 E-TRON ---
void AudiQ4EtronInterpreter::interpretDriveTrain(twai_message_t &msg) { parseMebDriveTrainFrame(msg); }
void AudiQ4EtronInterpreter::interpretComfort(twai_message_t &msg)    { parseMebComfortFrame(msg); }
void AudiQ4EtronInterpreter::interpretInfotainment(twai_message_t &msg) { parseMebInfotainmentFrame(msg); }
void AudiQ4EtronInterpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(220, 0, 0); // Audi red instrument theme
    if (sys_ctx->rpm_meter   != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,   0, 15000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 100);
}

// --- 6. SKODA ENYAQ iV ---
void SkodaEnyaqInterpreter::interpretDriveTrain(twai_message_t &msg) { parseMebDriveTrainFrame(msg); }
void SkodaEnyaqInterpreter::interpretComfort(twai_message_t &msg)    { parseMebComfortFrame(msg); }
void SkodaEnyaqInterpreter::interpretInfotainment(twai_message_t &msg) { parseMebInfotainmentFrame(msg); }
void SkodaEnyaqInterpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(0, 160, 80); // Skoda Simply Clever green
    if (sys_ctx->rpm_meter   != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,   0, 15000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 100);
}

// --- 7. CUPRA BORN ---
void CupraBorhInterpreter::interpretDriveTrain(twai_message_t &msg) { parseMebDriveTrainFrame(msg); }
void CupraBorhInterpreter::interpretComfort(twai_message_t &msg)    { parseMebComfortFrame(msg); }
void CupraBorhInterpreter::interpretInfotainment(twai_message_t &msg) { parseMebInfotainmentFrame(msg); }
void CupraBorhInterpreter::configureUiLimits() {
    if (sys_ctx == nullptr) return;
    sys_ctx->normal_green = lv_color_make(200, 120, 0); // Cupra copper/gold accent
    if (sys_ctx->rpm_meter   != nullptr) lv_arc_set_range(sys_ctx->rpm_meter,   0, 15000);
    if (sys_ctx->boost_meter != nullptr) lv_bar_set_range(sys_ctx->boost_meter, 0, 100);
}
