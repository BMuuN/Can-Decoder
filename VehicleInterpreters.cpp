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

// =========================================================================
//  PLATFORM CAPABILITY PROFILES
// =========================================================================
PlatformCapabilities getPlatformCapabilities(MqbPlatformSeries gen) {
    PlatformCapabilities c;
    switch (gen) {
        case SERIES_PQ24_PL45:
            // Early generation: limited CAN, basic mechanical telemetry only.
            c.has_boost           = true;   // Turbo models have 0x288-style MAP
            c.has_oil_temp        = true;
            c.has_gear_position   = false;  // No gearbox CAN message in this era
            c.has_drive_mode      = false;
            c.has_odometer        = false;  // Odometer not broadcast on CAN
            c.has_indicators      = true;   // Basic comfort bus bit
            c.has_exterior_lights = true;   // Basic comfort bus bit
            c.has_interior_lights = false;  // Cabin light not separately broadcast
            c.has_exterior_temp   = false;  // Not all PQ24 variants broadcast temp
            c.has_rear_doors      = false;  // Rear door CAN messages absent
            c.has_media_source    = false;  // Only basic radio wake signal
            c.has_phone           = false;
            c.has_mmi             = false;
            c.has_acc_radar       = false;
            c.has_ambient_rgb     = false;
            c.has_fuel_level      = false;  // No confirmed passive CAN fuel ID for PQ24
            c.has_ev_battery      = false;
            c.has_ev_charging     = false;
            c.has_ev_regen        = false;
            c.has_motor_temp      = false;
            c.has_ota_status      = false;
            break;

        case SERIES_PQ35_46_LEGACY:
            // Golden era: ~90% of IDs are community-documented.
            c.has_boost           = true;
            c.has_oil_temp        = true;
            c.has_gear_position   = true;
            c.has_drive_mode      = true;
            c.has_odometer        = true;
            c.has_indicators      = true;
            c.has_exterior_lights = true;
            c.has_interior_lights = true;
            c.has_exterior_temp   = true;
            c.has_rear_doors      = true;
            c.has_media_source    = true;
            c.has_phone           = true;
            c.has_mmi             = true;
            c.has_acc_radar       = false;
            c.has_ambient_rgb     = false;
            c.has_fuel_level      = true;   // Passive 0x2C0 / 0x621 [MEDIUM confidence]
            c.has_ev_battery      = false;
            c.has_ev_charging     = false;
            c.has_ev_regen        = false;
            c.has_motor_temp      = false;
            c.has_ota_status      = false;
            break;

        case SERIES_SMALL_PO_SKODA:
            // Small/compact: Mix of PQ25 (limited) and MQB A0 (fuller).
            c.has_boost           = true;
            c.has_oil_temp        = true;
            c.has_gear_position   = true;
            c.has_drive_mode      = true;
            c.has_odometer        = true;
            c.has_indicators      = true;
            c.has_exterior_lights = true;
            c.has_interior_lights = true;
            c.has_exterior_temp   = true;
            c.has_rear_doors      = false;  // Compact 3-door variants may lack rear signals
            c.has_media_source    = true;
            c.has_phone           = true;
            c.has_mmi             = false;  // No MMI rotary on compact platforms
            c.has_acc_radar       = false;
            c.has_ambient_rgb     = false;
            c.has_fuel_level      = true;   // MQB A0: passive 0x12F [MEDIUM confidence]
            c.has_ev_battery      = false;
            c.has_ev_charging     = false;
            c.has_ev_regen        = false;
            c.has_motor_temp      = false;
            c.has_ota_status      = false;
            break;

        case SERIES_MQB_A_CLASS:
            // Modern MQB/MQB Evo: complex but well-decoded.
            c.has_boost           = true;
            c.has_oil_temp        = true;
            c.has_gear_position   = true;
            c.has_drive_mode      = true;
            c.has_odometer        = true;
            c.has_indicators      = true;
            c.has_exterior_lights = true;
            c.has_interior_lights = true;
            c.has_exterior_temp   = true;
            c.has_rear_doors      = true;
            c.has_media_source    = true;
            c.has_phone           = true;
            c.has_mmi             = true;
            c.has_acc_radar       = true;
            c.has_ambient_rgb     = true;
            c.has_fuel_level      = true;   // Passive 0x12F + UDS DID 0x2203 [MEDIUM confidence]
            c.has_ev_battery      = false;
            c.has_ev_charging     = false;
            c.has_ev_regen        = false;
            c.has_motor_temp      = false;
            c.has_ota_status      = false;
            break;

        case SERIES_MLB_LONG_CLASS:
            // Premium longitudinal: similar capability to MQB, no EV fields.
            c.has_boost           = true;
            c.has_oil_temp        = true;
            c.has_gear_position   = true;
            c.has_drive_mode      = true;
            c.has_odometer        = true;
            c.has_indicators      = true;
            c.has_exterior_lights = true;
            c.has_interior_lights = true;
            c.has_exterior_temp   = true;
            c.has_rear_doors      = true;
            c.has_media_source    = true;
            c.has_phone           = true;
            c.has_mmi             = true;
            c.has_acc_radar       = true;
            c.has_ambient_rgb     = false;
            c.has_fuel_level      = true;   // Passive 0x12F [LOW confidence — validate per model]
            c.has_ev_battery      = false;
            c.has_ev_charging     = false;
            c.has_ev_regen        = false;
            c.has_motor_temp      = false;
            c.has_ota_status      = false;
            break;

        case SERIES_MQB_EVO_MEB:
            // MEB electric era: no ICE boost/oil, full EV + ADAS capability.
            c.has_boost           = false;  // No turbo
            c.has_oil_temp        = false;  // No engine oil
            c.has_gear_position   = true;   // P/R/N/D/B selector
            c.has_drive_mode      = true;   // Eco / Comfort / Sport
            c.has_odometer        = true;
            c.has_indicators      = true;
            c.has_exterior_lights = true;
            c.has_interior_lights = true;
            c.has_exterior_temp   = true;
            c.has_rear_doors      = true;
            c.has_media_source    = true;
            c.has_phone           = true;
            c.has_mmi             = false;  // Physical MMI replaced by touch
            c.has_acc_radar       = true;
            c.has_ambient_rgb     = true;
            c.has_fuel_level      = false;  // No liquid fuel tank on MEB
            c.has_ev_battery      = true;
            c.has_ev_charging     = true;
            c.has_ev_regen        = true;
            c.has_motor_temp      = true;
            c.has_ota_status      = true;
            break;

        default:
            // SERIES_UNKNOWN: no capabilities asserted
            break;
    }
    return c;
}

// =========================================================================
//  PER-PLATFORM MODULE SCAN CATALOG
// =========================================================================

static const ModuleScanEntry kPq24ModuleCatalog[] = {
    {0x01, "ECM",    "Engine Control Module",            "RPM, throttle, coolant temp, boost (turbo)"},
    {0x02, "TCM",    "Transmission Control Module",      "Gear selector state (automatic only)"},
    {0x03, "ABS",    "ABS Control Unit",                 "Individual wheel speeds, brake switch"},
    {0x15, "SRS",    "Airbag/SRS Module",                "Crash sensor status, airbag deployment"},
    {0x08, "HVAC",   "Climatronic / Heater Control",     "Climate demand, blower status"},
    {0x46, "CCU",    "Central Convenience Unit",         "Door status, central locking commands"},
    {0x42, "FDM",    "Front Driver Door Module",         "Window position, mirror control"},
    {0x52, "FPM",    "Front Passenger Door Module",      "Window position"},
    {0x19, "GW",     "CAN Gateway (cluster-integrated)", "Inter-bus bridge, ignition status"},
    {0x37, "NAV",    "Navigation / Radio",               "Basic radio wake signal"},
};

static const ModuleScanEntry kPq35ModuleCatalog[] = {
    {0x01, "ECM",    "Engine Control Module",            "RPM, torque, boost, throttle, injection rate"},
    {0x02, "DSG",    "DSG Gearbox Control Unit",         "Gear position, clutch temp, selector mode"},
    {0x03, "ABS",    "ABS / ESP Module",                 "Individual wheel speeds, ESP torque intervention"},
    {0x04, "SWM",    "Steering Angle Sensor",            "Steering wheel angle, rate"},
    {0x08, "HVAC",   "Climate Control Unit",             "Target temp, dial position, fan speed"},
    {0x09, "BCM",    "Body / Central Electronics (BCM)", "Power distribution, immobiliser status"},
    {0x15, "SRS",    "Airbag Module",                    "Crash status, occupant sensing"},
    {0x16, "SWC",    "Steering Wheel Controls Module",   "Button press IDs (volume, media, phone)"},
    {0x18, "AUX",    "Auxiliary Heater",                 "Pre-heater demand, fuel ignition status"},
    {0x19, "GW",     "CAN Gateway (standalone addr 19)", "All bus bridging, wake-up arbitration"},
    {0x22, "HAL",    "Haldex AWD Controller",            "AWD torque split, clutch engagement (if AWD)"},
    {0x37, "NAV",    "Navigation Unit (RNS510/RNS315)",  "GPS destination, map data requests"},
    {0x42, "FDM",    "Front Driver Door Module",         "Window exact position (%), lock commands"},
    {0x46, "CCU",    "Convenience / Comfort Controller", "Door lock/unlock, remote key events"},
    {0x47, "SND",    "Sound / Dynaudio Amplifier",       "Audio source routing, volume level"},
    {0x52, "FPM",    "Front Passenger Door Module",      "Window position"},
    {0x56, "RAD",    "Radio Head Unit",                  "Source selection, track metadata"},
    {0x62, "RLM",    "Rear Left Door Module",            "Window, child lock"},
    {0x72, "RRM",    "Rear Right Door Module",           "Window, child lock"},
    {0x76, "PDC",    "Park Distance Control",            "Parking sensor distances"},
    {0x77, "TEL",    "Telephone / Bluetooth Module",     "Call state, contact sync"},
};

static const ModuleScanEntry kMqbModuleCatalog[] = {
    {0x01, "ECM",    "Engine Control Module",            "RPM, torque, turbo boost, throttle, driving mode"},
    {0x02, "TCU",    "DSG/Automatic Gearbox Unit",       "Gear position, clutch temps, DSG modes"},
    {0x03, "ABS",    "ABS/ESC Module",                   "Wheel speeds, yaw rate, brake pressure"},
    {0x04, "SWM",    "Steering Angle Sensor",            "Wheel angle, torque feedback"},
    {0x08, "HVAC",   "Climate Control (Climatronic)",    "Temp, blend doors, fan speed, seat heating"},
    {0x09, "BCM",    "Body Control Module / CE",         "Lighting, exterior lights, ignition bus"},
    {0x13, "ACC",    "Adaptive Cruise Control Module",   "Radar distance to lead car, target vectors"},
    {0x15, "SRS",    "Airbag / Safety Module",           "Crash sensors, belt status"},
    {0x17, "IPC",    "Instrument / Virtual Cockpit",     "Displayed speed, DIS data, mileage"},
    {0x19, "GW",     "High-Performance CAN Gateway",     "Multi-bus arbitration, all CAN/LIN bridge"},
    {0x36, "SMR",    "Seat Memory Module",               "Seat position profiles, driver ID"},
    {0x37, "NAV",    "Navigation (Discover/MIB)",        "GPS lat/lon, destination, POI"},
    {0x42, "FDM",    "Front Driver Door Module",         "Window exact %, mirror position"},
    {0x46, "CCU",    "Convenience Control Unit",         "Lock/unlock, remote start arming"},
    {0x47, "SND",    "Sound Actuator / Amplifier",       "Audio zone control, EQ"},
    {0x52, "FPM",    "Front Passenger Door Module",      "Window position"},
    {0x5F, "MIB",    "MIB Infotainment Head Unit",       "Touch coords, media metadata, phone BT"},
    {0x62, "RLM",    "Rear Left Door Module",            "Window, child lock"},
    {0x72, "RRM",    "Rear Right Door Module",           "Window, child lock"},
    {0x76, "PDC",    "Park Distance / Parking Aid",      "Ultrasonic sensor distances"},
    {0xA5, "FCM",    "Front Camera Module",              "Lane assist, road sign recognition"},
    {0x3C, "BSM",    "Blind Spot Monitor (left)",        "Side radar target detection"},
    {0x4C, "BSM",    "Blind Spot Monitor (right)",       "Side radar target detection"},
};

static const ModuleScanEntry kMebModuleCatalog[] = {
    {0x01, "VCU",    "Vehicle Control Unit (e-motor)",   "Motor RPM, torque request, regen torque"},
    {0x03, "ABS",    "ABS/ESC Module",                   "Wheel speeds, yaw, brake pressure"},
    {0x04, "SWM",    "Steering Angle Sensor",            "Angle, torque, lane-keep assist input"},
    {0x08, "HVACe",  "Heat Pump / Climate System",       "Coolant temps, heat-pump state, seat heat"},
    {0x09, "BCM",    "Body Control Module",              "All body electrics, ambient lighting RGB"},
    {0x13, "ACC",    "Travel Assist / ACC Radar",        "Forward distance, emergency braking status"},
    {0x15, "SRS",    "Airbag Module",                    "Crash sensing, belt pre-tensioner"},
    {0x17, "IPC",    "Digital Cockpit Pro",              "Speed, SoC display, range, drive profile"},
    {0x19, "ICAS1",  "ICAS1 Gateway (CAN-FD bridge)",    "Bridges Ethernet to CAN-FD all branches"},
    {0x8C, "BMS",    "High-Voltage Battery Management",  "SoC %, cell voltages, thermal state, regen"},
    {0x3D, "CHG",    "AC/DC On-Board Charger",           "Charging kW, connection status, schedule"},
    {0x5F, "MIB3",   "MIB3 Infotainment Head Unit",      "OTA status, cloud traffic, media, phone"},
    {0x42, "FDM",    "Front Driver Door Module",         "Window, mirror, ambient light zone"},
    {0x52, "FPM",    "Front Passenger Door Module",      "Window position"},
    {0x62, "RLM",    "Rear Left Door Module",            "Window, child lock"},
    {0x72, "RRM",    "Rear Right Door Module",           "Window, child lock"},
    {0x76, "PDC",    "Park Distance / Area View",        "360-degree ultrasonic sensor map"},
    {0xA5, "FCM",    "Front Camera / IQ.DRIVE",         "Lane assist, road sign, pedestrian detect"},
    {0x3C, "BSM",    "Blind Spot Monitor",               "Side-radar, lane change warning"},
    {0x37, "NAV",    "Navigation (MIB3 embedded)",       "HERE maps, cloud traffic, EV route plan"},
};

// =========================================================================
//  POWERTRAIN SPEC TABLE
// =========================================================================
// Maps chassis code + production-year window to known-valid engine calibration
// codes and gearbox codes.  Codes are the 3-4 character suffix from VAG ECU
// component strings (e.g. "04E906027HJ DADA" → code = "DADA").
// Gearbox codes follow the same convention from the TCU component string.
// This table is intentionally non-exhaustive: an unknown code is flagged as
// UNVERIFIED rather than INVALID, since the list may be incomplete.
// =========================================================================
struct PowertrainSpec {
    const char* chassis;
    int year_from;
    int year_to;
    const char* valid_engine_codes;   // comma-separated calibration suffixes
    const char* valid_gearbox_codes;  // comma-separated codes
    const char* summary;              // human-readable powertrain description
};

static const PowertrainSpec kPowertrainSpecs[] = {
    // --- AUDI ---
    // A3 / S3 / RS3  8V  MQB (2013-2020)
    {"8V", 2013, 2020,
     "CJZA,CZCA,CXSA,CJSA,DADA,CHHB,DJHB,CRBC,DKLA,BHK,DBYA",
     "JHM,QNL,MMN,LMP,MXV,HXS,DQ250,DQ381",
     "Audi A3/S3/RS3 MQB 8V — Engines: 1.0-2.5 TFSI / 2.0 TDI | Gearboxes: Manual, DSG6 DQ250, DSG7 DQ381"},
    // A3 / S3 / RS3  GY / 8Y  MQB EVO (2020+)
    {"GY", 2020, 2030,
     "DAZA,TJXA,DKLA,DADA,DPCA",
     "QNL,DQ381,MXW",
     "Audi RS3/A3 MQB EVO GY — Engines: 2.5 TFSI RS3 (DAZA), 1.0-1.5 TSI | Gearbox: DSG7 DQ381"},
    {"8Y", 2020, 2030,
     "DAZA,DKLA,DADA,DJHB,DPCA",
     "QNL,DQ381,MXW",
     "Audi A3/S3/RS3 MQB EVO 8Y — Engines: 1.0-2.5 TFSI | Gearbox: DSG7 DQ381 / Manual"},
    // A4 / S4 / RS4  8K  MLB B8 (2007-2016)
    {"8K", 2007, 2016,
     "CDNB,CAEB,CDHB,CDVC,CFKA,CMGD,CNHA,CGWB",
     "JXX,GLH,MXW,PYP,DL501,TPC",
     "Audi A4/S4/RS4 MLB B8 — Engines: 1.8-3.0 TFSI / 2.0 TDI | Gearboxes: Manual, S-Tronic DL501, Tiptronic"},
    // A4 / S4  8W / F4  MLB B9 (2015+)
    {"8W", 2015, 2030,
     "DETA,DLZB,CYNB,CYMC,DEJA,DDAA,DDWB,DAZA",
     "PYP,QTE,LVS,DL501",
     "Audi A4/S4 MLB B9 — Engines: 2.0-3.0 TFSI / 2.0 TDI | Gearboxes: Manual, S-Tronic 7"},
    {"F4", 2016, 2030,
     "DETA,DLZB,CYNB,CYMC,DEJA,DDAA",
     "PYP,QTE,DL501",
     "Audi A5/RS5 MLB B9"},
    // A6 / S6 / A7 / RS6  4G  MLB C7 (2011-2018)
    {"4G", 2011, 2018,
     "CDNB,CGWB,CREC,CTGA,CTWC,CRLD,CTTB",
     "JXX,GLH,MXW,DL501",
     "Audi A6/A7/S6/RS6 MLB C7 — Engines: 2.0-4.0 TFSI / 3.0 TDI | Gearboxes: Manual, S-Tronic, Tiptronic"},
    // A6 / A7 / RS7  4K  MLB C8 (2018+)
    {"4K", 2018, 2030,
     "DETA,DLZB,DEJB,DNAC,CREC,CTWB",
     "PYP,QTE,DL501",
     "Audi A6/A7/S6/RS7 MLB C8 — Engines: 2.0-4.0 TFSI / 3.0 TDI"},
    // A5 / S5 / RS5  8T / 8F  MLB B8 Coupe/Cabriolet (2007-2017)
    {"8T", 2007, 2017,
     "CDNB,CAEB,CDVC,CNHA,CMGD,CFKA",
     "JXX,GLH,MXW,PYP",
     "Audi A5/S5/RS5 MLB B8 — Engines: 1.8-4.2 TFSI / 2.0 TDI | Gearboxes: Manual, S-Tronic, Multitronic"},
    {"8F", 2009, 2017,
     "CDNB,CAEB,CDVC,CNHA",
     "JXX,GLH,MXW",
     "Audi A5 Cabriolet MLB B8"},
    // A8 / S8  4H  MLB D4 (2009-2017)
    {"4H", 2009, 2017,
     "CDRA,CREC,CGWB,CDSB,CEUA",
     "MXW,JXX,GLH,TPC",
     "Audi A8/S8 MLB D4 — Engines: 3.0-6.3 TFSI / 3.0 TDI | Gearboxes: Tiptronic"},
    // A8 / S8  4N  MLB EVO D5 (2017+)
    {"4N", 2017, 2030,
     "CREC,CTWB,CGWB,DETA,DLZB",
     "PYP,QTE,DL501",
     "Audi A8/S8 MLB D5 — Engines: 3.0-4.0 TFSI / 3.0 TDI"},
    // TT / TTS / TT RS  8J  PQ35 Mk2 (2006-2014)
    {"8J", 2006, 2014,
     "BWA,CAWB,CDMA,BHZ,CDLB",
     "HXS,JXX,MXW,KNS,JPT",
     "Audi TT Mk2 PQ35 — Engines: 2.0 TFSI 200/211hp, 3.2 VR6 | Gearboxes: Manual, DSG6 DQ250, S-Tronic"},
    // TT / TTS / TT RS  8S  MQB Mk3 (2014-2023)
    {"8S", 2014, 2023,
     "CHHB,DJHB,DAZA,TFKA",
     "JHM,QNL,LMP,DQ250,DQ381",
     "Audi TT Mk3 MQB — Engines: 2.0 TFSI TT/TTS, 2.5 TFSI TT RS | Gearboxes: Manual, DSG6, DSG7"},
    // Q3  8U  PQ35 (2011-2018)
    {"8U", 2011, 2018,
     "CHPA,CCZC,CRBC,CDNB,CGWB,DKRF",
     "JHM,MXV,GLH,DQ250",
     "Audi Q3 PQ35 8U — Engines: 1.4-2.0 TSI / 2.0 TDI | Gearboxes: Manual, DSG6 DQ250, 6-spd Auto"},
    // Q3 / RS Q3  F3  MQB Evo (2018+)
    {"F3", 2018, 2030,
     "DKLA,DADA,DJHB,CRBC,DFGA,CZPA",
     "QNL,JHM,DQ381",
     "Audi Q3 MQB Evo F3 — Engines: 1.5-2.0 TFSI / 2.0 TDI | Gearboxes: DSG7 DQ381, DSG6 DQ250"},
    // Q5 / SQ5  8R  MLB (2008-2017)
    {"8R", 2008, 2017,
     "CDNB,CGWB,CDVC,CNHA,CREC,CMGD",
     "JXX,GLH,MXW,DL501",
     "Audi Q5 MLB 8R — Engines: 2.0-3.0 TFSI / 2.0-3.0 TDI | Gearboxes: S-Tronic, Tiptronic, Multitronic"},
    // Q5 / SQ5  FY  MLB EVO (2017+)
    {"FY", 2017, 2030,
     "DETA,DLZB,CNHA,DEJB,DDAA",
     "PYP,QTE,DL501",
     "Audi Q5/SQ5 MLB EVO FY — Engines: 2.0-3.0 TFSI / 2.0 TDI"},
    // Q7  4L  PQ47 (2005-2015)
    {"4L", 2005, 2015,
     "CTWA,CCGA,CREC,CKDA,BHK,CDRA",
     "JXX,GLH,TIE,DL501",
     "Audi Q7 PQ47 4L — Engines: 3.0-4.2 TFSI / 3.0-6.0 TDI | Gearboxes: 6-spd Tiptronic"},
    // Q7 / SQ7 / Q8  4M  MLB EVO (2015+)
    {"4M", 2015, 2030,
     "CREC,CNHA,DETA,DEJA,CTWB",
     "PYP,QTE,DL501",
     "Audi Q7/SQ7/Q8/SQ8/RSQ8 MLB EVO 4M — Engines: 2.0-4.0 TFSI / 3.0 TDI"},
    // Q2  GA  MQB (2016+)
    {"GA", 2016, 2030,
     "CZCA,DKLA,DADA,CRBC,DJHB",
     "QNL,JHM,LMP,DQ381",
     "Audi Q2 MQB GA — Engines: 1.0-2.0 TSI / 2.0 TDI | Gearboxes: Manual, DSG6, DSG7"},
    // A1  8X  PQ25 (2010-2018)
    {"8X", 2010, 2018,
     "CHZB,CJSA,CZCA,CPWA,CAWB",
     "MXV,JHM,HXX",
     "Audi A1 PQ25 8X — Engines: 1.0-1.8 TFSI | Gearboxes: Manual, DSG6 DQ250"},
    // A1 Sportback  GB  MQB A0 (2018+)
    {"GB", 2018, 2030,
     "DKLA,CHZB,CZCA,DADA,DJHB",
     "QNL,MXV,DQ381",
     "Audi A1 Sportback MQB A0 GB — Engines: 1.0-2.0 TSI | Gearboxes: Manual, DSG7 DQ381"},
    // A3  8L  PQ24 (1996-2003)
    {"8L", 1996, 2003,
     "AXX,BFQ,BHE,AGU,ARZ,BAM,ATC,APX",
     "DUU,DKG,FZP,EGZ",
     "Audi A3 PQ24 8L — Engines: 1.6-1.8T / 1.9 TDI | Gearboxes: Manual, DSG5"},
    // A4 / S4 / Cabriolet  8E / 8H  PQ24 B6/B7 (2000-2009)
    {"8E", 2000, 2009,
     "AXX,BWT,BFB,ALT,AWA,BNA,BGB,AVB,BHF,BEX",
     "DQS,HXS,DYL,GBT,ELU,LUK",
     "Audi A4/S4/RS4 B6/B7 PQ24 — Engines: 1.6-4.2 V8 / 2.0 TDI | Gearboxes: Manual, DSG6"},
    {"8H", 2002, 2009,
     "AXX,BWT,BFB,ALT",
     "DQS,GBT",
     "Audi A4 Cabriolet PQ24 B6/B7"},
    // A6 / S6 / RS6  4F  C6 PQ35-class (2004-2011)
    {"4F", 2004, 2011,
     "BXA,AXZ,BYU,AUK,BNA,BPP,BLB,BNG,CALA",
     "JXX,GBT,MXW,DL501",
     "Audi A6/S6/RS6/A6 Avant C6 4F — Engines: 2.4-5.2 V10 / 2.7-3.0 TDI"},
    // Porsche Cayenne  92  MLB (2010-2018)
    {"92", 2010, 2018,
     "CTWA,CREC,CGWB,CDRA",
     "PYP,JXX,TPC",
     "Porsche Cayenne 92A — Engines: 3.0-4.8 TFSI / 3.0 TDI | Gearboxes: Tiptronic, PDK"},
    // Porsche Macan  9B  MLB (2014-2023)
    {"9B", 2014, 2023,
     "CPDB,CMNA,CREC,CJXC",
     "PYP,JXX,PDK",
     "Porsche Macan 95B — Engines: 2.0-3.6 TFSI | Gearboxes: PDK"},

    // --- VOLKSWAGEN ---
    // Golf 7 / GTI / Golf R  MQB (2012-2021)
    {"5G", 2012, 2021,
     "CJZA,CHPA,CZCA,DADA,CJSA,CHHB,DJHB,CRBC,CRKB,DKRF,CHZB,DPCA",
     "MQS,JHM,QNL,QSB,LMP,DQ250,DQ381",
     "VW Golf Mk7/GTI/R MQB 5G — Engines: 1.0-2.0 TSI / 2.0 TDI | Gearboxes: Manual 6, DSG6 DQ250, DSG7 DQ381"},
    {"BA", 2012, 2021,
     "CJZA,CHPA,CZCA,DADA,CJSA,CHHB,DJHB,CRBC,CRKB,DKRF",
     "MQS,JHM,QNL,QSB,LMP",
     "VW Golf 7 Variant/Alltrack MQB BA"},
    {"AM", 2012, 2021,
     "CJZA,CHPA,CZCA,DADA,CJSA,CHHB,DJHB,CRBC",
     "MQS,JHM,QNL,LMP",
     "VW Golf Sportsvan MQB AM"},
    {"AU", 2012, 2021,
     "CJZA,CHPA,CZCA,DADA,CJSA,CHHB,DJHB,CRBC",
     "MQS,JHM,QNL,LMP,DSU",
     "VW Golf GTE/e-Golf MQB AU"},
    // Golf 8 / GTI / GTE / R  MQB EVO (2019+)
    {"CD", 2019, 2030,
     "DKRF,DKRD,DADA,DPCA,DNPA,DLBA,DTSA,DFGA,CZPA",
     "MQS,QNL,QSB,DSU,DQ381",
     "VW Golf 8/GTI/GTE/R MQB EVO CD — Engines: 1.0-2.0 TSI eHybrid / 2.0 TDI | Gearboxes: Manual, DSG7, DQ400e"},
    // Passat B6 / CC  PQ35 (2005-2015)
    {"3C", 2005, 2010,
     "BWA,CAWB,CDNB,BKD,BMP,BVY,CJSA",
     "HXS,GLH,GBT,MXV,DQ250",
     "VW Passat B6/CC PQ35 3C — Engines: 1.8-3.6 V6 FSI / 2.0 TDI | Gearboxes: Manual, DSG6, 6-spd Auto"},
    {"AN", 2010, 2015,
     "BWA,CAWB,CDNB,BKD,CFF,CFG,CDNC",
     "HXS,GLH,GBT,MXV",
     "VW Passat B7 PQ35 AN"},
    // Passat B8  MQB (2014+)
    {"3G", 2014, 2030,
     "CZPA,DADA,CJSA,CRBC,DGCA,CZD,DPCA",
     "MQS,QNL,JHM,DSU,DQ381",
     "VW Passat B8 MQB 3G — Engines: 1.4-2.0 TSI / 2.0 TDI | Gearboxes: Manual, DSG6 DQ250, DSG7 DQ381"},
    {"CB", 2014, 2030,
     "CZPA,DADA,CJSA,CRBC,DGCA",
     "MQS,QNL,JHM",
     "VW Passat B8 Variant MQB CB"},
    // Passat B9  MQB EVO (2023+)
    {"A3", 2023, 2030,
     "DETA,DADA,DGCA,DDAA,DFGA",
     "QNL,QTE,DQ381",
     "VW Passat B9 MQB EVO A3"},
    // Scirocco  PQ35 (2008-2017)
    {"13", 2008, 2017,
     "BWA,CAWB,CDMA,CDNB,CFGB,CHHB",
     "HXS,JHM,MXV,GBT,DQ250",
     "VW Scirocco PQ35 13 — Engines: 1.4 TSI to 2.0 TFSI / 2.0 TDI"},
    // Tiguan Mk1  PQ35 (2007-2017)
    {"5N", 2007, 2017,
     "BWA,CAWB,CDNB,CRBC,CFFA,CAWB",
     "HXS,JHM,GLH,MXV,DQ250",
     "VW Tiguan Mk1 PQ35 5N — Engines: 1.4-2.0 TSI / 2.0 TDI | Gearboxes: Manual, DSG6 DQ250, 6-spd Auto"},
    // Tiguan Mk2  MQB (2016-2023)
    {"AD", 2016, 2023,
     "CZPA,DKRF,DADA,CRBC,DFGA,DGCA",
     "QNL,JHM,MQS,DQ381,DQ250",
     "VW Tiguan Mk2 MQB AD — Engines: 1.4-2.0 TSI / 2.0 TDI | Gearboxes: Manual, DSG6 DQ250, DSG7 DQ381"},
    {"AX", 2016, 2023,
     "CZPA,DKRF,DADA,CRBC,DFGA",
     "QNL,JHM,DQ381",
     "VW Tiguan Allspace MQB AX"},
    // Tiguan Mk3  MQB EVO (2023+)
    {"CT", 2023, 2030,
     "DADA,DFGA,DNPA,DPCA",
     "QNL,QSB,DQ381",
     "VW Tiguan Mk3 MQB EVO CT"},
    // Arteon  MQB (2017+)
    {"3H", 2017, 2030,
     "CHHB,DJHB,DADA,CRBC,DFGA,CZPA",
     "QNL,MQS,DQ381",
     "VW Arteon MQB 3H — Engines: 1.5-2.0 TSI / 2.0 TDI"},
    // Polo PQ25 (2009-2018)
    {"6R", 2009, 2018,
     "CHZB,CZCA,CLNA,CRPC,CAYA,DKLA",
     "MXV,MXJ,JHM,HXX",
     "VW Polo PQ25 6R — Engines: 1.0-1.8 TSI / 1.4 TDI"},
    {"6C", 2014, 2018,
     "CHZB,CZCA,CLNA",
     "MXV,JHM",
     "VW Polo PQ25 6C"},
    // Polo MQB A0 (2017+)
    {"AW", 2017, 2030,
     "CHZB,DKLA,CZCA,DADA,DJHB",
     "QNL,MXV,DQ381",
     "VW Polo MQB A0 AW — Engines: 1.0-2.0 TSI"},
    // Golf Mk5 / Mk6 / Jetta  PQ35 (2003-2015)
    {"1K", 2003, 2009,
     "BLG,CAVE,BJX,BWA,CAWB,BKD,CJSA,BAG,BLP,CHHB",
     "GBT,HXS,JLN,MXV,DQ250",
     "VW Golf Mk5 PQ35 1K — Engines: 1.4 TSI to 2.0 TFSI GTI / R32 / 2.0 TDI"},
    {"5K", 2008, 2013,
     "BLG,CAVE,BJX,CAWB,CHHB,CDNB,BKD,CRKB",
     "GBT,HXS,JHM,MXV",
     "VW Golf Mk6/GTI/R PQ35 5K"},
    {"AJ", 2005, 2015,
     "BLG,CAVE,BJX,BWA,CAWB,BKD,CDNB",
     "GBT,HXS,JLN,MXV",
     "VW Jetta PQ35 AJ"},
    // Golf Mk4 / Bora  PQ24 (1997-2006)
    {"1J", 1997, 2006,
     "AXX,AGU,APX,AUQ,BFQ,AVR,AZM,ARL,BHW",
     "DUU,DKG,EPE,FZP,HXS",
     "VW Golf Mk4/Bora PQ24 1J — Engines: 1.6-1.8T / R32 / TDI"},
    // Polo Mk4 9N  PQ24 (2001-2010)
    {"9N", 2001, 2010,
     "BBZ,BKY,BME,AWY,BUD,AXR",
     "DQP,HXX,EGH",
     "VW Polo Mk4 PQ24 9N — Engines: 1.2-1.8 / TDI"},
    // Bora  PQ24 (1998-2006)
    {"1C", 1998, 2006,
     "AGU,AXX,APX,AUQ,AVF,AZM,ARJ",
     "DUU,DKG,FZP",
     "VW Bora PQ24 1C"},

    // --- SEAT / CUPRA ---
    // Leon Mk2  PQ35 (2005-2012)
    {"1P", 2005, 2012,
     "BWA,CAWB,BKD,BMP,CDMA,CHHB,CDNB",
     "HXS,GBT,MXV,DQ250",
     "Seat Leon Mk2/Cupra PQ35 1P — Engines: 1.4-2.0 TSI Cupra / 2.0 TDI"},
    // Leon Mk3 / Cupra  MQB (2012-2020)
    {"5F", 2012, 2020,
     "CJZA,CZCA,CHHB,DJHB,CRKB,CRBC,DKLA",
     "LMP,JHM,QNL,DQ250,DQ381",
     "Seat Leon Mk3/Cupra MQB 5F — Engines: 1.2-2.0 TSI / 2.0 TDI | Gearboxes: Manual, DSG6, DSG7"},
    // Cupra Leon / Formentor  MQB EVO (2020+)
    {"KL", 2020, 2030,
     "DJHB,DKRG,DNPA,DTSA",
     "QNL,DQ381,DSU",
     "Cupra Leon/Formentor MQB EVO KL — Engines: 2.0 TSI / 1.4 eHybrid | Gearbox: DSG7 DQ381"},
    // Ibiza / Arona  MQB A0 (2017+)
    {"KJ", 2017, 2030,
     "CHZB,DKLA,CZCA,DADA,DJHB",
     "QNL,MXV,DQ381",
     "Seat Ibiza/Arona MQB A0 KJ — Engines: 1.0-1.5 TSI"},

    // --- SKODA ---
    // Octavia Mk2  PQ35 (2004-2013)
    {"1Z", 2004, 2013,
     "BWA,BKD,BMP,CDNB,CAWB,CHHB",
     "HXS,GBT,MXV,DQ250",
     "Skoda Octavia Mk2/vRS PQ35 1Z — Engines: 1.6-2.0 TSI RS / 2.0 TDI | Gearboxes: Manual, DSG6"},
    // Octavia Mk3  MQB (2012-2020)
    {"5E", 2012, 2020,
     "CJZA,CZCA,CJSA,CHHB,CRKB,CRBC,DJHB",
     "MXV,JHM,QNL,DQ250,DQ381",
     "Skoda Octavia Mk3/vRS MQB 5E — Engines: 1.0-2.0 TSI RS / 2.0 TDI"},
    // Octavia Mk4  MQB EVO (2020+)
    {"NX", 2020, 2030,
     "DJHB,DADA,DKRF,DFGA,DPCA",
     "QNL,MQS,DQ381",
     "Skoda Octavia Mk4 MQB EVO NX"},
    // Superb 3T  PQ35-class (2008-2015)
    {"3T", 2008, 2015,
     "BWA,CDNB,CRBC,CFGB,CCZ,CHHB",
     "HXS,GLH,GBT,MXV,DQ250",
     "Skoda Superb 3T PQ35 — Engines: 1.8-3.6 V6 / 2.0 TDI | Gearboxes: Manual, DSG6, 6-spd Auto"},
    // Superb  MQB (2015+)
    {"3V", 2015, 2030,
     "CZPA,DADA,CJSA,CRBC,DFGA,DPCA",
     "MXV,JHM,QNL,DQ381",
     "Skoda Superb MQB 3V — Engines: 1.4-2.0 TSI / 2.0 TDI"},

    // --- MEB ELECTRIC (no ICE engine code; TCU is a single-speed reducer) ---
    {"E1", 2019, 2030, "VCU,MCU,BEV,EEM", "EDS,SSD", "VW ID.3 MEB — Electric motor; single-speed reducer"},
    {"E2", 2020, 2030, "VCU,MCU,BEV,EEM", "EDS,SSD", "VW ID.4 / ID.4 GTX MEB — Electric"},
    {"E3", 2021, 2030, "VCU,MCU,BEV,EEM", "EDS,SSD", "VW ID.5 MEB — Electric"},
    {"E9", 2022, 2030, "VCU,MCU,BEV,EEM", "EDS,SSD", "VW ID.Buzz MEB — Electric"},
    {"EB", 2022, 2030, "VCU,MCU,BEV,EEM", "EDS,SSD", "VW ID.Buzz Cargo MEB — Electric"},
    {"FZ", 2021, 2030, "VCU,MCU,BEV,EEM", "EDS,SSD", "Audi Q4 e-tron MEB — Electric"},
    {"I1", 2020, 2030, "VCU,MCU,BEV,EEM", "EDS,SSD", "Skoda Enyaq iV MEB — Electric"},
    {"K1", 2021, 2030, "VCU,MCU,BEV,EEM", "EDS,SSD", "Cupra Born MEB — Electric"},
};

// Returns the first matching PowertrainSpec for the given chassis + year, or nullptr.
static const PowertrainSpec* getPowertrainSpec(const char* chassis, int year) {
    if (chassis == nullptr || chassis[0] == '\0') return nullptr;
    for (size_t i = 0; i < sizeof(kPowertrainSpecs) / sizeof(kPowertrainSpecs[0]); i++) {
        const PowertrainSpec& s = kPowertrainSpecs[i];
        if (strcmp(s.chassis, chassis) == 0 && year >= s.year_from && year <= s.year_to) {
            return &s;
        }
    }
    return nullptr;
}

// Returns true if 'code' matches any comma-separated entry in 'list' (case-insensitive).
static bool codeInList(const char* code, const char* list) {
    if (code == nullptr || list == nullptr || code[0] == '\0' || list[0] == '\0') return false;
    const size_t code_len = strlen(code);
    const char* p = list;
    while (*p) {
        const char* comma = strchr(p, ',');
        size_t entry_len = comma ? (size_t)(comma - p) : strlen(p);
        if (entry_len == code_len && strncasecmp(code, p, code_len) == 0) return true;
        p = comma ? comma + 1 : p + entry_len;
        if (!comma) break;
    }
    return false;
}

void validatePowertrainCodes(const char* chassis, int year,
                             const char* engine_code, const char* gearbox_code,
                             bool* engine_valid_out, bool* gearbox_valid_out,
                             bool* combination_valid_out) {
    *engine_valid_out     = false;
    *gearbox_valid_out    = false;
    *combination_valid_out = false;

    const PowertrainSpec* spec = getPowertrainSpec(chassis, year);
    if (spec == nullptr) return;

    const bool have_engine  = engine_code  && engine_code[0];
    const bool have_gearbox = gearbox_code && gearbox_code[0];

    if (have_engine)  *engine_valid_out  = codeInList(engine_code,  spec->valid_engine_codes);
    if (have_gearbox) *gearbox_valid_out = codeInList(gearbox_code, spec->valid_gearbox_codes);

    // Combination is valid if every retrieved code is individually valid.
    *combination_valid_out = (have_engine  ? *engine_valid_out  : true) &&
                             (have_gearbox ? *gearbox_valid_out : true) &&
                             (have_engine || have_gearbox);
}

const char* lookupExpectedPowertrainDescription(const char* chassis, int year) {
    const PowertrainSpec* spec = getPowertrainSpec(chassis, year);
    return spec ? spec->summary : nullptr;
}

PlatformCapabilities getPlatformCapabilities(MqbPlatformSeries gen);  // fwd-declared in header

const ModuleScanEntry* getPlatformModuleCatalog(MqbPlatformSeries gen, size_t* out_count) {
    switch (gen) {
        case SERIES_PQ24_PL45:
            if (out_count) *out_count = sizeof(kPq24ModuleCatalog) / sizeof(kPq24ModuleCatalog[0]);
            return kPq24ModuleCatalog;
        case SERIES_PQ35_46_LEGACY:
        case SERIES_SMALL_PO_SKODA:
            if (out_count) *out_count = sizeof(kPq35ModuleCatalog) / sizeof(kPq35ModuleCatalog[0]);
            return kPq35ModuleCatalog;
        case SERIES_MQB_A_CLASS:
        case SERIES_MLB_LONG_CLASS:
            if (out_count) *out_count = sizeof(kMqbModuleCatalog) / sizeof(kMqbModuleCatalog[0]);
            return kMqbModuleCatalog;
        case SERIES_MQB_EVO_MEB:
            if (out_count) *out_count = sizeof(kMebModuleCatalog) / sizeof(kMebModuleCatalog[0]);
            return kMebModuleCatalog;
        default:
            if (out_count) *out_count = 0;
            return nullptr;
    }
}

const char* getPlatformEnglishSummary(MqbPlatformSeries gen) {
    switch (gen) {
        case SERIES_PQ24_PL45:
            return
                "PQ24/PQ34/PL45 (Early Gen, ~2001-2005)\n"
                "Drive Train Bus 500kbps: ECM, TCM, ABS, Airbag.\n"
                "Comfort Bus 100kbps: Convenience, Door Modules, Climatronic.\n"
                "Infotainment Bus 100kbps: Radio (basic wake signal only).\n"
                "Readable signals: RPM, throttle, coolant temp, boost (turbo),\n"
                "basic door status, indicators, headlights, ignition state.\n"
                "Limitations: no odometer broadcast, no gear CAN msg, limited comfort data.";
        case SERIES_PQ35_46_LEGACY:
            return
                "PQ35/PQ46/PL46 (Golden Era, ~2004-2015)\n"
                "Drive Train Bus 500kbps: ECM, DSG, ABS, Steering, Airbag, Haldex AWD.\n"
                "Comfort Bus 100kbps: BCM, Convenience, Doors, Climate, Park Assist, Aux Heater.\n"
                "Infotainment Bus 100kbps: Radio, Navigation, Sound, Bluetooth, Steering-Wheel Ctrl.\n"
                "Readable signals: Accurate torque, boost, injection rate, DSG clutch temp,\n"
                "individual wheel speeds, steering angle, window positions, lock commands,\n"
                "wiper status, light switch positions, MMI key codes, media metadata, phone call state.";
        case SERIES_MQB_A_CLASS:
        case SERIES_SMALL_PO_SKODA:
            return
                "MQB / MLB (Modern Modular, ~2013-2020)\n"
                "Drive Train Bus 500kbps: ECM, DSG/Auto, ABS/ESC, ACC Radar, Front Camera.\n"
                "Extended Bus 500kbps: Lane Assist, Blind-Spot Monitoring.\n"
                "Comfort Bus 500kbps: BCM, Climatronic, Seat Memory.\n"
                "Infotainment Bus 500kbps/MOST: MIB Head Unit, Virtual Cockpit.\n"
                "Readable signals: Drive mode (Eco/Normal/Sport), advanced pedal rate, steering torque,\n"
                "radar target vectors, yaw rate, brake pressure, RGB ambient lighting, HVAC blend doors,\n"
                "GPS lat/lon, oil level/quality, OBD2 UDS diagnostics.";
        case SERIES_MLB_LONG_CLASS:
            return
                "MLB Longitudinal (Premium, ~2007-present)\n"
                "Same capability as MQB with addition of FlexRay on later variants.\n"
                "Unique: transmission torque convertor data, suspension mode, air suspension status.\n"
                "Modules include premium-tier active chassis, night-vision, panoramic roof control.";
        case SERIES_MQB_EVO_MEB:
            return
                "MQB Evo / MEB Electric (~2020-present)\n"
                "Architecture: ICAS1 gateway, CAN-FD buses, 100Base-T1 Automotive Ethernet.\n"
                "Autonomous Bus (CAN-FD): Front Radar, LiDAR, Surround Cameras, IQ.DRIVE.\n"
                "Drivetrain/Thermal Bus (CAN-FD): Motor VCU, HV Battery BMS (addr 0x8C), Heat Pump.\n"
                "Readable signals: Battery SoC %, cell voltage delta, charging kW, regen torque,\n"
                "motor temp, OTA update status, cloud traffic warnings, Matrix LED beam patterns,\n"
                "capacitive steering-wheel touch (hand-on/off), ambient light RGB zones.\n"
                "Note: SFD-protected ECUs require authorisation for deeper diagnostics.";
        default:
            return "Unknown platform – no interpretation data available.";
    }
}
