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
