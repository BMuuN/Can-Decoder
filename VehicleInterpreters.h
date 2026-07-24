#ifndef VEHICLE_INTERPRETERS_H
#define VEHICLE_INTERPRETERS_H

#include <Arduino.h>
#include "driver/twai.h"
#include <lvgl.h>
#include "freertos/portmacro.h"
#include <atomic>

// --- CAN FILTER HELPER ---
// SJA1000/ESP32 TWAI: standard 11-bit IDs are stored in bits [31:21] of the
// 32-bit acceptance_code/acceptance_mask register.
static constexpr int CAN_FILTER_ID_SHIFT = 21;

// Decode raw VW/Audi temperature byte (value = Celsius + 40 offset, uint8_t).
// Promotes to int before subtracting to avoid uint8_t underflow at cold start
// (raw < 40 would wrap to 216+ as unsigned).
static inline float decode_temperature_offset(uint8_t raw) {
    return (float)((int)raw - 40);
}

// --- ADVANCED VEHICLE DECODING ENUMS ---
enum MqbPlatformSeries {
    SERIES_UNKNOWN,
    SERIES_MQB_A_CLASS,
    SERIES_MLB_LONG_CLASS,
    SERIES_PQ35_46_LEGACY,
    SERIES_SMALL_PO_SKODA
};

enum GearSelectorPosition : uint8_t {
    GEAR_SELECTOR_UNKNOWN = 0,
    GEAR_SELECTOR_PARK,
    GEAR_SELECTOR_REVERSE,
    GEAR_SELECTOR_NEUTRAL,
    GEAR_SELECTOR_DRIVE,
    GEAR_SELECTOR_SPORT,
    GEAR_SELECTOR_MANUAL
};

// --- GLOBAL TELEMETRY STRUCT LAYOUT TEMPLATES ---
struct LiveTelemetryMetrics {
    float engine_rpm = 0.0;
    float boost_bar = 0.0;
    float peak_boost_bar = 0.0;
    float oil_temp = 0.0;
    float coolant_temp = 0.0;
    bool driver_door_open = false;
    float target_temp = 0.0;
    uint8_t mmi_key_code = 0x00;
    // --- Extended telemetry fields ---
    float vehicle_speed = 0.0;        // km/h, platform-specific speed frame
    float throttle_pct = 0.0;         // 0–100 %, accelerator pedal position
    float exterior_temp = 0.0;        // °C, outside ambient temperature
    bool passenger_door_open = false;
    bool rear_left_door_open = false;
    bool rear_right_door_open = false;
    bool handbrake_active = false;
    bool left_indicator_active = false;
    bool left_indicator_known = false;
    bool right_indicator_active = false;
    bool right_indicator_known = false;
    bool parking_lights_active = false;
    bool parking_lights_known = false;
    bool low_beam_active = false;
    bool low_beam_known = false;
    bool high_beam_active = false;
    bool high_beam_known = false;
    bool interior_lights_active = false;
    bool interior_lights_known = false;
    bool sport_mode_active = false;
    bool sport_mode_known = false;
    GearSelectorPosition gear_position = GEAR_SELECTOR_UNKNOWN;
    bool gear_position_known = false;
    uint8_t selected_gear = 0;
    bool selected_gear_known = false;
    float odometer_km = 0.0f;
    bool odometer_valid = false;
    uint8_t infotainment_source_code = 0;
    bool infotainment_source_known = false;
    uint8_t infotainment_track = 0;
    bool infotainment_track_known = false;
    bool phone_call_active = false;
    bool phone_call_known = false;
    bool diagnostics_seen = false;
    bool mil_active = false;
    bool mil_status_known = false;
    uint8_t stored_dtc_count = 0;
    float control_module_voltage = 0.0f;
    bool control_module_voltage_known = false;
    uint8_t last_diag_service = 0x00;
    uint8_t last_diag_pid = 0x00;
    uint16_t last_diag_source = 0x000;
    uint32_t diag_response_counter = 0;
};

struct BoolSignalMapping {
    uint32_t frame_id;
    uint8_t byte_index;
    uint8_t bit_mask;
    bool LiveTelemetryMetrics::*value_field;
    bool LiveTelemetryMetrics::*known_field;
};

struct ByteSignalMapping {
    uint32_t frame_id;
    uint8_t byte_index;
    uint8_t bit_mask;
    uint8_t bit_shift;
    uint8_t LiveTelemetryMetrics::*value_field;
    bool LiveTelemetryMetrics::*known_field;
};

struct OdometerSignalMapping {
    uint32_t frame_id;
    uint8_t start_byte;
    uint8_t byte_count;
    float scale;
    bool big_endian;
};

bool applyBoolSignalMappings(twai_message_t &msg, const BoolSignalMapping* mappings, size_t count);
bool applyByteSignalMappings(twai_message_t &msg, const ByteSignalMapping* mappings, size_t count);
bool applyOdometerSignalMapping(twai_message_t &msg, const OdometerSignalMapping& mapping);
void applyGenericGearFrame(twai_message_t &msg, uint32_t frame_id, uint8_t gear_byte_index, uint8_t sport_byte_index, uint8_t sport_mask);
void parsePassiveDiagnosticsFrame(twai_message_t &msg);
const char* gearSelectorPositionLabel(GearSelectorPosition position);
const char* infotainmentSourceLabel(uint8_t code);
const char* availabilityLabel(bool known, bool active, const char* active_label, const char* inactive_label, const char* unknown_label = "UNAVAILABLE");
const char* openClosedLabel(bool open);

struct DecodedVehicleMetrics {
    const char* brand = "VAG MOTOR CORP";
    const char* model_name = "GENERIC MODEL ARCHITECTURE";
    const char* electrical_bus = "STANDARD INFRASTRUCTURE CAN";
    int production_year = 0;
    MqbPlatformSeries network_generation = SERIES_UNKNOWN;
};

// --- ABSTRACT MULTI-VEHICLE PARSER INTERFACE BLUEPRINT ---
class BaseVehicleInterpreter {
public:
    virtual ~BaseVehicleInterpreter() {}
    virtual void interpretDriveTrain(twai_message_t &msg) = 0;
    virtual void interpretComfort(twai_message_t &msg) = 0;
    virtual void interpretInfotainment(twai_message_t &msg) = 0;
    virtual void configureUiLimits() = 0;
};

// =========================================================================
//  CLEAN GLOBAL STORAGE CONFIGURATIONS
// =========================================================================
struct GlobalFrameworkContext {
    LiveTelemetryMetrics metrics;
    DecodedVehicleMetrics profile;
    BaseVehicleInterpreter* interpreter = nullptr;

    // UI Layout tracking pointer references
    lv_obj_t* tv = nullptr;
    lv_obj_t* rpm_meter = nullptr;
    lv_obj_t* boost_meter = nullptr;
    lv_obj_t* oil_arc = nullptr;
    lv_obj_t* coolant_arc = nullptr;

    lv_color_t normal_green;
};

// --- GLOBAL EXTERN LINKS SHARED ACROSS ALL WORKSPACES ---
extern GlobalFrameworkContext* sys_ctx;
extern DecodedVehicleMetrics active_vehicle_profile;
extern twai_handle_t twai_ports[];

extern lv_obj_t *rpm_meter;
extern lv_obj_t *boost_meter;
extern lv_color_t color_normal_green;

// --- MULTICORE SYNCHRONISATION PRIMITIVES ---
// g_metrics_mux: spinlock protecting sys_ctx->metrics fields against Core-0/Core-1 races.
// g_interpreter_mutex: FreeRTOS mutex protecting sys_ctx->interpreter and active_vehicle_profile.
extern portMUX_TYPE      g_metrics_mux;
extern SemaphoreHandle_t g_interpreter_mutex;

// g_twai0_valid: atomic flag that is cleared to false before the bus-off recovery on Core 1
// uninstalls the TWAI port-0 driver, and restored to true once the reinstall completes.
// Core 0's runBenchTelemetrySimulation checks this flag before every twai_transmit_v2 call
// on port 0, preventing a use-after-free crash when the handle is momentarily invalid.
extern std::atomic<bool> g_twai0_valid;

// =========================================================================
//  BENCH COMPACT GENERIC FALLBACK INTERPRETER BLUEPRINT
// =========================================================================
class GenericVehicleInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override {}
    void interpretComfort(twai_message_t &msg) override {}
    void interpretInfotainment(twai_message_t &msg) override {}
    void configureUiLimits() override;
};

// =========================================================================
//  FORWARD DECLARATIONS: GROUP 1 - MQB MATRIX VEHICLES (Platform_MQB_Matrix.h)
// =========================================================================
class AudiS38VInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiRS3GYInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwGolf7Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwGolf8Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwPassatB8Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwPassatB9Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwTiguanMk2Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwTiguanMk3Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwArteonInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SeatLeonMk3Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class CupraLeonFormentorInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SkodaOctaviaMk3Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SkodaOctaviaMk4Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SkodaSuperbMQBInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ3MQBInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiTTMk3Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ2Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};

// =========================================================================
//  FORWARD DECLARATIONS: GROUP 2 - PQ LEGACY INFRASTRUCTURE (Platform_PQ_Legacy.h)
// =========================================================================
class AudiS38PInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA6C6Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ3PQ35Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ74LInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiTTMk2Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwGolf56Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwPassatB67Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwSciroccoInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwTiguanMk1Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SeatLeonMk2Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SkodaOctaviaMk2Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SkodaSuperb3TInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};

// =========================================================================
//  FORWARD DECLARATIONS: GROUP 3 - MLB LONGITUDINAL BUS (Platform_MLB_Longitudinal.h)
// =========================================================================
class AudiA4MLB8KInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA4MLB8WInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA6MLBC7Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA6MLBC8Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA5MLBB8Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA8MLBD4Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA8MLBD5Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ5MLB8RInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ5MLBFYInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiQ7MLB4MInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class PorscheCayenne92Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class PorscheMacan9BInterpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};

// =========================================================================
//  FORWARD DECLARATIONS: GROUP 4 - SMALL COMPACT COMPACT PLATFORMS (Platform_Small_Compact.h)
// =========================================================================
class AudiA1PQ25Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class AudiA1MQBA0Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwPoloPQ25Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class VwPoloMQBA0Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};
class SeatIbizaMQBA0Interpreter : public BaseVehicleInterpreter {
public:
    void interpretDriveTrain(twai_message_t &msg) override; void interpretComfort(twai_message_t &msg) override; void interpretInfotainment(twai_message_t &msg) override; void configureUiLimits() override;
};

#endif // VEHICLE_INTERPRETERS_H
