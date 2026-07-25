#include "VehicleSimulator.h"
#include <math.h>

// =========================================================================
//  CLASS METHOD DEFINITIONS: PLATFORM-AWARE TELEMETRY SIMULATOR CORE
// =========================================================================

void runBenchTelemetrySimulation(float target_rpm, float target_boost, float target_oil, float target_h2o) {
    if (sys_ctx == nullptr) return;
    static uint32_t bench_tick = 0;
    bench_tick++;

    const MqbPlatformSeries gen = active_vehicle_profile.network_generation;
    const bool is_meb      = (gen == SERIES_MQB_EVO_MEB);
    const bool is_pq24     = (gen == SERIES_PQ24_PL45);

    const uint32_t lighting_phase = (bench_tick / 40) % 3;
    const bool left_indicator = lighting_phase == 0;
    const bool right_indicator = lighting_phase == 1;
    const bool cabin_light = lighting_phase == 2;
    const bool sport_mode = target_rpm >= 3600.0f;
    const uint8_t displayed_gear = target_rpm < 1400.0f ? 1 : target_rpm < 2200.0f ? 2 : target_rpm < 3000.0f ? 3 : target_rpm < 3900.0f ? 4 : target_rpm < 4700.0f ? 5 : 6;
    const uint8_t infotainment_source = (bench_tick / 120) % 2 == 0 ? 0x02 : 0x03;
    const float vehicle_speed = target_rpm * 0.018f;
    const float throttle_pct = (target_rpm / 5500.0f) * 70.0f;
    const float exterior_temp = 18.0f;
    const float odometer_km = 182345.6f + ((bench_tick % 50000000UL) * 0.02f); // Wrap after long bench sessions to avoid unbounded counter growth.
    const uint8_t infotainment_track = 7 + (bench_tick / 120) % 6;

    // MEB-specific bench values: simulate a battery discharging from 80% to 20% over a drive cycle.
    const float ev_soc_pct      = is_meb ? (80.0f - (target_rpm / 5500.0f) * 60.0f) : 0.0f;
    const float ev_hv_voltage   = is_meb ? (350.0f + ev_soc_pct * 0.5f)             : 0.0f; // 350-390 V range
    const float ev_charging_kw  = is_meb && target_rpm < 900.0f ? 11.0f : 0.0f;            // Bench: charging at low "RPM"
    const bool  ev_charging_act = is_meb && ev_charging_kw > 0.0f;
    const float ev_regen_torque = is_meb ? -(throttle_pct > 5.0f ? 0.0f : 80.0f)   : 0.0f; // Regen when not accelerating
    const float ev_motor_temp   = is_meb ? (40.0f + (target_rpm / 5500.0f) * 50.0f): 0.0f; // 40-90°C range

    // Fuel bench simulation: oscillate between 35 L and 45 L over a slow cycle.
    // MEB has no fuel tank so the field is left unknown for that platform.
    const bool has_fuel = !is_meb && (gen != SERIES_PQ24_PL45);
    const float fuel_bench_liters = has_fuel ? (40.0f + 5.0f * sinf((float)bench_tick * 0.002f)) : -1.0f;

    // C-4: Protect the metrics write with the spinlock so Core 1's
    //      updateUIElements/parsers never observe a partially-written struct.
    portENTER_CRITICAL(&g_metrics_mux);
    sys_ctx->metrics.engine_rpm   = target_rpm;
    // MEB: no turbo boost, no engine oil — clear those to avoid showing stale ICE values.
    sys_ctx->metrics.boost_bar    = is_meb ? 0.0f : target_boost;
    sys_ctx->metrics.oil_temp     = is_meb ? 0.0f : target_oil;
    sys_ctx->metrics.coolant_temp = target_h2o;
    // Derived bench values for new extended metrics
    sys_ctx->metrics.vehicle_speed  = vehicle_speed; // ~100 km/h at 5500 rpm
    sys_ctx->metrics.throttle_pct   = throttle_pct;
    sys_ctx->metrics.exterior_temp  = exterior_temp; // static bench ambient
    sys_ctx->metrics.left_indicator_active = left_indicator;
    sys_ctx->metrics.left_indicator_known = true;
    sys_ctx->metrics.right_indicator_active = right_indicator;
    sys_ctx->metrics.right_indicator_known = true;
    sys_ctx->metrics.parking_lights_active = true;
    sys_ctx->metrics.parking_lights_known = true;
    sys_ctx->metrics.low_beam_active = true;
    sys_ctx->metrics.low_beam_known = true;
    sys_ctx->metrics.high_beam_active = target_rpm > 4400.0f;
    sys_ctx->metrics.high_beam_known = true;
    // Interior lights: MEB has cabin light capability; PQ24 does not broadcast it
    sys_ctx->metrics.interior_lights_active = !is_pq24 && cabin_light;
    sys_ctx->metrics.interior_lights_known  = !is_pq24;
    // Gear/mode: PQ24 does not have gear-position CAN messages
    sys_ctx->metrics.sport_mode_active = !is_pq24 && sport_mode;
    sys_ctx->metrics.sport_mode_known  = !is_pq24;
    sys_ctx->metrics.gear_position = !is_pq24 ? (sport_mode ? GEAR_SELECTOR_SPORT : GEAR_SELECTOR_DRIVE)
                                               : GEAR_SELECTOR_UNKNOWN;
    sys_ctx->metrics.gear_position_known = !is_pq24;
    sys_ctx->metrics.selected_gear = !is_pq24 ? displayed_gear : 0;
    sys_ctx->metrics.selected_gear_known = !is_pq24;
    // Odometer: PQ24 doesn't broadcast odometer on CAN
    sys_ctx->metrics.odometer_km    = !is_pq24 ? odometer_km : 0.0f;
    sys_ctx->metrics.odometer_valid = !is_pq24;
    // Infotainment: PQ24 has no track metadata or phone
    sys_ctx->metrics.infotainment_source_code  = infotainment_source;
    sys_ctx->metrics.infotainment_source_known = true;
    sys_ctx->metrics.infotainment_track        = !is_pq24 ? infotainment_track : 0;
    sys_ctx->metrics.infotainment_track_known  = !is_pq24;
    sys_ctx->metrics.phone_call_active = false;
    sys_ctx->metrics.phone_call_known  = !is_pq24;
    sys_ctx->metrics.diagnostics_seen = true;
    sys_ctx->metrics.mil_active = false;
    sys_ctx->metrics.mil_status_known = true;
    sys_ctx->metrics.stored_dtc_count = 0;
    sys_ctx->metrics.control_module_voltage = 13.8f;
    sys_ctx->metrics.control_module_voltage_known = true;
    sys_ctx->metrics.last_diag_service = 0x41;
    sys_ctx->metrics.last_diag_pid = 0x42;
    sys_ctx->metrics.last_diag_source = 0x7E8;
    sys_ctx->metrics.diag_response_counter = bench_tick / 25;
    // MEB-specific EV fields
    sys_ctx->metrics.ev_soc_pct           = ev_soc_pct;
    sys_ctx->metrics.ev_soc_known         = is_meb;
    sys_ctx->metrics.ev_hv_voltage        = ev_hv_voltage;
    sys_ctx->metrics.ev_hv_voltage_known  = is_meb;
    sys_ctx->metrics.ev_charging_kw       = ev_charging_kw;
    sys_ctx->metrics.ev_charging_known    = is_meb;
    sys_ctx->metrics.ev_charging_active   = ev_charging_act;
    sys_ctx->metrics.ev_regen_torque      = ev_regen_torque;
    sys_ctx->metrics.ev_regen_known       = is_meb;
    sys_ctx->metrics.ev_motor_temp        = ev_motor_temp;
    sys_ctx->metrics.ev_motor_temp_known  = is_meb;
    sys_ctx->metrics.ev_cell_voltage_delta = is_meb ? 8.0f : 0.0f; // bench: 8 mV delta
    sys_ctx->metrics.ev_cell_delta_known  = is_meb;
    sys_ctx->metrics.ev_ota_update_active = false;
    sys_ctx->metrics.ev_ota_status_known  = is_meb;
    // Fuel level bench simulation
    sys_ctx->metrics.fuel_liters       = fuel_bench_liters;
    sys_ctx->metrics.fuel_percent      = has_fuel ? (fuel_bench_liters / 55.0f * 100.0f) : -1.0f;
    sys_ctx->metrics.fuel_level_known  = has_fuel;
    sys_ctx->metrics.fuel_timestamp_ms = has_fuel ? (uint32_t)millis() : 0;
    portEXIT_CRITICAL(&g_metrics_mux);

    // 2. Safely process hardware frame simulation ONLY if a real vehicle network is locked!
    // This stops the hardware registers from filling up and freezing the chip on your open desk.
    // C-5: Also guard on g_twai0_valid: the bus-off recovery on Core 1 temporarily uninstalls
    //      and reinstalls the port-0 driver.  Transmitting with a stale handle causes a crash.
    if (active_vehicle_profile.network_generation != SERIES_UNKNOWN &&
        g_twai0_valid.load(std::memory_order_acquire)) {
        twai_message_t tx_msg;
        tx_msg.extd = 0;
        tx_msg.rtr = 0;
        tx_msg.data_length_code = 8;

        if (active_vehicle_profile.network_generation == SERIES_MQB_EVO_MEB) {
            // === MEB ELECTRIC BENCH FRAMES ===

            // A. Motor speed (re-uses MQB 0x0FC frame ID)
            tx_msg.identifier = 0x0FC;
            uint16_t raw_motor = (uint16_t)(target_rpm / 0.25f);
            *(tx_msg.data + 0) = (uint8_t)(raw_motor & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_motor >> 8) & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // B. Thermal management (motor loop temp, inverter temp) — MEB 0x1A2
            tx_msg.identifier = 0x1A2;
            *(tx_msg.data + 0) = (uint8_t)(target_h2o + 40);          // coolant loop
            *(tx_msg.data + 1) = (uint8_t)(ev_motor_temp + 40.0f);    // motor temp
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // C. Vehicle speed — MEB 0x096 (same as MQB, 0.01 km/h/LSB)
            tx_msg.identifier = 0x096;
            uint16_t raw_spd_meb = (uint16_t)(vehicle_speed / 0.01f);
            *(tx_msg.data + 0) = (uint8_t)(raw_spd_meb & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_spd_meb >> 8) & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // D. Accelerator pedal — MEB 0x084 (same as MQB, 0.4 %/LSB)
            tx_msg.identifier = 0x084;
            *(tx_msg.data + 0) = (uint8_t)(throttle_pct / 0.4f);
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // E. HV Battery SoC — MEB 0x1C0 (big-endian, 0.1 % per LSB)
            tx_msg.identifier = 0x1C0;
            uint16_t raw_soc = (uint16_t)(ev_soc_pct * 10.0f);
            *(tx_msg.data + 0) = (uint8_t)((raw_soc >> 8) & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)(raw_soc & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // F. HV Pack Voltage — MEB 0x1C1 (big-endian, 0.1 V per LSB)
            tx_msg.identifier = 0x1C1;
            uint16_t raw_hv = (uint16_t)(ev_hv_voltage * 10.0f);
            *(tx_msg.data + 0) = (uint8_t)((raw_hv >> 8) & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)(raw_hv & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // G. Cell voltage delta — MEB 0x1C2
            tx_msg.identifier = 0x1C2;
            *(tx_msg.data + 0) = 8; // 8 mV bench delta
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // H. Charging power — MEB 0x29E (big-endian, 0.1 kW per LSB, byte 2 = active flag)
            tx_msg.identifier = 0x29E;
            uint16_t raw_chg = (uint16_t)(ev_charging_kw * 10.0f);
            *(tx_msg.data + 0) = (uint8_t)((raw_chg >> 8) & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)(raw_chg & 0xFF);
            *(tx_msg.data + 2) = ev_charging_act ? 0x01 : 0x00;
            for(int i = 3; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // I. Regen torque — MEB 0x1A4 (big-endian signed, 0.25 Nm per LSB)
            tx_msg.identifier = 0x1A4;
            int16_t raw_regen = (int16_t)(ev_regen_torque / 0.25f);
            *(tx_msg.data + 0) = (uint8_t)((raw_regen >> 8) & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)(raw_regen & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // J. Gear selector / drive mode — MEB 0x540
            tx_msg.identifier = 0x540;
            *(tx_msg.data + 0) = (uint8_t)((displayed_gear << 4) | (sport_mode ? 0x04 : 0x03));
            *(tx_msg.data + 1) = sport_mode ? 0x01 : 0x00;
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // K. Odometer — MEB 0x5A0 (0.1 km units, big-endian)
            tx_msg.identifier = 0x5A0;
            uint32_t raw_odo_meb = (uint32_t)(odometer_km * 10.0f);
            *(tx_msg.data + 0) = (uint8_t)((raw_odo_meb >> 24) & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_odo_meb >> 16) & 0xFF);
            *(tx_msg.data + 2) = (uint8_t)((raw_odo_meb >> 8) & 0xFF);
            *(tx_msg.data + 3) = (uint8_t)(raw_odo_meb & 0xFF);
            for(int i = 4; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // L. Exterior temp — MEB 0x317 (raw = °C + 40)
            tx_msg.identifier = 0x317;
            *(tx_msg.data + 0) = (uint8_t)(exterior_temp + 40.0f);
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);
        }
        else if (active_vehicle_profile.network_generation == SERIES_PQ24_PL45) {
            // === PQ24 EARLY GENERATION BENCH FRAMES ===
            // Only transmit the basic mechanical signals that PQ24 actually broadcasts.

            // A. Engine Speed — PQ24 0x280 (little-endian, 0.25 RPM/LSB)
            tx_msg.identifier = 0x280;
            uint16_t raw_rpm_pq24 = (uint16_t)(target_rpm / 0.25f);
            *(tx_msg.data + 0) = (uint8_t)(raw_rpm_pq24 & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_rpm_pq24 >> 8) & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // B. Thermal — PQ24 0x288 (coolant byte 0, oil byte 1, raw = °C + 40)
            tx_msg.identifier = 0x288;
            *(tx_msg.data + 0) = (uint8_t)(target_h2o + 40);
            *(tx_msg.data + 1) = (uint8_t)(target_oil + 40);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // C. MAP / Boost — PQ24 0x380 (raw byte × 10 = absolute mbar)
            tx_msg.identifier = 0x380;
            int abs_mbar_pq24 = (int)((target_boost * 1000.0f) + 1013.0f);
            int raw_map_pq24  = abs_mbar_pq24 / 10;
            *(tx_msg.data + 0) = (uint8_t)(raw_map_pq24 > 255 ? 255 : raw_map_pq24);
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // D. Vehicle Speed — PQ24 0x0C0 (0.01 km/h/LSB, little-endian)
            tx_msg.identifier = 0x0C0;
            uint16_t raw_spd_pq24 = (uint16_t)(vehicle_speed / 0.01f);
            *(tx_msg.data + 0) = (uint8_t)(raw_spd_pq24 & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_spd_pq24 >> 8) & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // E. Throttle — PQ24 0x088 (0.4 %/LSB)
            tx_msg.identifier = 0x088;
            *(tx_msg.data + 0) = (uint8_t)(throttle_pct / 0.4f);
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // NOTE: No 0x540 gear frame, no 0x5A0 odometer frame on PQ24.
            // NOTE: No 0x65D exterior temp on all PQ24 variants; omit from bench.
        }
        else if (active_vehicle_profile.network_generation == SERIES_MQB_A_CLASS) {
            // A. Pack Engine Speed to MQB Bus standard (ID: 0x0FC)
            tx_msg.identifier = 0x0FC;
            uint16_t raw_rpm = (uint16_t)(target_rpm / 0.25);
            *(tx_msg.data + 0) = (uint8_t)(raw_rpm & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_rpm >> 8) & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0); // Set timeout to 0 (NON-BLOCKING)

            // B. Pack Thermal Statistics to MQB Bus standard (ID: 0x1A2)
            tx_msg.identifier = 0x1A2;
            *(tx_msg.data + 0) = (uint8_t)(target_oil + 40);
            *(tx_msg.data + 1) = (uint8_t)(target_h2o + 40);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // C. Pack Boost Pressure to MQB Bus standard (ID: 0x28A)
            tx_msg.identifier = 0x28A;
            int absolute_mbar = (int)((target_boost * 1000.0) + 1013.0);
            uint16_t raw_mbar = (uint16_t)(absolute_mbar / 10);
            *(tx_msg.data + 0) = (uint8_t)(raw_mbar & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_mbar >> 8) & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // D. Pack Vehicle Speed to MQB standard (ID: 0x096, 0.01 km/h/LSB)
            tx_msg.identifier = 0x096;
            uint16_t raw_spd_mqb = (uint16_t)(vehicle_speed / 0.01f);
            *(tx_msg.data + 0) = (uint8_t)(raw_spd_mqb & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_spd_mqb >> 8) & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // E. Pack Throttle to MQB standard (ID: 0x084, 0.4 %/LSB)
            tx_msg.identifier = 0x084;
            *(tx_msg.data + 0) = (uint8_t)(throttle_pct / 0.4f);
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // F. Pack Exterior Temp to MQB standard (ID: 0x317, raw = °C + 40)
            tx_msg.identifier = 0x317;
            *(tx_msg.data + 0) = (uint8_t)(exterior_temp + 40.0f);
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // G. Pack representative gearbox and drive mode state (ID: 0x540)
            tx_msg.identifier = 0x540;
            *(tx_msg.data + 0) = (uint8_t)((displayed_gear << 4) | (sport_mode ? 0x04 : 0x03));
            *(tx_msg.data + 1) = sport_mode ? 0x01 : 0x00;
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // H. Pack representative odometer value (ID: 0x5A0, 0.1 km units)
            tx_msg.identifier = 0x5A0;
            uint32_t raw_odo = (uint32_t)(odometer_km * 10.0f);
            *(tx_msg.data + 0) = (uint8_t)((raw_odo >> 24) & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_odo >> 16) & 0xFF);
            *(tx_msg.data + 2) = (uint8_t)((raw_odo >> 8) & 0xFF);
            *(tx_msg.data + 3) = (uint8_t)(raw_odo & 0xFF);
            for(int i = 4; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);
        }
        else {
            // A. Pack Engine Speed to PQ Bus standard (ID: 0x280)
            tx_msg.identifier = 0x280;
            uint16_t raw_rpm = (uint16_t)(target_rpm / 0.25);
            *(tx_msg.data + 0) = (uint8_t)(raw_rpm & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_rpm >> 8) & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // B. Pack Thermal Channels to PQ Bus standard (ID: 0x288)
            tx_msg.identifier = 0x288;
            *(tx_msg.data + 0) = (uint8_t)(target_oil + 40);
            *(tx_msg.data + 1) = (uint8_t)(target_h2o + 40);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // C. Pack Boost Pressure to PQ Bus standard (ID: 0x380)
            // M-2: Clamp to uint8_t range to prevent overflow above ~1.4 Bar.
            tx_msg.identifier = 0x380;
            int absolute_mbar = (int)((target_boost * 1000.0) + 1013.0);
            int raw_mbar_clamped = absolute_mbar / 10;
            *(tx_msg.data + 0) = (uint8_t)(raw_mbar_clamped > 255 ? 255 : raw_mbar_clamped);
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // D. Pack Vehicle Speed to PQ standard (ID: 0x0C0, 0.01 km/h/LSB)
            tx_msg.identifier = 0x0C0;
            uint16_t raw_spd_pq = (uint16_t)(vehicle_speed / 0.01f);
            *(tx_msg.data + 0) = (uint8_t)(raw_spd_pq & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_spd_pq >> 8) & 0xFF);
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // E. Pack Throttle to PQ standard (ID: 0x088, 0.4 %/LSB)
            tx_msg.identifier = 0x088;
            *(tx_msg.data + 0) = (uint8_t)(throttle_pct / 0.4f);
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // F. Pack Exterior Temp to PQ standard (ID: 0x65D, raw = °C + 40)
            tx_msg.identifier = 0x65D;
            *(tx_msg.data + 0) = (uint8_t)(exterior_temp + 40.0f);
            for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // G. Pack representative gearbox and drive mode state (ID: 0x540)
            tx_msg.identifier = 0x540;
            *(tx_msg.data + 0) = (uint8_t)((displayed_gear << 4) | (sport_mode ? 0x04 : 0x03));
            *(tx_msg.data + 1) = sport_mode ? 0x01 : 0x00;
            for(int i = 2; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);

            // H. Pack representative odometer value (ID: 0x5A0, 0.1 km units)
            tx_msg.identifier = 0x5A0;
            uint32_t raw_odo = (uint32_t)(odometer_km * 10.0f);
            *(tx_msg.data + 0) = (uint8_t)((raw_odo >> 24) & 0xFF);
            *(tx_msg.data + 1) = (uint8_t)((raw_odo >> 16) & 0xFF);
            *(tx_msg.data + 2) = (uint8_t)((raw_odo >> 8) & 0xFF);
            *(tx_msg.data + 3) = (uint8_t)(raw_odo & 0xFF);
            for(int i = 4; i < 8; i++) *(tx_msg.data + i) = 0x00;
            twai_transmit_v2(*(twai_ports + 0), &tx_msg, 0);
        }

        // I. Pack representative comfort and infotainment states on shared buses
        tx_msg.identifier = 0x470;
        *(tx_msg.data + 0) = (left_indicator ? 0x01 : 0x00) |
                             (right_indicator ? 0x02 : 0x00) |
                             0x04 |
                             0x08 |
                             ((target_rpm > 4400.0f) ? 0x10 : 0x00) |
                             (cabin_light ? 0x20 : 0x00);
        for(int i = 1; i < 8; i++) *(tx_msg.data + i) = 0x00;
        twai_transmit_v2(*(twai_ports + 1), &tx_msg, 0);

        tx_msg.identifier = 0x6C1;
        *(tx_msg.data + 0) = infotainment_source;
        *(tx_msg.data + 1) = infotainment_track;
        *(tx_msg.data + 2) = 0x00;
        for(int i = 3; i < 8; i++) *(tx_msg.data + i) = 0x00;
        twai_transmit_v2(*(twai_ports + 2), &tx_msg, 0);
    }
}
