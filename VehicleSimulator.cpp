#include "VehicleSimulator.h"

// =========================================================================
//  CLASS METHOD DEFINITIONS: PLATFORM-AWARE TELEMETRY SIMULATOR CORE
// =========================================================================

void runBenchTelemetrySimulation(float target_rpm, float target_boost, float target_oil, float target_h2o) {
    if (sys_ctx == nullptr) return;
    static uint32_t bench_tick = 0;
    bench_tick++;

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
    const float odometer_km = 182345.6f + ((bench_tick % 500000UL) * 0.02f);
    const uint8_t infotainment_track = 7 + (bench_tick / 120) % 6;

    // C-4: Protect the metrics write with the spinlock so Core 1's
    //      updateUIElements/parsers never observe a partially-written struct.
    portENTER_CRITICAL(&g_metrics_mux);
    sys_ctx->metrics.engine_rpm   = target_rpm;
    sys_ctx->metrics.boost_bar    = target_boost;
    sys_ctx->metrics.oil_temp     = target_oil;
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
    sys_ctx->metrics.interior_lights_active = cabin_light;
    sys_ctx->metrics.interior_lights_known = true;
    sys_ctx->metrics.sport_mode_active = sport_mode;
    sys_ctx->metrics.sport_mode_known = true;
    sys_ctx->metrics.gear_position = sport_mode ? GEAR_SELECTOR_SPORT : GEAR_SELECTOR_DRIVE;
    sys_ctx->metrics.gear_position_known = true;
    sys_ctx->metrics.selected_gear = displayed_gear;
    sys_ctx->metrics.selected_gear_known = true;
    sys_ctx->metrics.odometer_km = odometer_km;
    sys_ctx->metrics.odometer_valid = true;
    sys_ctx->metrics.infotainment_source_code = infotainment_source;
    sys_ctx->metrics.infotainment_source_known = true;
    sys_ctx->metrics.infotainment_track = infotainment_track;
    sys_ctx->metrics.infotainment_track_known = true;
    sys_ctx->metrics.phone_call_active = false;
    sys_ctx->metrics.phone_call_known = true;
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

        if (active_vehicle_profile.network_generation == SERIES_MQB_A_CLASS) {
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
