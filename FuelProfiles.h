#ifndef FUEL_PROFILES_H
#define FUEL_PROFILES_H

// =============================================================================
//  FUEL LEVEL PROFILE REGISTRY
// =============================================================================
//
//  This file defines per-platform passive CAN and optional UDS endpoint
//  configurations for fuel level decoding.
//
//  IMPORTANT — READ BEFORE MODIFYING:
//  CAN IDs, DID values and byte layouts used here are derived from community
//  reverse-engineering of VAG platforms.  They are NOT guaranteed to be correct
//  for every model-year, cluster variant or regional market.  Each entry is
//  therefore gated behind a specific platform series so that an incorrect
//  mapping for platform A can never corrupt readings on platform B.
//
//  Confidence ratings:
//    HIGH   – Observed on multiple confirmed real vehicles; formula verified.
//    MEDIUM – Plausible from community data; treat as experimental.
//    LOW    – Unconfirmed; compiled from a single report.
//
//  For UDS entries the DID 0x2203 referenced in user documentation maps to
//  "Instrument Cluster fuel level" on some MQB/MLB clusters, but the exact
//  CAN routing ID (request 0x714, response 0x77E or 0x773) varies by year and
//  cluster software revision.  These entries are therefore MEDIUM confidence
//  and should be validated against your specific vehicle before relying on them.
//
// =============================================================================

#include "VehicleInterpreters.h"

// Maximum plausible fuel jump per update without being a refuel event (litres).
// A jump larger than this in the decreasing direction is silently rejected
// to guard against sensor noise or corrupt frames.
static constexpr float FUEL_PLAUSIBILITY_MAX_DROP_L  = 5.0f;
// A jump larger than this in the increasing direction is treated as a refuel.
static constexpr float FUEL_PLAUSIBILITY_REFUEL_L    = 8.0f;
// After this many milliseconds without a fuel update, mark the reading stale.
static constexpr uint32_t FUEL_STALE_TIMEOUT_MS = 10000;

// --- Passive CAN fuel frame definition ---
// Describes one CAN frame that carries a fuel-level byte.
struct FuelPassiveConfig {
    uint32_t    frame_id;       // Standard 11-bit CAN identifier
    uint8_t     byte_index;     // Byte position within the 8-byte data field (0-based)
    float       scale;          // Multiply raw byte value → litres
    float       offset;         // Add after scaling (litres)
    uint8_t     can_channel;    // 0 = drivetrain, 1 = comfort, 2 = infotainment
    bool        has_percent;    // True if a separate byte carries tank-percent
    uint8_t     pct_byte_index; // Byte index for percent (ignored if has_percent=false)
    float       pct_scale;      // Multiply raw percent byte → 0-100 %
    const char* note;           // Human-readable confidence note (PROGMEM-safe literal)
};

// --- UDS (diagnostic) fuel endpoint ---
// Describes an active UDS ReadDataByIdentifier request against the Instrument
// Cluster (address 0x17 in VAG addressing, physical CAN ID varies by platform).
// Set req_id = 0 to disable UDS for this platform.
struct FuelUdsConfig {
    uint32_t    req_id;         // Physical request CAN ID (e.g. 0x714)
    uint32_t    resp_id;        // Expected response CAN ID (e.g. 0x77E)
    uint16_t    did;            // ReadDataByIdentifier DID (e.g. 0x2203)
    uint8_t     resp_byte;      // Byte offset in positive response (after 0x62 hdr)
    float       scale;          // Multiply raw → litres
    float       offset;
    uint32_t    poll_interval_ms; // Minimum time between requests (ms)
    const char* note;
};

// --- Compound per-platform fuel profile ---
struct FuelPlatformProfile {
    MqbPlatformSeries   series;
    bool                passive_enabled;
    FuelPassiveConfig   passive[2];   // Up to 2 passive frame sources (first wins)
    uint8_t             passive_count;
    bool                uds_enabled;
    FuelUdsConfig       uds;
    const char*         platform_name;
};

// =============================================================================
//  PLATFORM PROFILES (one entry per MqbPlatformSeries value)
// =============================================================================

// MQB (Golf Mk7/Mk8, Audi A3 8V/8Y, Seat Leon 5F/KL, Skoda Octavia 5E, etc.)
// Passive: ID 0x12F, byte 4, scale 0.5 → litres.  Confidence: MEDIUM.
//   Field confirmed on Mk7 Golf and A3 8V by multiple community reports but
//   byte index and scale can differ on some cluster variants.
// UDS: DID 0x2203 via physical ID 0x714/0x77E.  Confidence: MEDIUM.
//   0x714 is the ISO-TP physical address for Instrument Cluster on most MQB
//   vehicles built 2013-2023.  The response ID 0x77E is the functional reply
//   from address 0x17.  DID 0x2203 is documented as "KI_Fuel_Level_Litre" in
//   some OEM cluster software revisions but may map differently on others.
static const FuelPlatformProfile kFuelProfileMqb = {
    SERIES_MQB_A_CLASS,
    /* passive_enabled */ true,
    {
        // Primary passive source
        {
            /* frame_id   */ 0x12F,
            /* byte_index */ 4,
            /* scale      */ 0.5f,
            /* offset     */ 0.0f,
            /* channel    */ 0,
            /* has_pct    */ false,
            /* pct_byte   */ 0,
            /* pct_scale  */ 0.0f,
            /* note       */ "MQB 0x12F b4*0.5L [MEDIUM confidence]"
        },
        // Fallback passive source (same frame, alternative clusters put value here)
        {
            /* frame_id   */ 0x12F,
            /* byte_index */ 3,
            /* scale      */ 0.5f,
            /* offset     */ 0.0f,
            /* channel    */ 0,
            /* has_pct    */ false,
            /* pct_byte   */ 0,
            /* pct_scale  */ 0.0f,
            /* note       */ "MQB 0x12F b3*0.5L fallback [LOW confidence]"
        }
    },
    /* passive_count */ 1,  // Use only primary by default; raise to 2 to try fallback
    /* uds_enabled   */ true,
    {
        /* req_id    */ 0x714,
        /* resp_id   */ 0x77E,
        /* did       */ 0x2203,
        /* resp_byte */ 0,   // First data byte after 0x62 0x22 0x03 header
        /* scale     */ 1.0f,
        /* offset    */ 0.0f,
        /* poll_ms   */ 1000,
        /* note      */ "MQB cluster DID 0x2203 req=0x714 [MEDIUM confidence — validate per year]"
    },
    "MQB / MQB Evo"
};

// PQ35 / PQ46 (Golf Mk5/Mk6, Passat B6/B7, Tiguan 5N, Audi A3 8P, etc.)
// Passive: ID 0x2C0, byte 2, direct hex → litres.  Confidence: MEDIUM.
//   Also observed on 0x621 byte 2 on some PQ35 comfort-bus configurations.
// Note: 0x2C0 is the primary Instrument Cluster gauge broadcast on PQ35.
//   The exact byte position can vary: byte 2 appears on most Mk5/Mk6 Golf
//   but some Passat B6 variants use byte 3.  Second passive entry covers that.
static const FuelPlatformProfile kFuelProfilePq35 = {
    SERIES_PQ35_46_LEGACY,
    /* passive_enabled */ true,
    {
        {
            /* frame_id   */ 0x2C0,
            /* byte_index */ 2,
            /* scale      */ 1.0f,
            /* offset     */ 0.0f,
            /* channel    */ 1,   // Comfort bus
            /* has_pct    */ false,
            /* pct_byte   */ 0,
            /* pct_scale  */ 0.0f,
            /* note       */ "PQ35 0x2C0 b2 direct L [MEDIUM confidence]"
        },
        {
            /* frame_id   */ 0x621,
            /* byte_index */ 2,
            /* scale      */ 1.0f,
            /* offset     */ 0.0f,
            /* channel    */ 1,   // Comfort bus
            /* has_pct    */ false,
            /* pct_byte   */ 0,
            /* pct_scale  */ 0.0f,
            /* note       */ "PQ35 0x621 b2 direct L [MEDIUM confidence]"
        }
    },
    /* passive_count */ 2,
    /* uds_enabled   */ false,  // PQ35/46 cluster UDS routing not consistently documented
    {
        /* req_id    */ 0x688,  // PQ35 Instrument Cluster physical request ID
        /* resp_id   */ 0x6B8,  // PQ35 Instrument Cluster response ID
        /* did       */ 0x2203,
        /* resp_byte */ 0,
        /* scale     */ 1.0f,
        /* offset    */ 0.0f,
        /* poll_ms   */ 1000,
        /* note      */ "PQ35 cluster DID 0x2203 [LOW confidence — unconfirmed]"
    },
    "PQ35 / PQ46"
};

// PQ24 / PL45 (Golf Mk4, Bora, Polo 9N, A3 8L, A4 B5/B6, early Fabia)
// Limited CAN documentation; no confirmed fuel broadcast ID.
// UDS not enabled — cluster on these platforms does not reliably respond to
// modern UDS ReadDataByIdentifier.
static const FuelPlatformProfile kFuelProfilePq24 = {
    SERIES_PQ24_PL45,
    /* passive_enabled */ false,
    {
        { 0x000, 0, 0.0f, 0.0f, 0, false, 0, 0.0f, "PQ24 passive fuel: not documented" },
        { 0x000, 0, 0.0f, 0.0f, 0, false, 0, 0.0f, "" }
    },
    /* passive_count */ 0,
    /* uds_enabled   */ false,
    { 0, 0, 0, 0, 0.0f, 0.0f, 0, "PQ24 UDS fuel: not supported" },
    "PQ24 / PL45"
};

// Small/Compact (MQB A0: Polo 6C/AW, Arona, Ibiza 6J/KJ, Fabia 3V, A1 GB)
// MQB A0 shares MQB frame IDs for most signals; 0x12F is confirmed on Polo AW.
static const FuelPlatformProfile kFuelProfileCompact = {
    SERIES_SMALL_PO_SKODA,
    /* passive_enabled */ true,
    {
        {
            /* frame_id   */ 0x12F,
            /* byte_index */ 4,
            /* scale      */ 0.5f,
            /* offset     */ 0.0f,
            /* channel    */ 0,
            /* has_pct    */ false,
            /* pct_byte   */ 0,
            /* pct_scale  */ 0.0f,
            /* note       */ "MQB-A0 0x12F b4*0.5L [MEDIUM confidence]"
        },
        { 0x000, 0, 0.0f, 0.0f, 0, false, 0, 0.0f, "" }
    },
    /* passive_count */ 1,
    /* uds_enabled   */ true,
    {
        /* req_id    */ 0x714,
        /* resp_id   */ 0x77E,
        /* did       */ 0x2203,
        /* resp_byte */ 0,
        /* scale     */ 1.0f,
        /* offset    */ 0.0f,
        /* poll_ms   */ 1000,
        /* note      */ "MQB-A0 cluster DID 0x2203 [MEDIUM confidence]"
    },
    "Small/Compact (MQB A0, PQ25)"
};

// MLB Longitudinal (Audi A4 B8/B9, A5, A6 C7/C8, Q5 80A, Q7 4M, Porsche Macan)
// MLB shares some MQB signal IDs; 0x12F b4 is plausible but unconfirmed on MLB.
// UDS request ID 0x714 is used on some MLB variants; treat as LOW confidence.
static const FuelPlatformProfile kFuelProfileMlb = {
    SERIES_MLB_LONG_CLASS,
    /* passive_enabled */ true,
    {
        {
            /* frame_id   */ 0x12F,
            /* byte_index */ 4,
            /* scale      */ 0.5f,
            /* offset     */ 0.0f,
            /* channel    */ 0,
            /* has_pct    */ false,
            /* pct_byte   */ 0,
            /* pct_scale  */ 0.0f,
            /* note       */ "MLB 0x12F b4*0.5L [LOW confidence — validate per model]"
        },
        { 0x000, 0, 0.0f, 0.0f, 0, false, 0, 0.0f, "" }
    },
    /* passive_count */ 1,
    /* uds_enabled   */ true,
    {
        /* req_id    */ 0x714,
        /* resp_id   */ 0x77E,
        /* did       */ 0x2203,
        /* resp_byte */ 0,
        /* scale     */ 1.0f,
        /* offset    */ 0.0f,
        /* poll_ms   */ 1000,
        /* note      */ "MLB cluster DID 0x2203 [LOW confidence — validate per model/year]"
    },
    "MLB Longitudinal"
};

// MEB Electric (ID.3, ID.4, ID.Buzz, Q4 e-tron, Enyaq, Cupra Born)
// No liquid fuel tank.  Fuel-level fields are always reported as unavailable.
// Battery SoC is handled separately via the EV fields (ev_soc_pct).
static const FuelPlatformProfile kFuelProfileMeb = {
    SERIES_MQB_EVO_MEB,
    /* passive_enabled */ false,
    {
        { 0x000, 0, 0.0f, 0.0f, 0, false, 0, 0.0f, "MEB: no fuel tank" },
        { 0x000, 0, 0.0f, 0.0f, 0, false, 0, 0.0f, "" }
    },
    /* passive_count */ 0,
    /* uds_enabled   */ false,
    { 0, 0, 0, 0, 0.0f, 0.0f, 0, "MEB: no fuel tank" },
    "MEB Electric"
};

// =============================================================================
//  LOOKUP HELPER
// =============================================================================
// Returns the fuel profile for the given platform series.
// Returns nullptr for SERIES_UNKNOWN.
static inline const FuelPlatformProfile* getFuelProfile(MqbPlatformSeries gen) {
    switch (gen) {
        case SERIES_MQB_A_CLASS:     return &kFuelProfileMqb;
        case SERIES_PQ35_46_LEGACY:  return &kFuelProfilePq35;
        case SERIES_PQ24_PL45:       return &kFuelProfilePq24;
        case SERIES_SMALL_PO_SKODA:  return &kFuelProfileCompact;
        case SERIES_MLB_LONG_CLASS:  return &kFuelProfileMlb;
        case SERIES_MQB_EVO_MEB:     return &kFuelProfileMeb;
        default:                     return nullptr;
    }
}

// =============================================================================
//  PLAUSIBILITY FILTER
// =============================================================================
// Call this instead of writing fuel_liters directly. Returns true if the new
// value was accepted.  Handles refuel events by allowing large jumps upward.
//
// Parameters:
//   ctx        – pointer to the shared GlobalFrameworkContext
//   new_liters – decoded litres from frame or UDS response
//   max_tank_l – maximum physical tank capacity for this vehicle (litres).
//                Pass 0.0f if unknown (disables upper-bound check).
static inline bool applyFuelPlausibility(GlobalFrameworkContext* ctx,
                                         float new_liters,
                                         float max_tank_l = 0.0f) {
    if (ctx == nullptr) return false;
    if (new_liters < 0.0f) return false;
    if (max_tank_l > 0.0f && new_liters > max_tank_l) return false;

    const float prev = ctx->metrics.fuel_liters;
    const bool had_value = ctx->metrics.fuel_level_known;

    if (had_value && prev >= 0.0f) {
        const float delta = new_liters - prev;
        // Reject implausible drops (sensor noise or corrupt frame)
        if (delta < -FUEL_PLAUSIBILITY_MAX_DROP_L) return false;
    }

    ctx->metrics.fuel_liters      = new_liters;
    ctx->metrics.fuel_level_known = true;
    ctx->metrics.fuel_timestamp_ms = (uint32_t)millis();
    return true;
}

#endif // FUEL_PROFILES_H
