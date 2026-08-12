// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Runtime configuration via system properties.
//!
//! GameSpace ThermalController writes settings to system properties,
//! and this module reads them. Polled every N cycles to minimize overhead.
//! Pattern copied from AOSP `mmd` daemon (system/memory/mmd).

use log::info;
use rustutils::system_properties;

/// How often to re-read system properties (in polling cycles).
/// At 1Hz polling, 5 = re-read every 5 seconds.
const PROP_POLL_INTERVAL: u32 = 5;

// Property names — must match GameSpace ThermalProfiles.kt
const PROP_TEMP_LIMIT: &str = "persist.sys.mithermal.temp_limit";
const PROP_MAX_FCC: &str = "persist.sys.mithermal.max_fcc";
const PROP_MIN_FCC: &str = "persist.sys.mithermal.min_fcc";
const PROP_ENABLED: &str = "persist.sys.mithermal.enabled";
/// Aggressiveness knob: signed deci-°C added as thermal headroom. Positive =
/// tolerate more heat before throttling (the daemon subtracts it from VSENSOR0
/// before evaluating SS + monitor thresholds). Written by GameSpace/Settings.
const PROP_TEMP_OFFSET: &str = "persist.sys.mithermal.temp_offset";

/// Clamp for the aggressiveness offset (milli-°C). ±15°C keeps it sane.
const TEMP_OFFSET_MIN_MC: i64 = -15_000;
const TEMP_OFFSET_MAX_MC: i64 = 15_000;

/// Runtime configuration state.
pub struct Config {
    /// Battery temperature limit in milli-°C (e.g. 38000 = 38.0°C).
    /// GameSpace sends deci-°C (380), we convert to milli-°C.
    pub temp_limit_mc: i64,

    /// Maximum Fast Charge Current in µA (e.g. 15600000 = 15.6A).
    pub max_fcc_ua: i64,

    /// Minimum Fast Charge Current in µA (floor for SIC PID algo).
    pub min_fcc_ua: i64,

    /// Kill switch — if false, daemon runs but doesn't write to cooling devices.
    pub enabled: bool,

    /// Aggressiveness headroom in milli-°C (signed). Subtracted from VSENSOR0
    /// before SS + monitor threshold evaluation. Positive = less throttling.
    pub temp_offset_mc: i64,

    /// Internal counter for property polling interval.
    poll_counter: u32,
}

impl Config {
    /// Create default configuration.
    pub fn new() -> Self {
        let mut cfg = Config {
            temp_limit_mc: 38000,     // 38.0°C default
            max_fcc_ua: 20_000_000,   // 90W full hardware limit (20A)
            min_fcc_ua: 100_000,      // 100mA default
            enabled: true,
            temp_offset_mc: 0,
            poll_counter: 0,
        };
        // Read initial values
        cfg.force_reload();
        cfg
    }

    /// Check if it's time to re-read properties.
    /// Call this every polling cycle. Returns true if config changed.
    pub fn maybe_reload(&mut self) -> bool {
        self.poll_counter += 1;
        if self.poll_counter >= PROP_POLL_INTERVAL {
            self.poll_counter = 0;
            self.force_reload();
            true
        } else {
            false
        }
    }

    /// Force re-read all properties from system_properties.
    fn force_reload(&mut self) {
        let old_limit = self.temp_limit_mc;
        let old_max = self.max_fcc_ua;
        let old_min = self.min_fcc_ua;
        let old_enabled = self.enabled;
        let old_offset = self.temp_offset_mc;

        // temp_limit: GameSpace writes deci-°C (e.g. "380")
        // We convert to milli-°C (38000) for consistency with thermal_zone units
        if let Some(val) = read_prop_i64(PROP_TEMP_LIMIT) {
            self.temp_limit_mc = val * 100; // deci-°C → milli-°C
        }

        if let Some(val) = read_prop_i64(PROP_MAX_FCC) {
            self.max_fcc_ua = val;
        }

        if let Some(val) = read_prop_i64(PROP_MIN_FCC) {
            self.min_fcc_ua = val;
        }

        // Ensure min <= max
        if self.min_fcc_ua > self.max_fcc_ua {
            self.min_fcc_ua = self.max_fcc_ua;
        }

        if let Some(val) = read_prop_bool(PROP_ENABLED) {
            self.enabled = val;
        }

        // Aggressiveness: GameSpace/Settings write signed deci-°C; convert to
        // milli-°C and clamp to a sane band.
        if let Some(val) = read_prop_i64(PROP_TEMP_OFFSET) {
            self.temp_offset_mc = (val * 100).clamp(TEMP_OFFSET_MIN_MC, TEMP_OFFSET_MAX_MC);
        } else {
            self.temp_offset_mc = 0;
        }

        // Log if anything changed
        if self.temp_limit_mc != old_limit {
            info!("Config: temp_limit = {}°C", self.temp_limit_mc as f64 / 1000.0);
        }
        if self.max_fcc_ua != old_max {
            info!("Config: max_fcc = {} µA ({:.1}A)", self.max_fcc_ua, self.max_fcc_ua as f64 / 1_000_000.0);
        }
        if self.min_fcc_ua != old_min {
            info!("Config: min_fcc = {} µA ({:.1}A)", self.min_fcc_ua, self.min_fcc_ua as f64 / 1_000_000.0);
        }
        if self.enabled != old_enabled {
            info!("Config: enabled = {}", self.enabled);
        }
        if self.temp_offset_mc != old_offset {
            info!("Config: temp_offset = {:+.1}°C", self.temp_offset_mc as f64 / 1000.0);
        }
    }
}

/// Read a system property as i64.
fn read_prop_i64(prop: &str) -> Option<i64> {
    match system_properties::read(prop) {
        Ok(Some(val)) => val.parse::<i64>().ok(),
        _ => None,
    }
}

/// Read a system property as bool ("true"/"1" = true).
fn read_prop_bool(prop: &str) -> Option<bool> {
    match system_properties::read(prop) {
        Ok(Some(val)) => {
            let v = val.trim();
            Some(v == "true" || v == "1")
        }
        _ => None,
    }
}
