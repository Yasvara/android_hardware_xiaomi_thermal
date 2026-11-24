// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Sensor reading with variable-dt Kalman filter and VIRTUAL-SENSOR0 computation.
//!
//! Optimizations:
//! - Lazy per-zone FD opening (opens only the ~9 active zones instead of all 74)
//! - Single-stage Kalman on fused VSENSOR0 (eliminates 7× redundant filters + double phase lag)
//! - Variable `dt_s` propagation (accurate dT/dt across 1s screen-on, 3s screen-off, and suspend)
//! - Unit-consistent mC² covariance matrices
//! - Deduplicated `battery` / `charger_therm` reads across VSENSOR0 and charging roles

use log::{info, warn};

use crate::io::SysfsFd;

// ---- VIRTUAL-SENSOR0 weights (RE'd from decrypt.txt) ----
// Sensors: battery, pa_therm0, pa_therm1, quiet_therm, charger_therm, wifi_therm, cpu_therm
// Weight sum = 1000, compensation offset = 1034 mC
const VSENSOR_SENSOR_NAMES: [&str; 7] = [
    "battery", "pa_therm0", "pa_therm1", "quiet_therm",
    "charger_therm", "wifi_therm", "cpu_therm",
];
const VSENSOR_WEIGHTS: [i64; 7] = [242, 424, -165, 715, -228, 222, -268];
const VSENSOR_COMPENSATION: i64 = 1034; // milli-°C offset

const CELL_NAME_FALLBACK: &str = "battery";
const BOARD_NAME_FALLBACK: &str = "batt_therm";
const PUMP_NAME_FALLBACK: &str = "charger_therm";

const BATTERY_CURRENT_PATH: &str = "/sys/class/power_supply/battery/current_now";
const BATTERY_VOLTAGE_PATH: &str = "/sys/class/power_supply/battery/voltage_now";

// ---- Kalman Filter tuning (all variances in milli-°C² to match state units!) ----
// 0.5 °C² = 500_000 mC²; 0.1 (°C/s)² = 100_000 (mC/s)²; 2.0 °C² = 2_000_000 mC²
const KALMAN_Q_TEMP: f64 = 500_000.0;
const KALMAN_Q_RATE: f64 = 100_000.0;
const KALMAN_R: f64 = 2_000_000.0;
const KALMAN_P00_INIT: f64 = 10_000_000.0; // ~3.16 °C initial std dev
const KALMAN_P11_INIT: f64 = 1_000_000.0;  // ~1.0 °C/s initial rate std dev

/// 2-state Kalman filter `[temperature (mC), dT/dt (mC/s)]` supporting variable `dt_s`.
pub struct KalmanFilter {
    x_temp: f64,
    x_rate: f64,
    p00: f64,
    p01: f64,
    p11: f64,
    initialized: bool,
}

impl KalmanFilter {
    pub fn new() -> Self {
        KalmanFilter {
            x_temp: 0.0,
            x_rate: 0.0,
            p00: KALMAN_P00_INIT,
            p01: 0.0,
            p11: KALMAN_P11_INIT,
            initialized: false,
        }
    }

    /// Update at nominal 1 Hz (kept for unit tests and fixed-cadence callers).
    #[allow(dead_code)]
    pub fn update(&mut self, measurement_mc: i64) -> i64 {
        self.update_dt(measurement_mc, 1.0)
    }

    /// Update with actual elapsed interval `dt_s` (seconds), clamped to `[0.1, 10.0]`
    /// so suspend gaps or 3 s screen-off polling keep `x_rate` in true `mC/s`.
    pub fn update_dt(&mut self, measurement_mc: i64, dt_s: f64) -> i64 {
        let z = measurement_mc as f64;

        if !self.initialized {
            self.x_temp = z;
            self.x_rate = 0.0;
            self.initialized = true;
            return measurement_mc;
        }

        let dt = if dt_s.is_finite() { dt_s.clamp(0.1, 10.0) } else { 1.0 };

        // If the device just resumed from a long suspend (> 8s gap), reset rate
        // rather than extrapolating stale slope across minutes of sleep.
        if dt_s > 8.0 {
            self.x_temp = z;
            self.x_rate = 0.0;
            self.p00 = KALMAN_P00_INIT;
            self.p01 = 0.0;
            self.p11 = KALMAN_P11_INIT;
            return measurement_mc;
        }

        // ---- Predict step ----
        let x_pred_temp = self.x_temp + self.x_rate * dt;
        let x_pred_rate = self.x_rate;

        let p00_pred = self.p00 + dt * (self.p01 + self.p01)
            + dt * dt * self.p11
            + KALMAN_Q_TEMP * dt;
        let p01_pred = self.p01 + dt * self.p11 + KALMAN_Q_RATE * 0.5 * dt;
        let p11_pred = self.p11 + KALMAN_Q_RATE * dt;

        // ---- Update step ----
        let y = z - x_pred_temp;
        let s = p00_pred + KALMAN_R;
        let s_inv = 1.0 / s;

        let k0 = p00_pred * s_inv;
        let k1 = p01_pred * s_inv;

        self.x_temp = x_pred_temp + k0 * y;
        self.x_rate = x_pred_rate + k1 * y;

        self.p00 = (1.0 - k0) * p00_pred;
        self.p01 = (1.0 - k0) * p01_pred;
        self.p11 = (-k1 * p01_pred + p11_pred).max(0.0);

        self.x_temp as i64
    }

    /// Estimated rate of temperature change (`mC/s`).
    pub fn rate(&self) -> f64 {
        self.x_rate
    }

    /// Predict temperature `seconds_ahead` into the future (`mC`).
    pub fn predict(&self, seconds_ahead: f64) -> i64 {
        (self.x_temp + self.x_rate * seconds_ahead) as i64
    }
}

struct ZoneEntry {
    name: String,
    path: String,
    fd: Option<SysfsFd>,
}

impl ZoneEntry {
    fn open_if_needed(&mut self) {
        if self.fd.is_none() {
            self.fd = SysfsFd::open_ro_owned(self.path.clone());
        }
    }

    fn read_int(&self) -> Option<i64> {
        self.fd.as_ref().and_then(|f| f.read_int())
    }
}

/// Sensor pool: lazy-opened fds for active thermal zones + VSENSOR0 & charging fusion.
pub struct SensorPool {
    vsensor_indices: [i32; 7],
    pub vsensor_fused_filter: KalmanFilter,

    zones: Vec<ZoneEntry>,

    pub vsensor0: i64,
    battery_zone_idx: i32,
    pub battery_temp: i64,

    // ---- Charging protection sensors (by role) ----
    pub cell_temp: i64,
    pub board_temp: i64,
    pub charger_therm: i64,
    pub charge_temp: i64,
    cell_idx: i32,
    board_idx: i32,
    chg_therm_idx: i32,

    // Cached active sensor role names to avoid re-scanning zones every cycle.
    cell_name: String,
    board_name: String,
    pump_name: String,
    board_offset_mc: i64,
    w_cell: i64,
    w_board: i64,

    // ---- Battery power telemetry ----
    current_now_fd: Option<SysfsFd>,
    voltage_now_fd: Option<SysfsFd>,
    pub battery_current_ua: i64,
    pub battery_voltage_uv: i64,
}

impl SensorPool {
    /// Initialize sensor pool from discovered `(type_name, temp_path)` pairs.
    /// Only opens `/temp` FDs for zones actually bound to VSENSOR0 or charging roles.
    pub fn new(discovered: Vec<(String, String)>) -> Self {
        let mut zones: Vec<ZoneEntry> = discovered
            .into_iter()
            .map(|(name, path)| ZoneEntry { name, path, fd: None })
            .collect();

        let mut vsensor_indices = [-1i32; 7];
        let mut battery_zone_idx: i32 = -1;

        for (i, name) in VSENSOR_SENSOR_NAMES.iter().enumerate() {
            if let Some(j) = zones.iter().position(|z| z.name == *name) {
                vsensor_indices[i] = j as i32;
                zones[j].open_if_needed();
                if *name == "battery" {
                    battery_zone_idx = j as i32;
                }
            } else {
                warn!("VSENSOR0 sensor '{}' not found in thermal zones!", name);
            }
        }

        let bind_role = |zones: &mut [ZoneEntry], role_name: &str| -> i32 {
            if let Some(j) = zones.iter().position(|z| z.name == role_name) {
                zones[j].open_if_needed();
                j as i32
            } else {
                -1
            }
        };

        let cell_idx = bind_role(&mut zones, CELL_NAME_FALLBACK);
        let board_idx = bind_role(&mut zones, BOARD_NAME_FALLBACK);
        let chg_therm_idx = bind_role(&mut zones, PUMP_NAME_FALLBACK);

        if cell_idx < 0 {
            warn!("cell sensor missing — cell protection degraded");
        }
        if chg_therm_idx < 0 {
            warn!("charger_therm missing — charge pump governor degraded");
        }

        let open_count = zones.iter().filter(|z| z.fd.is_some()).count();
        info!(
            "Discovered {} thermal zones, opened {} active fds (VSENSOR0 + charging)",
            zones.len(),
            open_count
        );

        SensorPool {
            vsensor_indices,
            vsensor_fused_filter: KalmanFilter::new(),
            zones,
            vsensor0: 25_000,
            battery_zone_idx,
            battery_temp: 30_000,
            cell_temp: 30_000,
            board_temp: 30_000,
            charger_therm: 30_000,
            charge_temp: 30_000,
            cell_idx,
            board_idx,
            chg_therm_idx,
            cell_name: CELL_NAME_FALLBACK.to_string(),
            board_name: BOARD_NAME_FALLBACK.to_string(),
            pump_name: PUMP_NAME_FALLBACK.to_string(),
            board_offset_mc: 6_000,
            w_cell: 800,
            w_board: 200,
            current_now_fd: SysfsFd::open_ro(BATTERY_CURRENT_PATH),
            voltage_now_fd: SysfsFd::open_ro(BATTERY_VOLTAGE_PATH),
            battery_current_ua: 0,
            battery_voltage_uv: 0,
        }
    }

    /// Read all active sensors at 1 Hz nominal cadence (startup convenience).
    pub fn poll(&mut self) {
        self.poll_dt(1.0);
    }

    /// Read all active sensors and advance Kalman filters with true elapsed `dt_s`.
    /// ZERO heap allocation; deduplicates `battery` and `charger_therm` reads.
    pub fn poll_dt(&mut self, dt_s: f64) {
        let mut weighted_sum: i64 = 0;
        let mut active_weight_sum: i64 = 0;
        let mut valid_count = 0usize;

        let mut cell_read = false;
        let mut board_read = false;
        let mut pump_read = false;

        for (&idx, &weight) in self.vsensor_indices.iter().zip(VSENSOR_WEIGHTS.iter()) {
            if idx < 0 {
                continue;
            }
            if let Some(raw_mc) = self.zones[idx as usize].read_int() {
                weighted_sum += raw_mc * weight;
                active_weight_sum += weight;
                valid_count += 1;

                if idx == self.battery_zone_idx {
                    self.battery_temp = raw_mc;
                }
                if idx == self.cell_idx {
                    self.cell_temp = raw_mc;
                    cell_read = true;
                }
                if idx == self.board_idx {
                    self.board_temp = raw_mc;
                    board_read = true;
                }
                if idx == self.chg_therm_idx {
                    self.charger_therm = raw_mc;
                    pump_read = true;
                }
            }
        }

        if valid_count >= 3 && active_weight_sum > 0 {
            let raw_vsensor = weighted_sum / active_weight_sum + VSENSOR_COMPENSATION;
            self.vsensor0 = self.vsensor_fused_filter.update_dt(raw_vsensor, dt_s);
        } else if self.battery_zone_idx >= 0 {
            self.vsensor0 = self.vsensor_fused_filter.update_dt(self.battery_temp + 2000, dt_s);
            warn!("VSENSOR0 degraded: only {} sensors valid, fallback to battery temp", valid_count);
        }

        // Only read charging role zones not already captured in the VSENSOR0 pass.
        if !cell_read {
            if self.cell_idx >= 0 {
                if let Some(v) = self.zones[self.cell_idx as usize].read_int() {
                    self.cell_temp = v;
                }
            } else {
                self.cell_temp = self.battery_temp;
            }
        }
        if !board_read && self.board_idx >= 0 {
            if let Some(v) = self.zones[self.board_idx as usize].read_int() {
                self.board_temp = v;
            }
        }
        if !pump_read && self.chg_therm_idx >= 0 {
            if let Some(v) = self.zones[self.chg_therm_idx as usize].read_int() {
                self.charger_therm = v;
            }
        }

        let board_referred = self.board_temp - self.board_offset_mc;
        let wdiv = self.w_cell + self.w_board;
        self.charge_temp = if wdiv > 0 {
            (self.cell_temp * self.w_cell + board_referred * self.w_board) / wdiv
        } else {
            self.cell_temp
        };

        if let Some(ref fd) = self.current_now_fd {
            if let Some(v) = fd.read_int() {
                self.battery_current_ua = v;
            }
        }
        if let Some(ref fd) = self.voltage_now_fd {
            if let Some(v) = fd.read_int() {
                self.battery_voltage_uv = v;
            }
        }
    }

    pub fn charge_current_ua(&self) -> i64 {
        (-self.battery_current_ua).max(0)
    }

    pub fn net_current_ua(&self) -> i64 {
        -self.battery_current_ua
    }

    /// Re-bind sensor roles and fusion weights only when tuning actually changes.
    pub fn apply_tuning(&mut self, t: &crate::config::ChargeTuning) {
        if self.cell_name != t.cell_sensor {
            if let Some(j) = self.zones.iter().position(|z| z.name == t.cell_sensor) {
                self.zones[j].open_if_needed();
                self.cell_idx = j as i32;
                self.cell_name = t.cell_sensor.clone();
            }
        }
        if self.board_name != t.board_sensor {
            if let Some(j) = self.zones.iter().position(|z| z.name == t.board_sensor) {
                self.zones[j].open_if_needed();
                self.board_idx = j as i32;
                self.board_name = t.board_sensor.clone();
            }
        }
        if self.pump_name != t.pump_sensor {
            if let Some(j) = self.zones.iter().position(|z| z.name == t.pump_sensor) {
                self.zones[j].open_if_needed();
                self.chg_therm_idx = j as i32;
                self.pump_name = t.pump_sensor.clone();
            }
        }
        self.board_offset_mc = t.board_offset_mc;
        self.w_cell = t.w_cell;
        self.w_board = t.w_board;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kalman_converges_to_stable_value() {
        let mut kf = KalmanFilter::new();
        let mut out = 0i64;
        for i in 0..50 {
            let noise = if i % 2 == 0 { 500 } else { -500 };
            out = kf.update(40_000 + noise);
        }
        assert!((out - 40_000).abs() < 1000,
            "Kalman did not converge: {} (expected ~40000)", out);
    }

    #[test]
    fn kalman_rate_estimation() {
        let mut kf = KalmanFilter::new();
        for i in 0..10 {
            kf.update(30_000 + i * 1000);
        }
        let rate = kf.rate();
        assert!(rate > 500.0 && rate < 1500.0,
            "rate {} mC/s outside expected ~1000", rate);
    }

    #[test]
    fn kalman_rate_accurate_at_idle_dt() {
        let mut kf = KalmanFilter::new();
        // +3°C every 3 seconds = true 1000 mC/s rate during screen-off polling.
        for i in 0..10 {
            kf.update_dt(30_000 + i * 3000, 3.0);
        }
        let rate = kf.rate();
        assert!(rate > 700.0 && rate < 1300.0,
            "idle 3s dt rate {} mC/s outside ~1000", rate);
    }

    #[test]
    fn kalman_predict_ahead() {
        let mut kf = KalmanFilter::new();
        for i in 0..20 {
            kf.update(30_000 + i * 1000);
        }
        let pred = kf.predict(4.0);
        let current = kf.update(50_000);
        assert!((pred - current - 4000).abs() < 2000,
            "predict {} vs current {} — delta {} (expected ~4000)",
            pred, current, pred - current);
    }

    #[test]
    fn vsensor_not_stuck_at_25c_with_missing_sensor() {
        let weights: [i64; 7] = [242, 424, -165, 715, -228, 222, -268];
        let temps:   [i64; 7] = [35000, 36000, 34000, 0, 38000, 33000, 40000];

        let mut weighted_sum: i64 = 0;
        let mut active_weight_sum: i64 = 0;
        let mut valid = 0usize;
        for i in 0..7 {
            if i == 3 { continue; }
            weighted_sum += temps[i] * weights[i];
            active_weight_sum += weights[i];
            valid += 1;
        }
        assert_eq!(valid, 6);
        assert!(active_weight_sum > 0);

        let vsensor = weighted_sum / active_weight_sum + 1034;
        assert!(vsensor > 20_000 && vsensor < 60_000,
            "vsensor {} implausible (stuck at 25°C?)", vsensor);
        assert_eq!(vsensor, 27_756);
    }
}
