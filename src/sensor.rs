// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Sensor reading with Kalman filter and VIRTUAL-SENSOR0 computation.
//!
//! SOTA improvement over mi_thermald:
//! - Kalman filter per-sensor for noise rejection
//! - Built-in rate-of-change estimation (dT/dt) for predictive throttling
//! - VIRTUAL-SENSOR0 uses same weights as RE'd from mi_thermald config

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
const VSENSOR_WEIGHT_SUM: i64 = 1000; // Divisor — NOT sum of weights
const VSENSOR_COMPENSATION: i64 = 1034; // milli-°C offset

// ---- Battery fuel-gauge nodes (POCO F7) ----
// current_now: µA, NEGATIVE while charging (verified on onyx: -1066000 = 1.066A in).
// voltage_now: µV (verified: 4436000 = 4.436V). Both labelled vendor_sysfs_xm_power,
// which mi_thermal_engine.te already grants r_file_perms. Consumed by SIC feedforward.
const BATTERY_CURRENT_PATH: &str = "/sys/class/power_supply/battery/current_now";
const BATTERY_VOLTAGE_PATH: &str = "/sys/class/power_supply/battery/voltage_now";

// ---- Kalman Filter tuning ----
// Process noise (Q): how much we expect temp to change per cycle
// Measurement noise (R): how noisy the NTC readings are
const KALMAN_Q_TEMP: f64 = 0.5;   // °C² per second — low: trust model
const KALMAN_Q_RATE: f64 = 0.1;   // (°C/s)² — rate changes slowly
const KALMAN_R: f64 = 2.0;        // °C² — NTC noise (fairly noisy)
const KALMAN_DT: f64 = 1.0;       // seconds between polls

/// Kalman filter state for a single temperature signal.
/// State vector: [temperature, rate_of_change]
/// This gives us noise-filtered temp AND built-in derivative for prediction.
pub struct KalmanFilter {
    // State: [temp (mC), dtemp/dt (mC/s)]
    x_temp: f64,
    x_rate: f64,

    // Error covariance matrix (2x2, symmetric)
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
            p00: 100.0, // High initial uncertainty
            p01: 0.0,
            p11: 100.0,
            initialized: false,
        }
    }

    /// Update the filter with a new measurement (in milli-°C).
    /// Returns the filtered temperature in milli-°C.
    pub fn update(&mut self, measurement_mc: i64) -> i64 {
        let z = measurement_mc as f64;

        if !self.initialized {
            self.x_temp = z;
            self.x_rate = 0.0;
            self.initialized = true;
            return measurement_mc;
        }

        // ---- Predict step ----
        // x_pred = F * x = [temp + rate*dt, rate]
        let x_pred_temp = self.x_temp + self.x_rate * KALMAN_DT;
        let x_pred_rate = self.x_rate;

        // P_pred = F * P * F' + Q
        let p00_pred = self.p00 + KALMAN_DT * (self.p01 + self.p01) // symmetric
                       + KALMAN_DT * KALMAN_DT * self.p11
                       + KALMAN_Q_TEMP;
        let p01_pred = self.p01 + KALMAN_DT * self.p11 + KALMAN_Q_RATE * 0.5;
        let p11_pred = self.p11 + KALMAN_Q_RATE;

        // ---- Update step ----
        // Innovation: y = z - H * x_pred (H = [1, 0])
        let y = z - x_pred_temp;

        // Innovation covariance: S = H * P_pred * H' + R = P00_pred + R
        let s = p00_pred + KALMAN_R;
        let s_inv = 1.0 / s;

        // Kalman gain: K = P_pred * H' * S^-1 = [P00_pred/S, P01_pred/S]
        let k0 = p00_pred * s_inv;
        let k1 = p01_pred * s_inv;

        // Updated state: x = x_pred + K * y
        self.x_temp = x_pred_temp + k0 * y;
        self.x_rate = x_pred_rate + k1 * y;

        // Updated covariance: P = (I - K*H) * P_pred
        self.p00 = (1.0 - k0) * p00_pred;
        self.p01 = (1.0 - k0) * p01_pred;
        self.p11 = -k1 * p01_pred + p11_pred;

        self.x_temp as i64
    }

    /// Get the estimated rate of temperature change (mC per second).
    /// Drives the predictive SS trigger in `ss::SsAlgo::run_predictive`.
    pub fn rate(&self) -> f64 {
        self.x_rate
    }

    /// Predict temperature N seconds into the future (mC).
    pub fn predict(&self, seconds_ahead: f64) -> i64 {
        (self.x_temp + self.x_rate * seconds_ahead) as i64
    }
}

/// Sensor pool: pre-opened fds for all thermal zones + VSENSOR0 calculation.
pub struct SensorPool {
    /// Indices into thermal_zones for VSENSOR0 components (7 sensors).
    /// -1 = not found.
    vsensor_indices: [i32; 7],

    /// Per-sensor Kalman filters for VSENSOR0 components.
    vsensor_filters: [KalmanFilter; 7],

    /// Kalman filter for the fused VSENSOR0 output.
    /// Public so the hot path can read `.rate()` (dT/dt) for predictive SS.
    pub vsensor_fused_filter: KalmanFilter,

    /// All thermal zone fds: (name, fd)
    thermal_zones: Vec<(String, SysfsFd)>,

    /// Last computed VSENSOR0 value (mC).
    pub vsensor0: i64,

    /// Battery temperature (mC) — read separately for SIC algo.
    battery_zone_idx: i32,
    pub battery_temp: i64,

    // ---- Battery power telemetry (for SIC feedforward, Phase 1.2) ----
    current_now_fd: Option<SysfsFd>,
    voltage_now_fd: Option<SysfsFd>,
    /// Battery current in µA (negative = charging). 0 = telemetry unavailable.
    pub battery_current_ua: i64,
    /// Battery voltage in µV. 0 = telemetry unavailable (used as a validity sentinel).
    pub battery_voltage_uv: i64,
}

impl SensorPool {
    /// Initialize sensor pool by opening all thermal zone fds.
    pub fn new(thermal_zones: Vec<(String, SysfsFd)>) -> Self {
        let mut vsensor_indices = [-1i32; 7];
        let mut battery_zone_idx: i32 = -1;

        // Find the indices for VSENSOR0 components
        for (i, name) in VSENSOR_SENSOR_NAMES.iter().enumerate() {
            for (j, (zone_name, _)) in thermal_zones.iter().enumerate() {
                if zone_name == name {
                    vsensor_indices[i] = j as i32;
                    if *name == "battery" {
                        battery_zone_idx = j as i32;
                    }
                    break;
                }
            }
            if vsensor_indices[i] < 0 {
                warn!("VSENSOR0 sensor '{}' not found in thermal zones!", name);
            }
        }

        let zone_count = thermal_zones.len();
        info!("Mapped {} thermal zones for VSENSOR0", zone_count);

        SensorPool {
            vsensor_indices,
            vsensor_filters: [
                KalmanFilter::new(), KalmanFilter::new(), KalmanFilter::new(),
                KalmanFilter::new(), KalmanFilter::new(), KalmanFilter::new(),
                KalmanFilter::new(),
            ],
            vsensor_fused_filter: KalmanFilter::new(),
            thermal_zones,
            vsensor0: 25000, // 25°C initial
            battery_zone_idx,
            battery_temp: 30000,
            current_now_fd: SysfsFd::open_ro(BATTERY_CURRENT_PATH),
            voltage_now_fd: SysfsFd::open_ro(BATTERY_VOLTAGE_PATH),
            battery_current_ua: 0,
            battery_voltage_uv: 0,
        }
    }

    /// Read all sensors and compute VSENSOR0.
    /// This is called once per polling cycle.
    pub fn poll(&mut self) {
        // Read each VSENSOR0 component, apply per-sensor Kalman filter
        let mut weighted_sum: i64 = 0;
        let mut all_valid = true;

        for (i, (&idx, weight)) in self.vsensor_indices.iter()
            .zip(VSENSOR_WEIGHTS.iter())
            .enumerate()
        {
            if idx < 0 {
                all_valid = false;
                continue;
            }

            if let Some(raw_mc) = self.thermal_zones[idx as usize].1.read_int() {
                let filtered = self.vsensor_filters[i].update(raw_mc);
                weighted_sum += filtered * weight;

                // Battery sensor is index 0 in VSENSOR_SENSOR_NAMES ("battery").
                // Capture it here to avoid a redundant pread syscall below.
                if idx == self.battery_zone_idx {
                    self.battery_temp = raw_mc;
                }
            } else {
                all_valid = false;
            }
        }

        if all_valid {
            let raw_vsensor = weighted_sum / VSENSOR_WEIGHT_SUM + VSENSOR_COMPENSATION;
            // Apply Kalman filter to the fused VSENSOR0 output for extra smoothing
            self.vsensor0 = self.vsensor_fused_filter.update(raw_vsensor);
        }

        // Battery power telemetry for SIC feedforward. Reads are best-effort:
        // on failure we retain the previous value (0 = never-read sentinel, which
        // SIC treats as "no telemetry" and falls back to plain feedback PID).
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

    /// Charging current magnitude in µA (0 when discharging/idle).
    /// `current_now` is negative while charging on this platform, so the
    /// in-flowing magnitude is `max(0, -current_now)`.
    pub fn charge_current_ua(&self) -> i64 {
        (-self.battery_current_ua).max(0)
    }
}
