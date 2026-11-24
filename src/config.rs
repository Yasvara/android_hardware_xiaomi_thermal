// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Runtime configuration via system properties.
//!
//! Optimizations:
//! - User CPU/GPU ceilings (`persist.sys.mithermal.user_cpu_caps`, `user_gpu_cap`) are watched
//!   via zero-allocation `PropertyWatcher` every cycle (eliminates Pentas's 2s polling race).
//! - Slow charging/thermal config properties reload every 5 cycles and set a `chg_dirty` flag
//!   so `SensorPool::apply_tuning` / `SicAlgo::apply_tuning` never re-scan or clone on steady ticks.

use log::info;
use rustutils::android::system_properties::{self, PropertyWatcher};

const PROP_POLL_INTERVAL: u32 = 5;

const PROP_TEMP_LIMIT: &str = "persist.sys.mithermal.temp_limit";
const PROP_MAX_FCC: &str = "persist.sys.mithermal.max_fcc";
const PROP_MIN_FCC: &str = "persist.sys.mithermal.min_fcc";
const PROP_ENABLED: &str = "persist.sys.mithermal.enabled";
const PROP_TEMP_OFFSET: &str = "persist.sys.mithermal.temp_offset";

/// User CPU frequency ceilings from Pentas (`"max0,max1,max2,max3"` kHz; `0` = uncapped).
const PROP_USER_CPU_CAPS: &str = "persist.sys.mithermal.user_cpu_caps";
/// User GPU frequency ceiling from Pentas (`"max_hz"`; `0` = uncapped).
const PROP_USER_GPU_CAP: &str = "persist.sys.mithermal.user_gpu_cap";

const TEMP_OFFSET_MIN_MC: i64 = -15_000;
const TEMP_OFFSET_MAX_MC: i64 = 15_000;

// ---- Charging tuning ----
const PROP_CELL_SENSOR: &str   = "persist.sys.mithermal.chg.cell_sensor";
const PROP_BOARD_SENSOR: &str  = "persist.sys.mithermal.chg.board_sensor";
const PROP_PUMP_SENSOR: &str   = "persist.sys.mithermal.chg.pump_sensor";
const PROP_BOARD_OFFSET: &str  = "persist.sys.mithermal.chg.board_offset";
const PROP_W_CELL: &str        = "persist.sys.mithermal.chg.w_cell";
const PROP_W_BOARD: &str       = "persist.sys.mithermal.chg.w_board";
const PROP_CHG_PATH: &str      = "persist.sys.mithermal.chg.path";
const PROP_SUSPEND_T: &str     = "persist.sys.mithermal.chg.suspend_t";
const PROP_RESUME_T: &str      = "persist.sys.mithermal.chg.resume_t";
const PROP_SUSPEND_HOLD: &str  = "persist.sys.mithermal.chg.suspend_hold";
const PROP_PRED_AHEAD: &str    = "persist.sys.mithermal.chg.pred_ahead";
const PROP_PRED_GAIN: &str     = "persist.sys.mithermal.chg.pred_gain";
const PROP_PRED_CAP: &str      = "persist.sys.mithermal.chg.pred_cap";
const PROP_SLEW: &str          = "persist.sys.mithermal.chg.slew";
const PROP_NETZERO: &str       = "persist.sys.mithermal.chg.netzero";
const PROP_NETZERO_TARGET: &str = "persist.sys.mithermal.chg.netzero_ma";

// ---- Hardware envelope: absolute silicon/chemistry ceiling ----
pub const HW_CELL_MAX_MC: i64 = 55_000;
pub const HW_PUMP_MAX_MC: i64 = 80_000;
pub const HW_FCC_MAX_UA: i64  = 25_000_000;

pub const HW_FV_MAX_MV: i64 = 4_750;
pub const HW_FV_MIN_MV: i64 = 3_000;
pub const HW_TEMP_MIN_C: i64 = -20;
pub const HW_TEMP_MAX_C: i64 = 60;
pub const HW_FCC_MAX_MA: i64 = 25_000;
pub const HW_IIN_MAX_MA: i64 = 8_000;
pub const HW_ITERM_MIN_MA: i64 = 50;
pub const HW_ITERM_MAX_MA: i64 = 6_000;

pub fn is_temp_sentinel(c: i64) -> bool {
    c <= -1000 || c >= 1000
}

#[derive(Clone, PartialEq)]
pub struct ChargeTuning {
    pub cell_sensor: String,
    pub board_sensor: String,
    pub pump_sensor: String,
    pub board_offset_mc: i64,
    pub w_cell: i64,
    pub w_board: i64,
    pub chg_path: String,
    pub suspend_mc: i64,
    pub resume_mc: i64,
    pub suspend_hold_cycles: u32,
    pub pred_ahead_s: f64,
    pub pred_gain: f64,
    pub pred_cap_mc: i64,
    pub slew_ua: i64,
    pub netzero: bool,
    pub netzero_target_ua: i64,
}

impl Default for ChargeTuning {
    fn default() -> Self {
        ChargeTuning {
            cell_sensor: "battery".to_string(),
            board_sensor: "batt_therm".to_string(),
            pump_sensor: "charger_therm".to_string(),
            board_offset_mc: 6_000,
            w_cell: 800,
            w_board: 200,
            chg_path: "all".to_string(),
            suspend_mc: 50_000,
            resume_mc: 46_000,
            suspend_hold_cycles: 30,
            pred_ahead_s: 12.0,
            pred_gain: 0.6,
            pred_cap_mc: 2_000,
            slew_ua: 500_000,
            netzero: false,
            netzero_target_ua: 0,
        }
    }
}

impl ChargeTuning {
    fn reload(&mut self) -> bool {
        let mut changed = false;
        let mut update_str = |dst: &mut String, prop: &str| {
            if let Some(v) = read_prop_str(prop) {
                if *dst != v {
                    *dst = v;
                    changed = true;
                }
            }
        };
        update_str(&mut self.cell_sensor, PROP_CELL_SENSOR);
        update_str(&mut self.board_sensor, PROP_BOARD_SENSOR);
        update_str(&mut self.pump_sensor, PROP_PUMP_SENSOR);
        update_str(&mut self.chg_path, PROP_CHG_PATH);

        if let Some(v) = read_prop_i64(PROP_BOARD_OFFSET) {
            let nv = (v * 100).clamp(-20_000, 20_000);
            if nv != self.board_offset_mc { self.board_offset_mc = nv; changed = true; }
        }
        if let Some(v) = read_prop_i64(PROP_W_CELL) {
            let nv = v.clamp(0, 1000);
            if nv != self.w_cell { self.w_cell = nv; changed = true; }
        }
        if let Some(v) = read_prop_i64(PROP_W_BOARD) {
            let nv = v.clamp(0, 1000);
            if nv != self.w_board { self.w_board = nv; changed = true; }
        }
        if self.w_cell + self.w_board == 0 { self.w_cell = 1000; self.w_board = 0; changed = true; }

        if let Some(v) = read_prop_i64(PROP_SUSPEND_T) {
            let nv = (v * 100).min(HW_CELL_MAX_MC);
            if nv != self.suspend_mc { self.suspend_mc = nv; changed = true; }
        }
        if let Some(v) = read_prop_i64(PROP_RESUME_T) {
            let nv = v * 100;
            if nv != self.resume_mc { self.resume_mc = nv; changed = true; }
        }
        if self.suspend_mc > HW_CELL_MAX_MC { self.suspend_mc = HW_CELL_MAX_MC; changed = true; }
        if self.resume_mc >= self.suspend_mc {
            self.resume_mc = self.suspend_mc - 3_000;
            changed = true;
        }
        if let Some(v) = read_prop_i64(PROP_SUSPEND_HOLD) {
            let nv = v.clamp(5, 600) as u32;
            if nv != self.suspend_hold_cycles { self.suspend_hold_cycles = nv; changed = true; }
        }
        if let Some(v) = read_prop_i64(PROP_PRED_AHEAD) {
            let nv = (v as f64).clamp(0.0, 60.0);
            if nv != self.pred_ahead_s { self.pred_ahead_s = nv; changed = true; }
        }
        if let Some(v) = read_prop_i64(PROP_PRED_GAIN) {
            let nv = (v as f64 / 1000.0).clamp(0.0, 2.0);
            if nv != self.pred_gain { self.pred_gain = nv; changed = true; }
        }
        if let Some(v) = read_prop_i64(PROP_PRED_CAP) {
            let nv = (v * 100).clamp(0, 10_000);
            if nv != self.pred_cap_mc { self.pred_cap_mc = nv; changed = true; }
        }
        if let Some(v) = read_prop_i64(PROP_SLEW) {
            let nv = v.max(0);
            if nv != self.slew_ua { self.slew_ua = nv; changed = true; }
        }
        if let Some(v) = read_prop_bool(PROP_NETZERO) {
            if v != self.netzero { self.netzero = v; changed = true; }
        }
        if let Some(v) = read_prop_i64(PROP_NETZERO_TARGET) {
            let nv = (v * 1000).clamp(-500_000, 3_000_000);
            if nv != self.netzero_target_ua { self.netzero_target_ua = nv; changed = true; }
        }
        changed
    }
}

/// Runtime configuration state.
pub struct Config {
    pub temp_limit_mc: i64,
    pub max_fcc_ua: i64,
    pub min_fcc_ua: i64,
    pub enabled: bool,
    pub temp_offset_mc: i64,
    pub chg: ChargeTuning,
    pub chg_dirty: bool,

    /// Per-cluster user CPU ceilings in kHz (`0` = uncapped).
    pub user_cpu_caps: [i64; 4],
    /// User GPU ceiling in Hz (`0` = uncapped).
    pub user_gpu_cap_hz: i64,
    pub caps_dirty: bool,

    cpu_caps_watcher: PropertyWatcher,
    gpu_cap_watcher: PropertyWatcher,
    poll_counter: u32,
}

impl Config {
    pub fn new() -> Self {
        let mut cfg = Config {
            temp_limit_mc: 38_000,
            max_fcc_ua: 20_000_000,
            min_fcc_ua: 100_000,
            enabled: true,
            temp_offset_mc: 0,
            chg: ChargeTuning::default(),
            chg_dirty: true,
            user_cpu_caps: [0; 4],
            user_gpu_cap_hz: 0,
            caps_dirty: true,
            cpu_caps_watcher: PropertyWatcher::new(PROP_USER_CPU_CAPS).expect("prop name"),
            gpu_cap_watcher: PropertyWatcher::new(PROP_USER_GPU_CAP).expect("prop name"),
            poll_counter: 0,
        };
        cfg.force_reload();
        cfg.poll_user_caps();
        cfg
    }

    /// Zero-allocation per-cycle check of user CPU/GPU clock ceilings, plus 5s slow config reload.
    pub fn maybe_reload(&mut self) {
        self.poll_user_caps();
        self.poll_counter += 1;
        if self.poll_counter >= PROP_POLL_INTERVAL {
            self.poll_counter = 0;
            self.force_reload();
        }
    }

    pub fn take_chg_dirty(&mut self) -> bool {
        let d = self.chg_dirty;
        self.chg_dirty = false;
        d
    }

    pub fn take_caps_dirty(&mut self) -> bool {
        let d = self.caps_dirty;
        self.caps_dirty = false;
        d
    }

    /// Parse `user_cpu_caps` ("c0,c1,c2,c3") and `user_gpu_cap` in-place with ZERO heap allocation.
    fn poll_user_caps(&mut self) {
        let next_cpu = self
            .cpu_caps_watcher
            .read(|_, v| parse_cpu_caps_csv(v))
            .unwrap_or([0; 4]);
        if next_cpu != self.user_cpu_caps {
            info!(
                "Config: user_cpu_caps = [{}, {}, {}, {}] kHz",
                next_cpu[0], next_cpu[1], next_cpu[2], next_cpu[3]
            );
            self.user_cpu_caps = next_cpu;
            self.caps_dirty = true;
        }

        let next_gpu = self
            .gpu_cap_watcher
            .read(|_, v| v.trim().parse::<i64>().unwrap_or(0).max(0))
            .unwrap_or(0);
        if next_gpu != self.user_gpu_cap_hz {
            info!("Config: user_gpu_cap = {} Hz", next_gpu);
            self.user_gpu_cap_hz = next_gpu;
            self.caps_dirty = true;
        }
    }

    fn force_reload(&mut self) {
        let old_limit = self.temp_limit_mc;
        let old_max = self.max_fcc_ua;
        let old_min = self.min_fcc_ua;
        let old_enabled = self.enabled;
        let old_offset = self.temp_offset_mc;

        if let Some(val) = read_prop_i64(PROP_TEMP_LIMIT) {
            self.temp_limit_mc = val * 100;
        }
        if let Some(val) = read_prop_i64(PROP_MAX_FCC) {
            self.max_fcc_ua = val;
        }
        if let Some(val) = read_prop_i64(PROP_MIN_FCC) {
            self.min_fcc_ua = val;
        }
        if self.min_fcc_ua > self.max_fcc_ua {
            self.min_fcc_ua = self.max_fcc_ua;
        }
        if let Some(val) = read_prop_bool(PROP_ENABLED) {
            self.enabled = val;
        }
        if let Some(val) = read_prop_i64(PROP_TEMP_OFFSET) {
            self.temp_offset_mc = (val * 100).clamp(TEMP_OFFSET_MIN_MC, TEMP_OFFSET_MAX_MC);
        } else {
            self.temp_offset_mc = 0;
        }

        if self.chg.reload() {
            self.chg_dirty = true;
            info!(
                "Config: chg cell='{}' board='{}'(off {:+.1}°C) pump='{}' path={} \
                 susp={:.1}°C res={:.1}°C hold={}s netzero={}",
                self.chg.cell_sensor,
                self.chg.board_sensor,
                self.chg.board_offset_mc as f64 / 1000.0,
                self.chg.pump_sensor,
                self.chg.chg_path,
                self.chg.suspend_mc as f64 / 1000.0,
                self.chg.resume_mc as f64 / 1000.0,
                self.chg.suspend_hold_cycles,
                self.chg.netzero
            );
        }

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

fn parse_cpu_caps_csv(raw: &str) -> [i64; 4] {
    let mut caps = [0i64; 4];
    for (i, tok) in raw.split(',').take(4).enumerate() {
        caps[i] = tok.trim().parse::<i64>().unwrap_or(0).max(0);
    }
    caps
}

fn read_prop_i64(prop: &str) -> Option<i64> {
    match system_properties::read(prop) {
        Ok(Some(val)) => val.trim().parse::<i64>().ok(),
        _ => None,
    }
}

fn read_prop_str(prop: &str) -> Option<String> {
    match system_properties::read(prop) {
        Ok(Some(val)) if !val.trim().is_empty() => Some(val.trim().to_string()),
        _ => None,
    }
}

fn read_prop_bool(prop: &str) -> Option<bool> {
    match system_properties::read(prop) {
        Ok(Some(val)) => {
            let v = val.trim();
            Some(v == "true" || v == "1")
        }
        _ => None,
    }
}
