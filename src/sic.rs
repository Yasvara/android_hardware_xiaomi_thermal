/* --- thermal/src/sic.rs --- */
// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only
//! Smart Input Current (SIC) — multi-tier charging PID.
//!
//! Replaces the single-setpoint PID with the vendor's 9-tier step-trigger
//! table (RE'd from thermal-profiles.json / sconfig_0.txt). Each tier carries
//! its own trigger/clear hysteresis, PID setpoint, FCC band [min,max], and
//! gain schedule. Tiers with max==min (or target==0) are pure caps — no PID.
//!
//! Key fixes over the previous implementation:
//! - Multi-tier table eliminates the 31.8–38°C dead-zone
//! - Per-tier gain + setpoint eliminates overshoot / oscillation
//! - Reset-on-tier-change prevents integral wind-up across tier boundaries
//! - Anti-desync uses tolerance + unit validation (no more write-spam)
//! - Battery safeguard floor (called from main.rs) protects the pack
//!   independently of the VSENSOR0 fusion
use log::{info, warn};
use rustutils::android::system_properties::PropertyWatcher;
use crate::io::SysfsFd;
use crate::sensor::KalmanFilter;

// ================================================================
// Tier table — GLOBAL profile (sconfig=0), from thermal-profiles.json
// max/min in mA in the JSON → converted to µA here (*1000).
// Gain: vendor raw / 10000 to match the custom PID_DIVISOR=1000 scale.
// ================================================================

// ---- Custom Charging Profile IPC (mirror thermal custom profile) ----
pub(crate) const PROP_CHARGING_MODE: &str   = "persist.sys.mithermal.charging_profile"; // "global"|"custom"
pub(crate) const PROP_CHARGING_SEQ: &str    = "persist.sys.mithermal.charging.seq";
const PROP_CHARGING_COUNT: &str  = "persist.sys.mithermal.charging.n";
const PROP_CHARGING_CHUNK: &str  = "persist.sys.mithermal.charging.";
const CHARGING_CHUNK_SIZE: usize = 88;
const CHARGING_CHUNK_MAX: usize  = 64;
const MAX_CHARGING_TIERS: usize  = 12;

// Sane FCC bounds for validation (µA): 100 mA .. 25 A (silicon ABS, the live
// vendor-policy envelope is pushed separately via policy_envelope).
const FCC_HARD_MIN_UA: i64 = 100_000;
const FCC_HARD_MAX_UA: i64 = 25_000_000;

const DEFAULT_SUSPEND_MC: i64 = 48_000;
const DEFAULT_RESUME_MC: i64 = 45_000;

// ---- SOTA Charge-Pump Silicon Governor (independent of cell fusion) ----
pub fn cp_governor(cp_mc: i64) -> i64 {
    let t = cp_mc as f64 / 1000.0;
    if t < 60.0 { return 22_000_000; }
    if t < 65.0 { return (22e6 - (t - 60.0) / 5.0 * 10e6) as i64; }
    if t < 70.0 { return (12e6 - (t - 65.0) / 5.0 * 4e6) as i64; }
    if t < 75.0 { return (8e6  - (t - 70.0) / 5.0 * 3e6) as i64; }
    if t < 80.0 { return (5e6  - (t - 75.0) / 5.0 * 2e6) as i64; }
    2_000_000
}

// ---- Predictive temperature from the Kalman rate estimate ----
//
// Symmetric and bounded, evaluated on the FILTERED estimate.
//
// The previous version took `rate.max(0.0)` and then `base.max(ahead)`, and used
// the raw sample as its base. That made it a ratchet: it added heat on a rising
// edge, contributed nothing on a falling one, and passed raw measurement noise
// straight through before amplifying it by the lookahead. Stacked on the old
// `batt_ntc.max(blend)` fusion it put the control temperature several °C above
// the cell with no path back down.
//
// Lookahead, gain and cap are all GameSpace-writable (config::ChargeTuning).
pub fn predictive_temp(
    kf: &KalmanFilter,
    filtered_mc: i64,
    lookahead_s: f64,
    gain: f64,
    cap_mc: i64,
) -> i64 {
    let delta = kf.rate() * lookahead_s * gain;
    let delta = delta.clamp(-(cap_mc as f64), cap_mc as f64);
    filtered_mc + delta as i64
}

/// `"<user> <type>"` prefix for the mca_charge_interface sysfs grammar.
///
/// The type MUST NOT be "buck". mca_charge_interface exposes one op table per
/// charge path ("buck", "quick", "div1/2/4", ... "all" — see
/// g_mca_charge_if_op_type_table in mca_charge_interface.c) and a vote only
/// reaches the path it is addressed to. Addressing "buck" meant every FCC vote
/// and every emergency suspend missed the charge pump entirely: inert while the
/// pump was carrying the charge, and destructive in buck-only mode where it cut
/// the one live path and forced the pack to run the whole system.
/// Overridable via `persist.sys.mithermal.chg.path`.
const ICHG_LIMIT_PATH_DEFAULT: &str = "all";

/// Cycles a bypass reading must agree before a transition is trusted.
const BYPASS_DEBOUNCE_CYCLES: u32 = 3;
/// Below this the pack is not meaningfully charging, so cutting VBUS removes no
/// heat and only forces a discharge.
const SUSPEND_MIN_CHARGE_UA: i64 = 300_000;

/// Convert adapter APDO wattage (e.g. 140, 120, 100, 90, 65, 33) to max FCC current in µA.
/// Universal across Xiaomi and 3rd-party USB-PD 3.0 PPS chargers.
pub fn apdo_wattage_to_max_fcc_ua(apdo_watt: i64) -> i64 {
    match apdo_watt {
        100.. => 22_000_000,    // 100W/120W/140W PPS -> up to 22.0A cold burst
        85..=99 => 20_000_000,  // 90W PPS -> 20.0A (~90W adapter)
        60..=84 => 15_600_000,  // 65W/67W PPS -> 15.6A
        45..=59 => 11_000_000,  // 45W/55W PPS -> 11.0A
        25..=44 => 6_600_000,   // 30W/33W PPS -> 6.6A
        _ => 22_000_000,        // Unreported/0 -> defer to user max_fcc
    }
}

/// One SIC tier — cell-scale thermal-equilibrium charging schedule.
#[derive(Clone, Copy, Debug)]
struct SicTier {
    trig_mc: i64,     // engage threshold (mC)
    clr_mc: i64,      // release threshold (mC) — hysteresis
    target_mc: i64,   // PID setpoint (mC); 0 = pure cap, no PID
    max_ua: i64,      // FCC ceiling (µA)
    min_ua: i64,      // FCC floor (µA)
    ks: i64,          // proportional gain (velocity PID)
    ki: i64,          // integral gain
    kc: i64,          // derivative / damping gain
}

/// Cell-scale fast-yet-cool charging schedule for onyx (1S 6500mAh Si-C + single SC8581):
/// - Below 36.5°C: up to 22A cold burst (bounded by adapter APDO & max_fcc)
/// - 36.5..38.0°C: 20A (90W production peak)
/// - 38.0..40.5°C: smooth PID cruise 16A..11.5A (50-70W, optimal Cauchy-Schwarz cool plateau)
/// - 40.5..42.5°C: warm PID 11.5A..7.5A
/// - 42.5..44.0°C: 7.5A..4.5A
/// - >=44.0°C: progressive hard caps (3.5A -> 2.5A -> 1.5A -> 0.8A)
const SIC_TIERS_GLOBAL: [SicTier; 9] = [
    //  trig    clr     target  max_ua          min_ua          ks   ki  kc
    SicTier{trig_mc:15000, clr_mc:14000, target_mc:0,     max_ua:22_000_000, min_ua:22_000_000, ks:0,   ki:0,  kc:0},
    SicTier{trig_mc:36500, clr_mc:34500, target_mc:0,     max_ua:20_000_000, min_ua:20_000_000, ks:0,   ki:0,  kc:0},
    SicTier{trig_mc:38000, clr_mc:36500, target_mc:39500, max_ua:16_000_000, min_ua:11_500_000, ks:200, ki:10, kc:10},
    SicTier{trig_mc:40500, clr_mc:39000, target_mc:41500, max_ua:11_500_000, min_ua: 7_500_000, ks:200, ki:20, kc:20},
    SicTier{trig_mc:42500, clr_mc:41000, target_mc:43000, max_ua: 7_500_000, min_ua: 4_500_000, ks:200, ki:35, kc:35},
    SicTier{trig_mc:44000, clr_mc:42800, target_mc:0,     max_ua: 3_500_000, min_ua: 3_500_000, ks:0,   ki:0,  kc:0},
    SicTier{trig_mc:45000, clr_mc:44000, target_mc:0,     max_ua: 2_500_000, min_ua: 2_500_000, ks:0,   ki:0,  kc:0},
    SicTier{trig_mc:46000, clr_mc:45000, target_mc:0,     max_ua: 1_500_000, min_ua: 1_500_000, ks:0,   ki:0,  kc:0},
    SicTier{trig_mc:47000, clr_mc:46000, target_mc:0,     max_ua:   800_000, min_ua:   800_000, ks:0,   ki:0,  kc:0},
];

/// PID output range: 0..PID_OUTPUT_MAX maps linearly to [tier.max, tier.min].
const PID_OUTPUT_MAX: i64 = 8000;
const PID_DIVISOR: i64 = 1000;

// ---- Anti-desync ----
/// Don't re-write if the target changed by less than this (µA).
const FCC_WRITE_DEADBAND_UA: i64 = 50_000;   // 50 mA
/// Kernel may cap the real current below our setpoint (PD negotiation).
/// Only force a re-write if the readback diverges by more than this.
const FCC_DESYNC_TOLERANCE_UA: i64 = 200_000; // 200 mA
/// Sane readback range for unit validation (100 mA .. 23 A in µA).
const FCC_SANE_MIN_UA: i64 = 100_000;
const FCC_SANE_MAX_UA: i64 = 23_000_000;

// ---- Battery safeguard (called from main.rs) ----
/// Above this battery temp (mC), force FCC to at most BATT_SAFEGUARD_FCC_UA.
const BATT_SAFEGUARD_MC: i64 = 45_000;       // 45°C
const BATT_SAFEGUARD_FCC_UA: i64 = 3_000_000; // 3 A

// ---- Feedforward (kept from Phase 1.2) ----
const FEEDFORWARD_CURRENT_DELTA_UA: i64 = 3_000_000;
const FEEDFORWARD_OFFSET_LEVELS: i32 = 2;
const FEEDFORWARD_DECAY_CYCLES: u32 = 10;

// ---- Charge-phase detection (kept) ----
const CV_VOLTAGE_THRESHOLD_UV: i64 = 4_350_000;
const CHARGING_MIN_CURRENT_UA: i64 = 50_000;

#[derive(Clone, Copy, PartialEq, Debug)]
enum ChargePhase { Discharging, CC, CV }

// ================================================================
// Tier selector with hysteresis (same pattern as monitor::compute_level)
// ================================================================

/// Select the highest active tier. Levels ≤ `current` use `clr_mc` (conservative
/// release); levels above use `trig_mc` (engage). Returns -1 when below all tiers.
fn compute_tier_level(tiers: &[SicTier], current: i32, temp_mc: i64) -> i32 {
    let mut new_level: i32 = -1;
    for (i, tier) in tiers.iter().enumerate() {
        let idx = i as i32;
        if idx <= current {
            if temp_mc >= tier.clr_mc { new_level = idx; }
        } else if temp_mc >= tier.trig_mc {
            new_level = idx;
        }
    }
    new_level
}

// ================================================================
// SicAlgo
// ================================================================

pub struct SicAlgo {
    // Sysfs nodes
    fcc_fd1: Option<SysfsFd>,
    fcc_fd2: Option<SysfsFd>,
    thermal_remove_fd: Option<SysfsFd>,
    ichg_limit_fd: Option<SysfsFd>,
    suspend_fd: Option<SysfsFd>,
    /// Kernel bypass-charging state (mca_smart_charge). Absent on older kernels.
    bypass_state_fd: Option<SysfsFd>,
    /// Type-C adapter maximum negotiated APDO wattage (e.g. 90, 67, 33).
    apdo_max_fd: Option<SysfsFd>,
    /// True charger presence (prevents discharging fast-path from skipping
    /// plugged-in charging deficits and detects replug after kernel zeroes FCC).
    usb_online_fd: Option<SysfsFd>,
    wls_online_fd: Option<SysfsFd>,
    was_plugged: bool,
    #[cfg(test)]
    plugged_override: Option<bool>,

    // Tier state
    tier_level: i32,       // current tier index (-1 = below all)
    tier_floor: i32,       // battery-safeguard floor (-1 = inactive)

    // PID state (only meaningful inside PID tiers 3-5)
    ek_1: i64,
    ek_2: i64,
    output_level: i64,     // 0..PID_OUTPUT_MAX

    // SOTA Kalman filter for charge_temp
    temp_filter: KalmanFilter,

    // Applied state
    applied_fcc: i64,
    emergency_suspend: bool,

    // Feedforward
    prev_charge_current_ua: i64,
    feedforward_offset: i32,
    feedforward_decay: u32,
    charge_phase: ChargePhase,

    // ---- NEW: custom charging profile ----
    custom_tiers: Vec<SicTier>,      // empty = use SIC_TIERS_GLOBAL
    custom_seq: i64,
    custom_suspend_mc: i64,
    custom_resume_mc: i64,

    // ---- Live tuning (mirrored from config::ChargeTuning) ----
    chg_path: String,
    tune_suspend_mc: i64,
    tune_resume_mc: i64,
    suspend_hold_max: u32,
    pred_ahead_s: f64,
    pred_gain: f64,
    pred_cap_mc: i64,
    slew_ua: i64,

    // ---- Emergency state ----
    suspend_held: u32,
    bypass_last: bool,
    bypass_stable: u32,
    bypass_prev: bool,

    // ---- Cached adapter APDO max (avoids hammering ADSP Glink every tick) ----
    cached_apdo_fcc: Option<i64>,
    apdo_poll_cnt: u32,

    // ---- Zero-alloc property watchers for custom charging profile ----
    mode_watcher: PropertyWatcher,
    seq_watcher: PropertyWatcher,
}

impl SicAlgo {
    pub fn new() -> Self {
        let fcc_fd1 = SysfsFd::open_rw(
            "/sys/devices/platform/soc/soc:mca_charger_thermal/wired_chg_curr")
            .or_else(|| SysfsFd::open_rw(
                "/sys/class/power_supply/battery/constant_charge_current_max"));
        let fcc_fd2 = SysfsFd::open_rw(
            "/sys/devices/platform/soc/soc:mca_charger_thermal/wired_chg_curr2");
        let thermal_remove_fd = SysfsFd::open_rw(
            "/sys/devices/platform/soc/soc:mca_charger_thermal/wired_thermal_remove");
        let ichg_limit_fd = SysfsFd::open_rw(
            "/sys/devices/platform/soc/soc:mca_charge_interface/ichg_limit");
        let suspend_fd = SysfsFd::open_rw(
            "/sys/devices/platform/soc/soc:mca_charge_interface/input_suspend");
        let bypass_state_fd = SysfsFd::open_ro(
            "/sys/class/xm_power/charger/smart_charge/bypass_charge_state");
        let apdo_max_fd = SysfsFd::open_ro(
            "/sys/class/xm_power/typec/apdo_max")
            .or_else(|| SysfsFd::open_ro(
                "/sys/devices/virtual/xm_power/typec/apdo_max"));
        let usb_online_fd = SysfsFd::open_ro(
            "/sys/class/power_supply/usb/online");
        let wls_online_fd = SysfsFd::open_ro(
            "/sys/class/power_supply/wireless/online");

        SicAlgo {
            fcc_fd1, fcc_fd2, thermal_remove_fd, ichg_limit_fd, suspend_fd,
            bypass_state_fd, apdo_max_fd, usb_online_fd, wls_online_fd,
            was_plugged: false,
            #[cfg(test)]
            plugged_override: None,
            tier_level: -1,
            tier_floor: -1,
            ek_1: 0, ek_2: 0, output_level: 0,
            temp_filter: KalmanFilter::new(),
            applied_fcc: -1,
            emergency_suspend: false,
            prev_charge_current_ua: 0,
            feedforward_offset: 0,
            feedforward_decay: 0,
            charge_phase: ChargePhase::CC,
            custom_tiers: Vec::new(),
            custom_seq: -1,
            custom_suspend_mc: 0,
            custom_resume_mc: 0,
            chg_path: ICHG_LIMIT_PATH_DEFAULT.to_string(),
            tune_suspend_mc: 0,
            tune_resume_mc: 0,
            suspend_hold_max: 30,
            pred_ahead_s: 12.0,
            pred_gain: 0.6,
            pred_cap_mc: 2_000,
            slew_ua: 500_000,
            suspend_held: 0,
            bypass_last: false,
            bypass_stable: 0,
            bypass_prev: false,
            cached_apdo_fcc: None,
            apdo_poll_cnt: 0,
            mode_watcher: PropertyWatcher::new(PROP_CHARGING_MODE).expect("prop name"),
            seq_watcher: PropertyWatcher::new(PROP_CHARGING_SEQ).expect("prop name"),
        }
    }

    /// Mirror the GameSpace-writable tuning into the algorithm.
    pub fn apply_tuning(&mut self, t: &crate::config::ChargeTuning) {
        if self.chg_path != t.chg_path {
            self.chg_path = t.chg_path.clone();
        }
        self.tune_suspend_mc = t.suspend_mc;
        self.tune_resume_mc = t.resume_mc;
        self.suspend_hold_max = t.suspend_hold_cycles;
        self.pred_ahead_s = t.pred_ahead_s;
        self.pred_gain = t.pred_gain;
        self.pred_cap_mc = t.pred_cap_mc;
        self.slew_ua = t.slew_ua;
    }

    /// The active tier table: custom if loaded, else vendor GLOBAL.
    fn active_tiers(&self) -> &[SicTier] {
        if self.custom_tiers.is_empty() {
            &SIC_TIERS_GLOBAL
        } else {
            &self.custom_tiers
        }
    }

    /// Emergency thresholds, on the CELL scale.
    /// Profile > global tuning prop > compiled fallback.
    fn suspend_threshold(&self) -> i64 {
        if !self.custom_tiers.is_empty() && self.custom_suspend_mc > 0 {
            self.custom_suspend_mc
        } else if self.tune_suspend_mc > 0 {
            self.tune_suspend_mc
        } else {
            DEFAULT_SUSPEND_MC
        }
    }

    fn resume_threshold(&self) -> i64 {
        if !self.custom_tiers.is_empty() && self.custom_resume_mc > 0 {
            self.custom_resume_mc
        } else if self.tune_resume_mc > 0 {
            self.tune_resume_mc
        } else {
            DEFAULT_RESUME_MC
        }
    }

    /// Reload custom charging profile from chunked props (zero-alloc when unchanged).
    pub fn maybe_reload_custom(&mut self) {
        let is_custom = self
            .mode_watcher
            .read(|_, v| v.trim() == "custom")
            .unwrap_or(false);
        if !is_custom {
            if !self.custom_tiers.is_empty() {
                info!("SIC: charging profile -> GLOBAL ({} tiers dropped)",
                    self.custom_tiers.len());
                self.custom_tiers.clear();
                self.custom_seq = -1;
                self.output_level = 0;
                self.ek_1 = 0;
                self.ek_2 = 0;
                self.tier_level = -1;
            }
            return;
        }
        let seq = self
            .seq_watcher
            .read(|_, v| v.trim().parse::<i64>().unwrap_or(-1))
            .unwrap_or(-1);
        if seq == self.custom_seq && !self.custom_tiers.is_empty() {
            return; // unchanged
        }
        match read_and_parse_charging_profile() {
            Some((tiers, suspend_mc, resume_mc)) => {
                info!("SIC: custom charging profile loaded (seq={}, {} tiers, suspend={:.1}°C)",
                    seq, tiers.len(), suspend_mc as f64 / 1000.0);
                self.custom_tiers = tiers;
                self.custom_seq = seq;
                self.custom_suspend_mc = suspend_mc;
                self.custom_resume_mc = resume_mc;
                self.output_level = 0;
                self.ek_1 = 0;
                self.ek_2 = 0;
                self.tier_level = -1;
            }
            None => warn!("SIC: failed to parse custom charging profile (seq={}), keeping current", seq),
        }
    }

    /// Compatibility shim (no telemetry).
    #[allow(dead_code)]
    pub fn run(&mut self, charge_temp_mc: i64, target_mc: i64, max_fcc_ua: i64, min_fcc_ua: i64) {
        self.run_with_feedforward(charge_temp_mc, charge_temp_mc, charge_temp_mc, target_mc, max_fcc_ua, min_fcc_ua, 0, 0);
    }

    /// Battery safeguard floor — called from main.rs each cycle with physical NTC.
    pub fn update_battery_safeguard(&mut self, cell_mc: i64) {
        let active = self.active_tiers();
        if cell_mc > BATT_SAFEGUARD_MC {
            let floor = active.iter()
                .position(|t| t.max_ua <= BATT_SAFEGUARD_FCC_UA)
                .map(|i| i as i32)
                .unwrap_or((active.len() - 1) as i32);
            if self.tier_floor != floor {
                info!("SIC: cell {:.1}°C → safeguard floor tier {}",
                    cell_mc as f64 / 1000.0, floor);
                self.tier_floor = floor;
            }
        } else if self.tier_floor >= 0 && cell_mc < BATT_SAFEGUARD_MC - 2000 {
            self.tier_floor = -1; // 2°C hysteresis to release
        }
    }

    /// Main per-cycle entry point at nominal 1 Hz (kept for unit tests).
    pub fn run_with_feedforward(
        &mut self,
        charge_temp_mc: i64,
        cell_mc: i64,
        charger_therm_mc: i64,
        _target_mc: i64,
        max_fcc_ua: i64,
        min_fcc_ua: i64,
        charge_current_ua: i64,
        battery_voltage_uv: i64,
    ) {
        self.run_with_feedforward_dt(
            charge_temp_mc,
            cell_mc,
            charger_therm_mc,
            _target_mc,
            max_fcc_ua,
            min_fcc_ua,
            charge_current_ua,
            battery_voltage_uv,
            1.0,
        );
    }

    /// Main per-cycle entry point with true elapsed `dt_s` (accurate dT/dt at 1s and 3s).
    /// Skips ADSP Glink sysfs polling while cleanly unplugged/discharging.
    #[allow(clippy::too_many_arguments)]
    pub fn run_with_feedforward_dt(
        &mut self,
        charge_temp_mc: i64,
        cell_mc: i64,
        charger_therm_mc: i64,
        _target_mc: i64,
        max_fcc_ua: i64,
        min_fcc_ua: i64,
        charge_current_ua: i64,
        battery_voltage_uv: i64,
        dt_s: f64,
    ) {
        // ---- Phase detection ----
        let telemetry = battery_voltage_uv > 0;
        let phase = if !telemetry {
            ChargePhase::CC
        } else if charge_current_ua < CHARGING_MIN_CURRENT_UA {
            ChargePhase::Discharging
        } else if battery_voltage_uv > CV_VOLTAGE_THRESHOLD_UV {
            ChargePhase::CV
        } else {
            ChargePhase::CC
        };
        let phase_changed = phase != self.charge_phase;
        if phase_changed {
            info!("SIC: phase {:?} → {:?} (I={}µA V={}µV)",
                self.charge_phase, phase, charge_current_ua, battery_voltage_uv);
            self.charge_phase = phase;
        }

        // ---- True charger presence (USB / wireless online) ----
        let plugged_in = self.is_plugged_in(telemetry, charge_current_ua);
        let plug_attached = plugged_in && !self.was_plugged;
        let plug_detached = !plugged_in && self.was_plugged;
        self.was_plugged = plugged_in;
        if plug_attached {
            // Kernel poweroff_clear_voter zeroes wired_chg_curr on disconnect;
            // invalidate applied_fcc and APDO cache so replug re-applies immediately.
            self.applied_fcc = -1;
            self.cached_apdo_fcc = None;
            self.apdo_poll_cnt = 0;
        } else if plug_detached {
            self.prev_charge_current_ua = 0;
            self.cached_apdo_fcc = None;
            self.apdo_poll_cnt = 0;
        }

        // ---- Predictive temperature (true dt_s for accurate rate during 3s screen-off) ----
        let filtered = self.temp_filter.update_dt(charge_temp_mc, dt_s);

        // Fast-path ONLY while genuinely unplugged: never skip while plugged in
        // even if net pack current is negative (charging deficit / heavy load).
        if telemetry
            && !plugged_in
            && phase == ChargePhase::Discharging
            && !phase_changed
            && self.applied_fcc > 0
        {
            self.prev_charge_current_ua = 0;
            self.cached_apdo_fcc = None;
            self.apdo_poll_cnt = 0;
            return;
        }

        // ---- Feedforward ----
        if telemetry
            && charge_current_ua - self.prev_charge_current_ua > FEEDFORWARD_CURRENT_DELTA_UA
        {
            self.feedforward_offset = FEEDFORWARD_OFFSET_LEVELS;
            self.feedforward_decay = FEEDFORWARD_DECAY_CYCLES;
            info!("SIC: charger plug (ΔI={}µA) → feedforward +{} levels",
                charge_current_ua - self.prev_charge_current_ua, self.feedforward_offset);
        }
        self.prev_charge_current_ua = charge_current_ua;

        let eff = predictive_temp(
            &self.temp_filter, filtered,
            self.pred_ahead_s, self.pred_gain, self.pred_cap_mc);

        // ---- Tier selection with hysteresis on predictive effective temp ----
        let new_tier = compute_tier_level(self.active_tiers(), self.tier_level, eff);
        let effective_tier = new_tier.max(self.tier_floor);

        if effective_tier != self.tier_level {
            info!("SIC: tier {} → {} (cT={:.1}°C eff={:.1}°C table={})",
                self.tier_level, effective_tier, charge_temp_mc as f64 / 1000.0,
                eff as f64 / 1000.0,
                if self.custom_tiers.is_empty() { "GLOBAL" } else { "CUSTOM" });
            self.output_level = 0;
            self.ek_1 = 0;
            self.ek_2 = 0;
            self.tier_level = effective_tier;
        }

        // ---- Dynamic Adapter APDO & User Ceiling (cached every 10 ticks / phase change) ----
        if self.apdo_poll_cnt == 0 || phase_changed || plug_attached {
            self.cached_apdo_fcc = self.apdo_max_fd
                .as_ref()
                .and_then(|fd| fd.read_int())
                .filter(|&w| w > 0)
                .map(apdo_wattage_to_max_fcc_ua);
            // Poll faster (2 ticks) right after plug-in until APDO wattage settles.
            self.apdo_poll_cnt = if plugged_in && self.cached_apdo_fcc.is_none() {
                2
            } else {
                10
            };
        } else {
            self.apdo_poll_cnt -= 1;
        }
        let dynamic_ceiling_ua = self.cached_apdo_fcc.unwrap_or(max_fcc_ua).min(max_fcc_ua);

        // ---- Compute target FCC ----
        let tiers_len = self.active_tiers().len();
        let tier: Option<SicTier> = if effective_tier >= 0 && (effective_tier as usize) < tiers_len {
            Some(self.active_tiers()[effective_tier as usize])
        } else {
            None
        };

        let target_fcc = match tier {
            None => {
                // Below all tiers — full charge, PID already reset above.
                self.active_tiers().first().map(|t| t.max_ua.min(dynamic_ceiling_ua)).unwrap_or(dynamic_ceiling_ua)
            }
            Some(t) => {
                if t.target_mc == 0 || t.max_ua == t.min_ua {
                    if effective_tier == 0 {
                        t.max_ua.min(dynamic_ceiling_ua)
                    } else {
                        t.max_ua
                    }
                } else {
                    let ek = eff - t.target_mc;

                    let (eff_ki, eff_kc) = match phase {
                        ChargePhase::CV          => (t.ki / 3, t.kc * 2),
                        ChargePhase::Discharging => (0,        t.kc),
                        ChargePhase::CC          => (t.ki,     t.kc),
                    };

                    let delta = (t.ks * (ek - self.ek_1)
                               + eff_ki * ek
                               + eff_kc * (ek - 2 * self.ek_1 + self.ek_2))
                              / PID_DIVISOR;

                    self.output_level = (self.output_level + delta).clamp(0, PID_OUTPUT_MAX);

                    self.ek_2 = self.ek_1;
                    self.ek_1 = ek;

                    let frac = self.output_level as f64 / PID_OUTPUT_MAX as f64;
                    let pid_fcc = (t.max_ua as f64
                        - frac * (t.max_ua - t.min_ua) as f64) as i64;

                    let ff_shift = self.feedforward_offset as i64
                        * (t.max_ua - t.min_ua) / 8;
                    (pid_fcc - ff_shift).clamp(t.min_ua, t.max_ua)
                }
            }
        };

        // (a) Charge-Pump Governor (clamps ceiling based on charger_therm silicon)
        let target_fcc = target_fcc.min(cp_governor(charger_therm_mc));

        // (b) Hard physical NTC safeguard (caps at 3A when real cell > 45°C)
        let target_fcc = if cell_mc > 45_000 {
            target_fcc.min(3_000_000)
        } else {
            target_fcc
        };

        // ---- User clamp (GameSpace max/min & dynamic adapter ceiling) ----
        let (floor_ua, ceil_ua) = if self.custom_tiers.is_empty() {
            (min_fcc_ua, dynamic_ceiling_ua)
        } else {
            (FCC_HARD_MIN_UA, FCC_HARD_MAX_UA)
        };
        let target_fcc = target_fcc.clamp(floor_ua.min(ceil_ua), ceil_ua);

        // ---- Slew limit ----
        // Ramp instead of stepping. The tier table needed wide hysteresis bands
        // purely to damp its own step changes; bounding the rate of change
        // removes the excitation rather than filtering the response.
        let target_fcc = if self.slew_ua > 0 && self.applied_fcc > 0 {
            let delta = target_fcc - self.applied_fcc;
            if delta.abs() > self.slew_ua {
                self.applied_fcc + delta.signum() * self.slew_ua
            } else {
                target_fcc
            }
        } else {
            target_fcc
        };

        // ---- Anti-desync write with tolerance + unit validation ----
        let mut needs_write = self.thermal_limit_removed()
            || (target_fcc - self.applied_fcc).abs() > FCC_WRITE_DEADBAND_UA;
        if !needs_write {
            if let Some(ref fd) = self.fcc_fd1 {
                if let Some(actual) = fd.read_int() {
                    // Kernel poweroff_clear_voter zeroes wired_chg_curr on unplug;
                    // while plugged in, actual == 0 means our vote must be restored.
                    if (plugged_in && actual == 0)
                        || (actual > FCC_SANE_MIN_UA
                            && actual < FCC_SANE_MAX_UA
                            && (actual - target_fcc).abs() > FCC_DESYNC_TOLERANCE_UA)
                    {
                        needs_write = true;
                    }
                }
            }
        }

        if needs_write {
            self.write_fcc(target_fcc);
            let tier_label = if effective_tier >= 0 { effective_tier } else { -1 };
            info!("SIC: cT={:.1}°C NTC={:.1}°C CP={:.1}°C tier={} {:?} ff={} table={} → FCC={} µA ({:.1}A)",
                charge_temp_mc as f64 / 1000.0,
                cell_mc as f64 / 1000.0,
                charger_therm_mc as f64 / 1000.0,
                tier_label,
                phase,
                self.feedforward_offset,
                if self.custom_tiers.is_empty() { "GLOBAL" } else { "CUSTOM" },
                target_fcc,
                target_fcc as f64 / 1_000_000.0);
            self.applied_fcc = target_fcc;
        }

        // ---- Feedforward decay ----
        if self.feedforward_decay > 0 {
            self.feedforward_decay -= 1;
            if self.feedforward_decay == 0 {
                self.feedforward_offset = 0;
            }
        }
    }

    /// True when USB or wireless charger is physically online.
    /// Falls back to current telemetry when sysfs nodes are absent (host tests).
    fn is_plugged_in(&self, telemetry: bool, charge_current_ua: i64) -> bool {
        #[cfg(test)]
        if let Some(v) = self.plugged_override {
            return v;
        }
        let mut any_fd = false;
        if let Some(ref fd) = self.usb_online_fd {
            any_fd = true;
            if fd.read_int().unwrap_or(0) != 0 {
                return true;
            }
        }
        if let Some(ref fd) = self.wls_online_fd {
            any_fd = true;
            if fd.read_int().unwrap_or(0) != 0 {
                return true;
            }
        }
        if any_fd {
            charge_current_ua >= CHARGING_MIN_CURRENT_UA
        } else {
            !telemetry || charge_current_ua >= CHARGING_MIN_CURRENT_UA
        }
    }

    /// True while the kernel holds the pack out of the charge loop.
    /// Mirrors enum mca_bypass_chg_state:
    ///   1 = ACTIVE, 2 = DEFICIT, 3 = ASSIST, 4 = FULL, 5 = CP_BYPASS.
    fn bypass_engaged(&self) -> bool {
        let fd = match self.bypass_state_fd {
            Some(ref fd) => fd,
            None => return false,
        };
        let mut buf = [0u8; 32];
        // "<state> <ibat_mA> <fcc_mA>" — only the leading state field matters here.
        match fd.read_str(&mut buf) {
            Some(s) => Self::parse_bypass_state_engaged(s),
            None => false,
        }
    }

    fn parse_bypass_state_engaged(s: &str) -> bool {
        match s.split_whitespace().next() {
            Some(tok) => matches!(tok.parse::<i64>(), Ok(1..=5)),
            None => false,
        }
    }

    /// Emergency charge suspend.
    ///
    /// Suspending the input is only protective while the charger is actually
    /// pushing current into the pack. Under bypass it inverts: charging is
    /// already off, so cutting VBUS does not remove any charging heat — it just
    /// dumps the whole system load back onto the battery. Measured on onyx
    /// during Genshin, the suspend took the pack from 0 mA / 41.3 °C to 1.8 A
    /// discharge / 44.3 °C, i.e. the "protection" heated the cell it was meant
    /// to protect. So while bypass is engaged we skip the suspend, and release
    /// one we already latched.
    /// `cell_mc` must be the CELL role (fuel gauge), never the board sensor:
    /// `batt_therm` runs ~6°C hot, so a board-scale threshold fires early and,
    /// worse, cannot clear — the suspend moves the system load onto the pack,
    /// which keeps the board hot, which holds the suspend. Measured: latched at
    /// 10:36:30 and still latched three minutes later, bleeding 1.2 A into a
    /// plugged-in 90 W adapter.
    pub fn check_emergency(
        &mut self,
        cell_mc: i64,
        pump_mc: i64,
        charge_current_ua: i64,
    ) -> bool {
        // Fast-path: when not suspended and not meaningfully charging, skip ADSP bypass sysfs read.
        if !self.emergency_suspend && charge_current_ua <= SUSPEND_MIN_CHARGE_UA {
            return false;
        }

        let suspend_mc = self.suspend_threshold();
        let resume_mc = self.resume_threshold();

        // Debounce the bypass reading so one stray sample cannot arm the latch.
        let raw_bypass = self.bypass_engaged();
        if raw_bypass == self.bypass_last {
            self.bypass_stable = self.bypass_stable.saturating_add(1);
        } else {
            self.bypass_stable = 0;
            self.bypass_last = raw_bypass;
        }
        if self.bypass_stable >= BYPASS_DEBOUNCE_CYCLES {
            self.bypass_prev = raw_bypass;
        }
        let bypass = self.bypass_prev;

        // Suspending is only protective while the charger is actually pushing
        // current into the pack. Under bypass, or at ~0 A, it inverts: it dumps
        // the whole system load onto the cell and heats what it meant to protect.
        let would_help = !bypass && charge_current_ua > SUSPEND_MIN_CHARGE_UA;

        if self.emergency_suspend {
            self.suspend_held += 1;
            let why = if cell_mc < resume_mc {
                Some("cell cooled")
            } else if !would_help {
                Some("no charge current / bypass engaged")
            } else if self.suspend_held >= self.suspend_hold_max {
                Some("max hold elapsed, re-evaluating")
            } else {
                None
            };
            if let Some(why) = why {
                self.emergency_suspend = false;
                self.suspend_held = 0;
                if let Some(ref fd) = self.suspend_fd {
                    fd.write_str(&format!("mipower {} 0", self.chg_path));
                }
                info!("Emergency released ({}) at cell {:.1}°C", why, cell_mc as f64 / 1000.0);
            }
            return self.emergency_suspend;
        }

        if (cell_mc > suspend_mc || pump_mc > crate::config::HW_PUMP_MAX_MC) && would_help {
            self.emergency_suspend = true;
            self.suspend_held = 0;
            if let Some(ref fd) = self.suspend_fd {
                fd.write_str(&format!("mipower {} 1", self.chg_path));
            }
            warn!("EMERGENCY: cell {:.1}°C pump {:.1}°C → suspended (max {} cycles)",
                cell_mc as f64 / 1000.0, pump_mc as f64 / 1000.0, self.suspend_hold_max);
        }
        self.emergency_suspend
    }

    /// Push a new FCC setpoint down to the charger.
    ///
    /// `wired_thermal_remove` is the vendor's *escape hatch*, not an arm switch.
    /// A 1 sends mca_charger_thermal_handle_limit() down the branch that
    /// re-votes every votable with `enabled=false`, dropping the "mca_thermal"
    /// client entirely — and its file-static `thermal_removed` latch then makes
    /// every later `wired_chg_curr` write a no-op. The node keeps echoing back
    /// whatever we last stored, so the readback looks healthy while the charger
    /// runs on some other voter (measured: node said 1.77 A, pack drew 2.08 A,
    /// effective client `volt_thermal_limit` at 3.0 A). Our tier limit only
    /// reaches the hardware while this reads 0.
    fn write_fcc(&self, fcc_ua: i64) {
        // Final hardware clamp. Profile policy may tighten this, never loosen it.
        let fcc_ua = fcc_ua.clamp(0, crate::config::HW_FCC_MAX_UA);
        let fcc_ma = fcc_ua / 1000;
        if let Some(ref fd) = self.thermal_remove_fd { fd.write_int(0); }
        if let Some(ref fd) = self.fcc_fd1 { fd.write_int(fcc_ua); }
        if let Some(ref fd) = self.fcc_fd2 { fd.write_int(fcc_ua); }
        // mca_charge_interface grammar:
        //   "buck"  expects "<user> buck <mA>"
        //   "quick" expects "<user> quick <single_mA>#<multi_mA>"
        // Writing "<user> all <mA>" silently returns -1 on quickchg!
        if let Some(ref fd) = self.ichg_limit_fd {
            if self.chg_path == "all" {
                fd.write_prefixed_int("mithermal buck", fcc_ma);
                fd.write_prefixed_pair("mithermal quick", fcc_ma, fcc_ma);
            } else if self.chg_path == "quick" {
                fd.write_prefixed_pair("mithermal quick", fcc_ma, fcc_ma);
            } else {
                let mut prefix = [0u8; 32];
                let head = b"mithermal ";
                let path = self.chg_path.as_bytes();
                if head.len() + path.len() <= prefix.len() {
                    prefix[..head.len()].copy_from_slice(head);
                    prefix[head.len()..head.len() + path.len()].copy_from_slice(path);
                    if let Ok(p) = std::str::from_utf8(&prefix[..head.len() + path.len()]) {
                        fd.write_prefixed_int(p, fcc_ma);
                    }
                }
            }
        }
    }

    /// True when something re-armed the vendor escape hatch behind our back,
    /// which silently voids the active FCC vote.
    fn thermal_limit_removed(&self) -> bool {
        self.thermal_remove_fd
            .as_ref()
            .and_then(|fd| fd.read_int())
            .is_some_and(|v| v != 0)
    }
}

// ================================================================
// Helpers: Prop readers & Custom Profile JSON Parser
// ================================================================

pub(crate) fn read_prop_str_local(prop: &str) -> Option<String> {
    match rustutils::android::system_properties::read(prop) {
        Ok(Some(val)) if !val.trim().is_empty() => Some(val),
        _ => None,
    }
}

pub(crate) fn read_prop_i64_local(prop: &str) -> Option<i64> {
    read_prop_str_local(prop).and_then(|v| v.trim().parse::<i64>().ok())
}

pub(crate) fn read_charging_chunks() -> Option<String> {
    let count = read_prop_i64_local(PROP_CHARGING_COUNT).unwrap_or(0);
    if count <= 0 || count as usize > CHARGING_CHUNK_MAX {
        warn!("SIC: invalid charging chunk count {}", count);
        return None;
    }
    let mut json = String::with_capacity(count as usize * CHARGING_CHUNK_SIZE);
    for i in 0..count {
        let name = format!("{}{}", PROP_CHARGING_CHUNK, i);
        match rustutils::android::system_properties::read(&name) {
            Ok(Some(chunk)) if !chunk.is_empty() => json.push_str(&chunk),
            _ => {
                warn!("SIC: missing charging chunk {}/{}", i, count);
                return None;
            }
        }
    }
    Some(json)
}

fn read_and_parse_charging_profile() -> Option<(Vec<SicTier>, i64, i64)> {
    let json = read_charging_chunks()?;
    parse_charging_json(&json)
}

fn parse_charging_json(json: &str) -> Option<(Vec<SicTier>, i64, i64)> {
    let mut tiers: Vec<SicTier> = Vec::with_capacity(MAX_CHARGING_TIERS);

    let suspend_mc = extract_top_level_i64(json, "suspend").unwrap_or(DEFAULT_SUSPEND_MC);
    let mut resume_mc = extract_top_level_i64(json, "resume").unwrap_or(DEFAULT_RESUME_MC);
    let real_suspend = if suspend_mc <= 0 { DEFAULT_SUSPEND_MC } else { suspend_mc };
    if resume_mc <= 0 || resume_mc >= real_suspend {
        resume_mc = real_suspend - 3000;
    }

    let arr_start = json.find("\"tiers\"")?;
    let bracket_start = json[arr_start..].find('[').map(|i| i + arr_start)?;
    let bracket_end = json[bracket_start..].find(']').map(|i| i + bracket_start)?;
    let body = &json[bracket_start + 1..bracket_end];

    let mut depth = 0i32;
    let mut obj_start: Option<usize> = None;
    let bytes = body.as_bytes();
    for (i, &b) in bytes.iter().enumerate() {
        match b {
            b'{' => {
                if depth == 0 { obj_start = Some(i); }
                depth += 1;
            }
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    if let Some(start) = obj_start.take() {
                        let obj = &body[start..=i];
                        if let Some(tier) = parse_one_tier(obj) {
                            if tiers.len() < MAX_CHARGING_TIERS {
                                tiers.push(tier);
                            }
                        }
                    }
                }
            }
            _ => {}
        }
    }

    if tiers.is_empty() {
        warn!("SIC: no valid charging tiers parsed (torn/garbage read?)");
        return None;
    }

    for t in tiers.iter_mut() {
        t.max_ua = t.max_ua.clamp(FCC_HARD_MIN_UA, FCC_HARD_MAX_UA);
        t.min_ua = t.min_ua.clamp(FCC_HARD_MIN_UA, t.max_ua);
        if t.ks < 0 { t.ks = 0; }
        if t.ki < 0 { t.ki = 0; }
        if t.kc < 0 { t.kc = 0; }
        if t.clr_mc >= t.trig_mc { t.clr_mc = t.trig_mc - 1000; }
        if t.clr_mc <= 0 { t.clr_mc = t.trig_mc - 2000; }
    }
    tiers.sort_by_key(|t| t.trig_mc);
    tiers.dedup_by_key(|t| t.trig_mc);

    info!("SIC: parsed {} charging tiers, suspend={:.1}°C resume={:.1}°C",
        tiers.len(), real_suspend as f64 / 1000.0, resume_mc as f64 / 1000.0);
    Some((tiers, real_suspend, resume_mc))
}

fn parse_one_tier(obj: &str) -> Option<SicTier> {
    let trig   = extract_field_i64(obj, "trig")?;
    let clr    = extract_field_i64(obj, "clr").unwrap_or(trig - 2000);
    let target = extract_field_i64(obj, "target").unwrap_or(0);
    let max    = extract_field_i64(obj, "max")?;
    let min    = extract_field_i64(obj, "min").unwrap_or(max);
    let ks     = extract_field_i64(obj, "ks").unwrap_or(0);
    let ki     = extract_field_i64(obj, "ki").unwrap_or(0);
    let kc     = extract_field_i64(obj, "kc").unwrap_or(0);
    if trig <= 0 || max <= 0 {
        return None;
    }
    Some(SicTier {
        trig_mc: trig, clr_mc: clr, target_mc: target,
        max_ua: max, min_ua: min, ks, ki, kc,
    })
}

pub(crate) fn extract_field_i64(s: &str, key: &str) -> Option<i64> {
    let pat = format!("\"{}\"", key);
    let idx = s.find(&pat)?;
    let rest = &s[idx + pat.len()..];
    let colon = rest.find(':')?;
    parse_leading_int(&rest[colon + 1..])
}

fn extract_top_level_i64(s: &str, key: &str) -> Option<i64> {
    let tiers_idx = s.find("\"tiers\"").unwrap_or(s.len());
    extract_field_i64(&s[..tiers_idx], key)
}

pub(crate) fn parse_leading_int(s: &str) -> Option<i64> {
    let b = s.as_bytes();
    let mut i = 0;
    while i < b.len() && (b[i] == b' ' || b[i] == b'\t') { i += 1; }
    let neg = i < b.len() && b[i] == b'-';
    if neg || (i < b.len() && b[i] == b'+') { i += 1; }
    let mut n: i64 = 0;
    let mut any = false;
    while i < b.len() && b[i].is_ascii_digit() {
        n = n.wrapping_mul(10).wrapping_add((b[i] - b'0') as i64);
        any = true;
        i += 1;
    }
    if !any { None } else { Some(if neg { -n } else { n }) }
}

// ================================================================
// Tests
// ================================================================
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tier_cold_below_all() {
        assert_eq!(compute_tier_level(&SIC_TIERS_GLOBAL, -1, 25_000), 0);
        assert_eq!(compute_tier_level(&SIC_TIERS_GLOBAL, -1, 14_999), -1);
    }

    #[test]
    fn tier_rising_sequence() {
        let temps = [15_000, 36_500, 38_000, 40_500, 42_500,
                     44_000, 45_000, 46_000, 47_000];
        let mut tier = -1;
        for (i, &t) in temps.iter().enumerate() {
            tier = compute_tier_level(&SIC_TIERS_GLOBAL, tier, t);
            assert_eq!(tier, i as i32, "at {}°C expected tier {}", t / 1000, i);
        }
    }

    #[test]
    fn tier_hysteresis_holds_on_drop() {
        let mut tier = compute_tier_level(&SIC_TIERS_GLOBAL, -1, 44_000);
        assert_eq!(tier, 5);
        tier = compute_tier_level(&SIC_TIERS_GLOBAL, tier, 43_000);
        assert_eq!(tier, 5, "43°C > clr 42.8°C → stay tier 5");
        tier = compute_tier_level(&SIC_TIERS_GLOBAL, tier, 42_800);
        assert_eq!(tier, 5, "42.8°C == clr → stay");
        tier = compute_tier_level(&SIC_TIERS_GLOBAL, tier, 42_799);
        assert_eq!(tier, 4, "42.799°C < clr → drop to tier 4");
    }

    #[test]
    fn tier_full_drop_to_cold() {
        let mut tier = compute_tier_level(&SIC_TIERS_GLOBAL, -1, 47_000);
        assert_eq!(tier, 8);
        for &(t, expect) in &[
            (45_999, 7), (44_999, 6), (43_999, 5), (42_799, 4),
            (40_999, 3), (38_999, 2), (36_499, 1), (34_499, 0),
            (13_999, -1),
        ] {
            tier = compute_tier_level(&SIC_TIERS_GLOBAL, tier, t);
            assert_eq!(tier, expect, "at {}°C", t / 1000);
        }
    }

    #[test]
    fn bypass_engaged_covers_all_active_states() {
        assert!(!SicAlgo::parse_bypass_state_engaged("0 0 0"));
        assert!(SicAlgo::parse_bypass_state_engaged("1 -50 100"));  // ACTIVE
        assert!(SicAlgo::parse_bypass_state_engaged("2 -300 600")); // DEFICIT
        assert!(SicAlgo::parse_bypass_state_engaged("3 -400 3000")); // ASSIST
        assert!(SicAlgo::parse_bypass_state_engaged("4 0 0"));       // FULL
        assert!(SicAlgo::parse_bypass_state_engaged("5 -20 0"));     // CP_BYPASS
        assert!(!SicAlgo::parse_bypass_state_engaged(""));
    }

    #[test]
    fn apdo_universal_third_party_tiers() {
        assert_eq!(apdo_wattage_to_max_fcc_ua(140), 22_000_000);
        assert_eq!(apdo_wattage_to_max_fcc_ua(100), 22_000_000);
        assert_eq!(apdo_wattage_to_max_fcc_ua(90), 20_000_000);
        assert_eq!(apdo_wattage_to_max_fcc_ua(65), 15_600_000);
        assert_eq!(apdo_wattage_to_max_fcc_ua(45), 11_000_000);
        assert_eq!(apdo_wattage_to_max_fcc_ua(30), 6_600_000);
        assert_eq!(apdo_wattage_to_max_fcc_ua(0), 22_000_000);
    }

    fn make_algo() -> SicAlgo {
        SicAlgo::new()
    }

    fn run_cycle(a: &mut SicAlgo, temp: i64) -> i64 {
        a.run_with_feedforward(temp, temp, temp, 38_000, 15_480_000, 860_000, 5_000_000, 4_000_000);
        a.applied_fcc
    }

    /// Hold a temperature until the loop is stationary, then report FCC.
    ///
    /// A single cycle no longer tells you the tier cap: FCC is slew-limited to
    /// 500 mA per cycle, and the predictor adds the Kalman rate, which is large
    /// right after a step. Both settle to zero contribution once the input is
    /// held, so a cap assertion has to be made on the settled value or it is
    /// really asserting on the ramp.
    fn settle(a: &mut SicAlgo, temp: i64) -> i64 {
        let mut fcc = 0;
        for _ in 0..80 {
            fcc = run_cycle(a, temp);
        }
        fcc
    }

    #[test]
    fn sic_cold_full_charge() {
        let mut a = make_algo();
        let fcc = run_cycle(&mut a, 30_000);
        assert_eq!(fcc, 15_480_000, "30°C → tier 0 cap (clamped to user max)");
    }

    // A cap assertion has to start from a fresh algo. KalmanFilter::update
    // seeds itself from its first sample and reports zero rate, so cycle one is
    // the only cycle where the effective temperature is exactly the input:
    // afterwards the predictor's rate term and the tier hysteresis both carry
    // history, and "the tier at 32°C" genuinely depends on how it got there.
    // Approaching 32°C from 30°C really does latch tier 2 - the rate spike
    // pushes eff over trig[2]=33.5°C and 32°C never falls back under
    // clr[2]=30.8°C. That is the table working, not a bug, so it does not
    // belong in a test about caps.

    #[test]
    fn sic_tier1_clamped_by_user_max() {
        let mut a = make_algo();
        let fcc = run_cycle(&mut a, 37_000);
        // Tier 1 (36.5°C) caps at 20 A, above the 15.48 A user max run_cycle passes,
        // so the user clamp binds here.
        assert_eq!(a.tier_level, 1);
        assert_eq!(fcc, 15_480_000, "37°C → tier 1, clamped to the 15.48A user max");
    }

    #[test]
    fn sic_tier2_pid_cool_cruise() {
        let mut a = make_algo();
        a.run_with_feedforward(39_000, 39_000, 39_000, 38_000, 20_000_000, 860_000, 5_000_000, 4_000_000);
        assert_eq!(a.tier_level, 2);
        assert!(a.applied_fcc >= 11_500_000 && a.applied_fcc <= 16_000_000,
            "39°C → tier 2 cool cruise PID [11.5A, 16A], got {}", a.applied_fcc);
    }

    #[test]
    fn sic_pid_tier4_band() {
        let mut a = make_algo();
        // Realistic warming rate (~3°C/min = 50 m°C/s) so Kalman lookahead stays inside tier 4.
        for t in (40_000..=42_600).step_by(50) {
            run_cycle(&mut a, t);
        }
        let mut fcc = 0;
        for _ in 0..30 {
            fcc = run_cycle(&mut a, 43_000);
        }
        assert!(fcc >= 4_500_000 && fcc <= 7_500_000,
            "tier 4 PID FCC {} out of [4.5A,7.5A]", fcc);
    }

    #[test]
    fn sic_pure_cap_tier6() {
        let mut a = make_algo();
        for t in [35_000, 39_000, 42_500, 45_000] {
            run_cycle(&mut a, t);
        }
        let fcc = settle(&mut a, 45_000);
        assert_eq!(fcc, 2_500_000, "45.0°C → tier 6 pure cap 2.5A");
    }

    #[test]
    fn sic_anti_windup_no_overshoot() {
        let mut a = make_algo();
        for t in [34_000, 38_500, 44_000] { run_cycle(&mut a, t); }
        for _ in 0..60 { run_cycle(&mut a, 44_200); }

        for _ in 0..30 { run_cycle(&mut a, 32_000); }
        assert!(a.tier_level < 5, "cooling to 32°C must leave tier 5, got {}", a.tier_level);
        assert_eq!(a.output_level, 0, "PID must reset on tier change");

        let mut min_fcc = i64::MAX;
        for t in [36_000, 39_000, 41_500] {
            let fcc = run_cycle(&mut a, t);
            min_fcc = min_fcc.min(fcc);
        }
        for _ in 0..20 {
            let fcc = run_cycle(&mut a, 43_000);
            min_fcc = min_fcc.min(fcc);
        }
        assert!(min_fcc >= 4_500_000,
            "anti-windup failed: FCC dropped to {} (<4.5A)", min_fcc);
    }

    #[test]
    fn sic_user_clamp_respected() {
        let mut a = make_algo();
        a.run_with_feedforward(30_000, 30_000, 30_000, 38_000, 10_000_000, 860_000, 5_000_000, 4_000_000);
        assert_eq!(a.applied_fcc, 10_000_000, "user max 10A must clamp tier 0");
    }

    #[test]
    fn custom_profile_ignores_global_min_fcc_floor() {
        let mut a = SicAlgo::new();
        // A "5 W cap" profile: 860 mA at the top, trickling to 100 mA when hot.
        a.custom_tiers = vec![
            SicTier{trig_mc:31_000,clr_mc:29_000,target_mc:0,max_ua:860_000,min_ua:860_000,ks:0,ki:0,kc:0},
            SicTier{trig_mc:41_000,clr_mc:39_000,target_mc:0,max_ua:100_000,min_ua:100_000,ks:0,ki:0,kc:0},
        ];
        // min_fcc_ua is the 5 W global floor from persist.sys.mithermal.min_fcc.
        a.run_with_feedforward(42_000, 42_000, 42_000, 38_000, 15_480_000, 860_000, 5_000_000, 4_000_000);
        assert_eq!(a.applied_fcc, 100_000,
            "custom tier floor must survive the global min_fcc floor");
    }

    #[test]
    fn vendor_table_still_honours_min_fcc_floor() {
        let mut a = SicAlgo::new();
        // 47.0°C, not 46.0°C: tier 8 triggers at 46.5°C, so a steady 46.0°C is
        // genuinely tier 7. The old ratchet predictor only reached tier 8 there
        // because it added its rising-edge bias on top of the sample — exactly
        // the over-aggression this test should not be encoding.
        for t in [30_000, 35_000, 40_000, 44_500, 47_000] {
            a.run_with_feedforward(t, t, t, 38_000, 15_480_000, 1_000_000, 5_000_000, 4_000_000);
        }
        // FCC is slew-limited now, so the floor is reached by ramping rather than
        // in one step: 15.48 A down to 1 A at 500 mA per 1 Hz cycle is ~29 cycles.
        for _ in 0..60 {
            a.run_with_feedforward(47_000, 47_000, 47_000, 38_000,
                                   15_480_000, 1_000_000, 5_000_000, 4_000_000);
        }
        assert_eq!(a.applied_fcc, 1_000_000,
            "vendor tier 8 (600 mA) must still be lifted to the 1 A min_fcc floor");
    }

    #[test]
    fn clamp_survives_max_below_min() {
        let mut a = SicAlgo::new();
        // A custom SS profile may hand down a sic_max_fcc under config.min_fcc;
        // clamp() would panic on an inverted range.
        a.run_with_feedforward(30_000, 30_000, 30_000, 38_000, 500_000, 860_000, 5_000_000, 4_000_000);
        assert_eq!(a.applied_fcc, 500_000, "ceiling wins when it sits below the floor");
    }

    #[test]
    fn sic_battery_safeguard_floor() {
        let mut a = make_algo();
        // BATT_SAFEGUARD_MC is 45°C and the comparison is strict, so 44°C is
        // deliberately *not* enough to arm the floor.
        a.update_battery_safeguard(46_000);
        assert!(a.tier_floor >= 6, "safeguard must set floor ≥ tier 6");
        for t in [30_000, 35_000, 38_000] { run_cycle(&mut a, t); }
        let fcc = settle(&mut a, 38_000);
        assert!(fcc <= BATT_SAFEGUARD_FCC_UA,
            "battery safeguard must cap FCC ≤ {}, got {}", BATT_SAFEGUARD_FCC_UA, fcc);
    }

    #[test]
    fn sic_battery_safeguard_does_not_arm_below_threshold() {
        let mut a = make_algo();
        a.update_battery_safeguard(44_000);
        assert_eq!(a.tier_floor, -1, "44°C is under the 45°C safeguard threshold");
    }

    #[test]
    fn sic_battery_safeguard_releases() {
        let mut a = make_algo();
        a.update_battery_safeguard(46_000);
        assert!(a.tier_floor >= 0);
        a.update_battery_safeguard(40_000);
        assert_eq!(a.tier_floor, -1, "safeguard must release at 40°C");
    }

    #[test]
    fn sim_90w_charging_no_oscillation() {
        let mut a = make_algo();
        let mut t_skin: f64 = 30.0;
        let t_amb: f64 = 28.0;
        let c_thermal: f64 = 12.0;
        let r_thermal: f64 = 0.35;
        let v_charge: f64 = 4.2;

        let mut cycles_below_1a = 0u32;
        let mut min_current = f64::MAX;

        for _ in 0..240 {
            let temp_mc = (t_skin * 1000.0) as i64;
            a.run_with_feedforward(temp_mc, temp_mc, temp_mc, 38_000, 15_480_000, 860_000,
                                   5_000_000, 4_000_000);
            let current_a = a.applied_fcc as f64 / 1_000_000.0;
            if current_a < 1.0 { cycles_below_1a += 1; }
            min_current = min_current.min(current_a);

            let p_charge = v_charge * current_a;
            let p_dissipate = (t_skin - t_amb) / r_thermal;
            t_skin += (p_charge - p_dissipate) / c_thermal;
        }

        assert_eq!(cycles_below_1a, 0,
            "oscillation: {} cycles below 1A (min {:.2}A)", cycles_below_1a, min_current);
        assert!(t_skin < 46.5, "skin temp {:.1}°C too high", t_skin);
        assert!(t_skin > 30.0, "skin temp {:.1}°C — throttle never engaged?", t_skin);
        let final_a = a.applied_fcc as f64 / 1_000_000.0;
        assert!(final_a >= 2.0 && final_a <= 12.0,
            "final current {:.1}A outside stable band [2,12]A", final_a);
    }

    #[test]
    fn sim_30w_slow_charge_stable() {
        let mut a = make_algo();
        let mut t_skin: f64 = 28.0;
        let (t_amb, c, r, v) = (25.0, 15.0, 0.30, 4.0);
        let mut min_current = f64::MAX;

        for _ in 0..300 {
            let temp_mc = (t_skin * 1000.0) as i64;
            a.run_with_feedforward(temp_mc, temp_mc, temp_mc, 38_000, 15_480_000, 860_000,
                                   3_000_000, 4_000_000);
            let current_a = a.applied_fcc as f64 / 1_000_000.0;
            min_current = min_current.min(current_a);
            let p = v * current_a;
            let pd = (t_skin - t_amb) / r;
            t_skin += (p - pd) / c;
        }

        assert!(min_current >= 5.0, "slow charge dipped to {:.1}A", min_current);
        // Cell-scale cool-cruise PID setpoint is 39.5°C (tier 2) / 41.5°C (tier 3).
        assert!(t_skin < 41.5, "slow charge overheated to {:.1}°C", t_skin);
    }

    #[test]
    fn custom_charging_parse_and_validate() {
        let json = r#"{"suspend":47000,"resume":44000,"tiers":[
            {"trig":33000,"clr":32000,"target":34000,"max":9000000,"min":5000000,"ks":200,"ki":5,"kc":5},
            {"trig":38000,"clr":37000,"target":39000,"max":5000000,"min":2500000,"ks":200,"ki":50,"kc":50},
            {"trig":44000,"clr":43500,"target":0,"max":1000000,"min":1000000,"ks":0,"ki":0,"kc":0}
        ]}"#;
        let (tiers, susp, res) = parse_charging_json(json).unwrap();
        assert_eq!(tiers.len(), 3);
        assert_eq!(susp, 47000);
        assert_eq!(res, 44000);
        assert!(tiers[0].trig_mc < tiers[1].trig_mc && tiers[1].trig_mc < tiers[2].trig_mc);
        assert_eq!(tiers[2].target_mc, 0);
    }

    #[test]
    fn custom_charging_repairs_bad_hysteresis() {
        let json = r#"{"tiers":[{"trig":35000,"clr":36000,"target":0,"max":5000000,"min":9000000,"ks":0,"ki":0,"kc":0}]}"#;
        let (tiers, _, _) = parse_charging_json(json).unwrap();
        assert!(tiers[0].clr_mc < tiers[0].trig_mc, "clr must be repaired below trig");
        assert!(tiers[0].min_ua <= tiers[0].max_ua, "min must clamp to max");
    }

    /// The predictor must be able to subtract, not only add. The old ratchet
    /// (`rate.max(0.0)` then `base.max(ahead)`) could raise the control
    /// temperature on a rising edge but never lower it on a falling one, so a
    /// throttle state had no path back down.
    #[test]
    fn predictor_is_symmetric() {
        let mut rising = KalmanFilter::new();
        for i in 0..20 { rising.update(30_000 + i * 500); }
        let up = predictive_temp(&rising, 40_000, 12.0, 0.6, 2_000);
        assert!(up > 40_000, "rising temp must predict above base, got {}", up);

        let mut falling = KalmanFilter::new();
        for i in 0..20 { falling.update(40_000 - i * 500); }
        let down = predictive_temp(&falling, 40_000, 12.0, 0.6, 2_000);
        assert!(down < 40_000, "falling temp must predict below base, got {}", down);
    }

    /// The lookahead contribution is bounded, so a noise spike in the rate
    /// estimate cannot move the setpoint arbitrarily far.
    #[test]
    fn predictor_contribution_is_capped() {
        let mut kf = KalmanFilter::new();
        for i in 0..20 { kf.update(30_000 + i * 5_000); } // absurd 5°C/s ramp
        let p = predictive_temp(&kf, 40_000, 12.0, 0.6, 2_000);
        assert!(p <= 42_000, "cap breached: {} > 42000", p);
        assert!(p >= 38_000, "cap breached low: {}", p);
    }

    #[test]
    fn custom_charging_rejects_empty() {
        assert!(parse_charging_json(r#"{"tiers":[]}"#).is_none());
        assert!(parse_charging_json(r#"garbage"#).is_none());
    }

    #[test]
    fn sim_custom_profile_no_oscillation() {
        let mut a = SicAlgo::new();
        a.custom_tiers = vec![
            SicTier{trig_mc:33000,clr_mc:32000,target_mc:34000,max_ua:6_000_000,min_ua:3_000_000,ks:200,ki:5,kc:5},
            SicTier{trig_mc:40000,clr_mc:39000,target_mc:41000,max_ua:2_000_000,min_ua:1_000_000,ks:350,ki:25,kc:25},
        ];
        let mut t = 30.0f64;
        let mut below_1a = 0u32;
        for _ in 0..240 {
            a.run_with_feedforward((t*1000.0) as i64, (t*1000.0) as i64, (t*1000.0) as i64, 38000, 15_480_000, 860_000, 5_000_000, 4_000_000);
            let cur = a.applied_fcc as f64 / 1e6;
            if cur < 1.0 { below_1a += 1; }
            let p = 4.2 * cur;
            t += (p - (t - 28.0)/0.35) / 12.0;
        }
        assert_eq!(below_1a, 0, "custom profile must not oscillate below 1A");
    }

    #[test]
    fn plugged_in_deficit_does_not_skip_sic() {
        let mut a = make_algo();
        a.plugged_override = Some(true);
        // Initial tick while plugged in and draining (-500 mA) at 30°C
        a.run_with_feedforward(30_000, 30_000, 30_000, 38_000, 20_000_000, 860_000, -500_000, 4_000_000);
        assert_eq!(a.applied_fcc, 20_000_000);

        // Temperature rises to 45°C while STILL plugged in and draining:
        // discharging fast-path MUST NOT skip SIC while USB is online.
        for _ in 0..80 {
            a.run_with_feedforward(45_000, 45_000, 45_000, 38_000, 20_000_000, 860_000, -500_000, 4_000_000);
        }
        assert_eq!(a.applied_fcc, 2_500_000,
            "plugged-in negative current must still update SIC tier/FCC");
    }

    #[test]
    fn replug_invalidates_applied_fcc_and_reapplies() {
        let mut a = make_algo();
        a.plugged_override = Some(true);
        a.run_with_feedforward(30_000, 30_000, 30_000, 38_000, 20_000_000, 860_000, 3_000_000, 4_000_000);
        assert_eq!(a.applied_fcc, 20_000_000);
        a.cached_apdo_fcc = Some(6_600_000);

        // Unplug
        a.plugged_override = Some(false);
        a.run_with_feedforward(30_000, 30_000, 30_000, 38_000, 20_000_000, 860_000, -400_000, 4_000_000);
        assert!(a.cached_apdo_fcc.is_none(), "unplug must clear stale APDO cache");

        // Replug with a higher-wattage charger: must immediately apply full 20A (not slew from old)
        a.plugged_override = Some(true);
        a.run_with_feedforward(30_000, 30_000, 30_000, 38_000, 20_000_000, 860_000, 3_000_000, 4_000_000);
        assert_eq!(a.applied_fcc, 20_000_000);
    }
}
