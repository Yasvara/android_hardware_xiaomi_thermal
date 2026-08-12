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
use crate::io::SysfsFd;

// ================================================================
// Tier table — GLOBAL profile (sconfig=0), from thermal-profiles.json
// max/min in mA in the JSON → converted to µA here (*1000).
// Gain: vendor raw / 10000 to match the custom PID_DIVISOR=1000 scale.
// ================================================================

// ---- Custom Charging Profile IPC (mirror thermal custom profile) ----
const PROP_CHARGING_MODE: &str   = "persist.sys.mithermal.charging_profile"; // "global"|"custom"
const PROP_CHARGING_SEQ: &str    = "persist.sys.mithermal.charging.seq";
const PROP_CHARGING_COUNT: &str  = "persist.sys.mithermal.charging.n";
const PROP_CHARGING_CHUNK: &str  = "persist.sys.mithermal.charging.";
const CHARGING_CHUNK_SIZE: usize = 88;
const CHARGING_CHUNK_MAX: usize  = 64;
const MAX_CHARGING_TIERS: usize  = 12;

// Sane FCC bounds for validation (µA): 100 mA .. 20 A
const FCC_HARD_MIN_UA: i64 = 100_000;
const FCC_HARD_MAX_UA: i64 = 20_000_000;

const DEFAULT_SUSPEND_MC: i64 = 48_000;
const DEFAULT_RESUME_MC: i64  = 45_000;

/// `"<user> <type>"` prefix for the mca_charge_interface sysfs grammar.
const ICHG_LIMIT_CLIENT: &str = "mithermal buck";

/// One SIC tier — mirrors a row of the vendor `[GLOBAL-SIC-BAT]` block.
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

const SIC_TIERS_GLOBAL: [SicTier; 9] = [
    //  trig    clr     target  max_ua      min_ua      ks   ki  kc
    SicTier{trig_mc:15000, clr_mc:14000, target_mc:0,     max_ua:15_600_000, min_ua:15_600_000, ks:0,   ki:0,  kc:0},
    SicTier{trig_mc:31800, clr_mc:25000, target_mc:0,     max_ua:12_400_000, min_ua:12_400_000, ks:0,   ki:0,  kc:0},
    SicTier{trig_mc:33500, clr_mc:30800, target_mc:32000, max_ua: 9_000_000, min_ua: 9_000_000, ks:0,   ki:0,  kc:0},
    SicTier{trig_mc:34600, clr_mc:33000, target_mc:36000, max_ua: 9_000_000, min_ua: 5_000_000, ks:200, ki:5,  kc:5},
    SicTier{trig_mc:37000, clr_mc:35500, target_mc:38000, max_ua: 8_000_000, min_ua: 5_000_000, ks:200, ki:5,  kc:5},
    SicTier{trig_mc:39000, clr_mc:37400, target_mc:40500, max_ua: 5_000_000, min_ua: 2_500_000, ks:200, ki:50, kc:50},
    SicTier{trig_mc:44500, clr_mc:44000, target_mc:44500, max_ua: 2_000_000, min_ua: 2_000_000, ks:350, ki:3,  kc:25},
    SicTier{trig_mc:45000, clr_mc:44500, target_mc:45000, max_ua: 1_000_000, min_ua: 1_000_000, ks:350, ki:3,  kc:25},
    SicTier{trig_mc:46000, clr_mc:45500, target_mc:46000, max_ua:   600_000, min_ua:   600_000, ks:350, ki:3,  kc:25},
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
/// Sane readback range for unit validation (100 mA .. 20 A in µA).
const FCC_SANE_MIN_UA: i64 = 100_000;
const FCC_SANE_MAX_UA: i64 = 20_000_000;

// ---- Battery safeguard (called from main.rs) ----
/// Above this battery temp (mC), force FCC to at most BATT_SAFEGUARD_FCC_UA.
const BATT_SAFEGUARD_MC: i64 = 43_000;       // 43°C
const BATT_SAFEGUARD_FCC_UA: i64 = 2_000_000; // 2 A

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

    // Tier state
    tier_level: i32,       // current tier index (-1 = below all)
    tier_floor: i32,       // battery-safeguard floor (-1 = inactive)

    // PID state (only meaningful inside PID tiers 3-5)
    ek_1: i64,
    ek_2: i64,
    output_level: i64,     // 0..PID_OUTPUT_MAX

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

        SicAlgo {
            fcc_fd1, fcc_fd2, thermal_remove_fd, ichg_limit_fd, suspend_fd,
            bypass_state_fd,
            tier_level: -1,
            tier_floor: -1,
            ek_1: 0, ek_2: 0, output_level: 0,
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
        }
    }

    /// The active tier table: custom if loaded, else vendor GLOBAL.
    fn active_tiers(&self) -> &[SicTier] {
        if self.custom_tiers.is_empty() {
            &SIC_TIERS_GLOBAL
        } else {
            &self.custom_tiers
        }
    }

    fn suspend_threshold(&self) -> i64 {
        if !self.custom_tiers.is_empty() && self.custom_suspend_mc > 0 {
            self.custom_suspend_mc
        } else {
            DEFAULT_SUSPEND_MC
        }
    }

    fn resume_threshold(&self) -> i64 {
        if !self.custom_tiers.is_empty() && self.custom_resume_mc > 0 {
            self.custom_resume_mc
        } else {
            DEFAULT_RESUME_MC
        }
    }

    /// Reload custom charging profile from chunked props.
    pub fn maybe_reload_custom(&mut self) {
        let mode = read_prop_str_local(PROP_CHARGING_MODE);
        if mode.as_deref() != Some("custom") {
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
        let seq = read_prop_i64_local(PROP_CHARGING_SEQ).unwrap_or(-1);
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
    pub fn run(&mut self, vsensor0_mc: i64, target_mc: i64, max_fcc_ua: i64, min_fcc_ua: i64) {
        self.run_with_feedforward(vsensor0_mc, target_mc, max_fcc_ua, min_fcc_ua, 0, 0);
    }

    /// Battery safeguard floor — called from main.rs each cycle.
    pub fn update_battery_safeguard(&mut self, battery_temp_mc: i64) {
        let active = self.active_tiers();
        if battery_temp_mc > BATT_SAFEGUARD_MC {
            let floor = active.iter()
                .position(|t| t.max_ua <= BATT_SAFEGUARD_FCC_UA)
                .map(|i| i as i32)
                .unwrap_or((active.len() - 1) as i32);
            if self.tier_floor != floor {
                info!("SIC: battery {:.1}°C → safeguard floor tier {}",
                    battery_temp_mc as f64 / 1000.0, floor);
                self.tier_floor = floor;
            }
        } else if self.tier_floor >= 0 && battery_temp_mc < BATT_SAFEGUARD_MC - 2000 {
            self.tier_floor = -1; // 2°C hysteresis to release
        }
    }

    /// Main per-cycle entry point.
    pub fn run_with_feedforward(
        &mut self,
        vsensor0_mc: i64,
        _target_mc: i64,   // kept for API compat; tiers carry their own setpoints
        max_fcc_ua: i64,
        min_fcc_ua: i64,
        charge_current_ua: i64,
        battery_voltage_uv: i64,
    ) {
        // ---- Phase detection (unchanged) ----
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
        if phase != self.charge_phase {
            info!("SIC: phase {:?} → {:?} (I={}µA V={}µV)",
                self.charge_phase, phase, charge_current_ua, battery_voltage_uv);
            self.charge_phase = phase;
        }

        // ---- Feedforward (unchanged) ----
        if telemetry
            && charge_current_ua - self.prev_charge_current_ua > FEEDFORWARD_CURRENT_DELTA_UA
        {
            self.feedforward_offset = FEEDFORWARD_OFFSET_LEVELS;
            self.feedforward_decay = FEEDFORWARD_DECAY_CYCLES;
            info!("SIC: charger plug (ΔI={}µA) → feedforward +{} levels",
                charge_current_ua - self.prev_charge_current_ua, self.feedforward_offset);
        }
        self.prev_charge_current_ua = charge_current_ua;

        // ---- Tier selection with hysteresis (active table) ----
        let new_tier = compute_tier_level(self.active_tiers(), self.tier_level, vsensor0_mc);
        let effective_tier = new_tier.max(self.tier_floor);

        if effective_tier != self.tier_level {
            info!("SIC: tier {} → {} (VSENSOR0={:.1}°C, table={})",
                self.tier_level, effective_tier, vsensor0_mc as f64 / 1000.0,
                if self.custom_tiers.is_empty() { "GLOBAL" } else { "CUSTOM" });
            self.output_level = 0;
            self.ek_1 = 0;
            self.ek_2 = 0;
            self.tier_level = effective_tier;
        }

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
                self.active_tiers().first().map(|t| t.max_ua).unwrap_or(max_fcc_ua)
            }
            Some(t) => {
                if t.target_mc == 0 || t.max_ua == t.min_ua {
                    t.max_ua // pure cap tier
                } else {
                    let ek = vsensor0_mc - t.target_mc;

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

        // ---- User clamp (GameSpace max/min) ----
        // The min_fcc floor is a safety net for the *vendor* table, whose deepest
        // rung is a fixed 600 mA. A custom profile states its own floor and ceiling on every
        // tier, and applying global min/max FCC silently clamps deep/fast tiers —
        // a 90W profile could then never reach 90W if global max_fcc is lingering at 4.3A.
        let (floor_ua, ceil_ua) = if self.custom_tiers.is_empty() {
            (min_fcc_ua, max_fcc_ua)
        } else {
            (FCC_HARD_MIN_UA, FCC_HARD_MAX_UA)
        };
        let target_fcc = target_fcc.clamp(floor_ua.min(ceil_ua), ceil_ua);

        // ---- Anti-desync write with tolerance + unit validation ----
        // A re-armed escape hatch voids the vote without changing any readback
        // we could otherwise diff against, so treat it as an unconditional
        // re-assert of the whole setpoint rather than just clearing the flag.
        let mut needs_write = self.thermal_limit_removed()
            || (target_fcc - self.applied_fcc).abs() > FCC_WRITE_DEADBAND_UA;
        if !needs_write {
            if let Some(ref fd) = self.fcc_fd1 {
                if let Some(actual) = fd.read_int() {
                    if actual > FCC_SANE_MIN_UA && actual < FCC_SANE_MAX_UA
                        && (actual - target_fcc).abs() > FCC_DESYNC_TOLERANCE_UA
                    {
                        needs_write = true;
                    }
                }
            }
        }

        if needs_write {
            self.write_fcc(target_fcc);
            let tier_label = if effective_tier >= 0 { effective_tier } else { -1 };
            info!("SIC: VSENSOR0={:.1}°C tier={} {:?} ff={} table={} → FCC={} µA ({:.1}A)",
                vsensor0_mc as f64 / 1000.0,
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

    /// True while the kernel holds the pack out of the charge loop.
    /// State ids mirror enum mca_bypass_chg_state: 1 = ACTIVE, 2 = DEFICIT.
    fn bypass_engaged(&self) -> bool {
        let fd = match self.bypass_state_fd {
            Some(ref fd) => fd,
            None => return false,
        };
        let mut buf = [0u8; 32];
        // "<state> <ibat_mA>" — only the leading state field matters here.
        match fd.read_str(&mut buf) {
            Some(s) => match s.split_whitespace().next() {
                Some(tok) => matches!(tok.parse::<i64>(), Ok(1) | Ok(2)),
                None => false,
            },
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
    pub fn check_emergency(&mut self, vsensor0_mc: i64) -> bool {
        let suspend_mc = self.suspend_threshold();
        let resume_mc = self.resume_threshold();

        if self.bypass_engaged() {
            if self.emergency_suspend {
                self.emergency_suspend = false;
                if let Some(ref fd) = self.suspend_fd {
                    fd.write_str("mipower buck 0");
                }
                info!("Emergency released: bypass active, input stays up at {:.1}°C",
                    vsensor0_mc as f64 / 1000.0);
            }
            return false;
        }

        if vsensor0_mc > suspend_mc && !self.emergency_suspend {
            self.emergency_suspend = true;
            if let Some(ref fd) = self.suspend_fd {
                fd.write_str("mipower buck 1");
            }
            info!("EMERGENCY: {:.1}°C > {:.1}°C → charge suspended!",
                vsensor0_mc as f64 / 1000.0, suspend_mc as f64 / 1000.0);
        } else if vsensor0_mc < resume_mc && self.emergency_suspend {
            self.emergency_suspend = false;
            if let Some(ref fd) = self.suspend_fd {
                fd.write_str("mipower buck 0");
            }
            info!("Emergency cleared: {:.1}°C < {:.1}°C → charging resumed",
                vsensor0_mc as f64 / 1000.0, resume_mc as f64 / 1000.0);
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
        if let Some(ref fd) = self.thermal_remove_fd { fd.write_int(0); }
        if let Some(ref fd) = self.fcc_fd1 { fd.write_int(fcc_ua); }
        if let Some(ref fd) = self.fcc_fd2 { fd.write_int(fcc_ua); }
        // mca_charge_interface wants "<user> <type> <value>"; see write_prefixed_int.
        if let Some(ref fd) = self.ichg_limit_fd {
            fd.write_prefixed_int(ICHG_LIMIT_CLIENT, fcc_ua / 1000);
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

fn read_prop_str_local(prop: &str) -> Option<String> {
    match rustutils::system_properties::read(prop) {
        Ok(Some(val)) if !val.trim().is_empty() => Some(val),
        _ => None,
    }
}

fn read_prop_i64_local(prop: &str) -> Option<i64> {
    read_prop_str_local(prop).and_then(|v| v.trim().parse::<i64>().ok())
}

fn read_charging_chunks() -> Option<String> {
    let count = read_prop_i64_local(PROP_CHARGING_COUNT).unwrap_or(0);
    if count <= 0 || count as usize > CHARGING_CHUNK_MAX {
        warn!("SIC: invalid charging chunk count {}", count);
        return None;
    }
    let mut json = String::with_capacity(count as usize * CHARGING_CHUNK_SIZE);
    for i in 0..count {
        let name = format!("{}{}", PROP_CHARGING_CHUNK, i);
        match rustutils::system_properties::read(&name) {
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

fn extract_field_i64(s: &str, key: &str) -> Option<i64> {
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

fn parse_leading_int(s: &str) -> Option<i64> {
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
        let temps = [15_000, 31_800, 33_500, 34_600, 37_000,
                     39_000, 44_500, 45_000, 46_000];
        let mut tier = -1;
        for (i, &t) in temps.iter().enumerate() {
            tier = compute_tier_level(&SIC_TIERS_GLOBAL, tier, t);
            assert_eq!(tier, i as i32, "at {}°C expected tier {}", t / 1000, i);
        }
    }

    #[test]
    fn tier_hysteresis_holds_on_drop() {
        let mut tier = compute_tier_level(&SIC_TIERS_GLOBAL, -1, 39_000);
        assert_eq!(tier, 5);
        tier = compute_tier_level(&SIC_TIERS_GLOBAL, tier, 38_000);
        assert_eq!(tier, 5, "38°C > clr 37.4°C → stay tier 5");
        tier = compute_tier_level(&SIC_TIERS_GLOBAL, tier, 37_400);
        assert_eq!(tier, 5, "37.4°C == clr → stay");
        tier = compute_tier_level(&SIC_TIERS_GLOBAL, tier, 37_399);
        assert_eq!(tier, 4, "37.399°C < clr → drop to tier 4");
    }

    #[test]
    fn tier_full_drop_to_cold() {
        let mut tier = compute_tier_level(&SIC_TIERS_GLOBAL, -1, 46_000);
        assert_eq!(tier, 8);
        for &(t, expect) in &[
            (45_499, 7), (44_499, 6), (43_999, 5), (37_399, 4),
            (35_499, 3), (32_999, 2), (30_799, 1), (24_999, 0),
            (13_999, -1),
        ] {
            tier = compute_tier_level(&SIC_TIERS_GLOBAL, tier, t);
            assert_eq!(tier, expect, "at {}°C", t / 1000);
        }
    }

    fn make_algo() -> SicAlgo {
        SicAlgo::new()
    }

    fn run_cycle(a: &mut SicAlgo, temp: i64) -> i64 {
        a.run_with_feedforward(temp, 38_000, 15_480_000, 860_000, 5_000_000, 4_000_000);
        a.applied_fcc
    }

    #[test]
    fn sic_cold_full_charge() {
        let mut a = make_algo();
        let fcc = run_cycle(&mut a, 30_000);
        assert_eq!(fcc, 15_480_000, "30°C → tier 0 cap (clamped to user max)");
    }

    #[test]
    fn sic_tier1_cap_12_4a() {
        let mut a = make_algo();
        run_cycle(&mut a, 30_000);
        let fcc = run_cycle(&mut a, 32_000);
        assert_eq!(fcc, 12_400_000, "32°C → tier 1 pure cap 12.4A");
    }

    #[test]
    fn sic_tier2_cap_9a() {
        let mut a = make_algo();
        run_cycle(&mut a, 32_000);
        let fcc = run_cycle(&mut a, 34_000);
        assert_eq!(fcc, 9_000_000, "34°C → tier 2 pure cap 9A");
    }

    #[test]
    fn sic_pid_tier4_band() {
        let mut a = make_algo();
        for t in [30_000, 32_000, 34_000, 35_000, 37_000] {
            run_cycle(&mut a, t);
        }
        let mut fcc = 0;
        for _ in 0..30 {
            fcc = run_cycle(&mut a, 38_500);
        }
        assert!(fcc >= 5_000_000 && fcc <= 8_000_000,
            "tier 4 PID FCC {} out of [5A,8A]", fcc);
    }

    #[test]
    fn sic_pure_cap_tier6() {
        let mut a = make_algo();
        for t in [30_000, 35_000, 40_000, 44_500] {
            run_cycle(&mut a, t);
        }
        let fcc = run_cycle(&mut a, 44_500);
        assert_eq!(fcc, 2_000_000, "44.5°C → tier 6 pure cap 2A");
    }

    #[test]
    fn sic_anti_windup_no_overshoot() {
        let mut a = make_algo();
        for t in [30_000, 35_000, 39_000] { run_cycle(&mut a, t); }
        for _ in 0..60 { run_cycle(&mut a, 41_000); }

        // 28°C sits inside tier 1's unusually wide hysteresis band (trig 31.8°C,
        // clr 25.0°C), so the vendor table deliberately holds tier 1 on the way
        // down — it does not fall back to tier 0 until 25°C. Either way the tier
        // *changed*, which is what has to reset the PID.
        for _ in 0..30 { run_cycle(&mut a, 28_000); }
        assert_eq!(a.tier_level, 1);
        assert_eq!(a.output_level, 0, "PID must reset on tier change");

        let mut min_fcc = i64::MAX;
        for t in [32_000, 35_000, 37_000] {
            let fcc = run_cycle(&mut a, t);
            min_fcc = min_fcc.min(fcc);
        }
        for _ in 0..20 {
            let fcc = run_cycle(&mut a, 38_000);
            min_fcc = min_fcc.min(fcc);
        }
        assert!(min_fcc >= 5_000_000,
            "anti-windup failed: FCC dropped to {} (<5A)", min_fcc);
    }

    #[test]
    fn sic_user_clamp_respected() {
        let mut a = make_algo();
        a.run_with_feedforward(30_000, 38_000, 10_000_000, 860_000, 5_000_000, 4_000_000);
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
        a.run_with_feedforward(42_000, 38_000, 15_480_000, 860_000, 5_000_000, 4_000_000);
        assert_eq!(a.applied_fcc, 100_000,
            "custom tier floor must survive the global min_fcc floor");
    }

    #[test]
    fn vendor_table_still_honours_min_fcc_floor() {
        let mut a = SicAlgo::new();
        for t in [30_000, 35_000, 40_000, 44_500, 46_000] {
            a.run_with_feedforward(t, 38_000, 15_480_000, 1_000_000, 5_000_000, 4_000_000);
        }
        assert_eq!(a.applied_fcc, 1_000_000,
            "vendor tier 8 (600 mA) must still be lifted to the 1 A min_fcc floor");
    }

    #[test]
    fn clamp_survives_max_below_min() {
        let mut a = SicAlgo::new();
        // A custom SS profile may hand down a sic_max_fcc under config.min_fcc;
        // clamp() would panic on an inverted range.
        a.run_with_feedforward(30_000, 38_000, 500_000, 860_000, 5_000_000, 4_000_000);
        assert_eq!(a.applied_fcc, 500_000, "ceiling wins when it sits below the floor");
    }

    #[test]
    fn sic_battery_safeguard_floor() {
        let mut a = make_algo();
        a.update_battery_safeguard(44_000);
        assert!(a.tier_floor >= 6, "safeguard must set floor ≥ tier 6");
        for t in [30_000, 35_000, 38_000] { run_cycle(&mut a, t); }
        let fcc = run_cycle(&mut a, 38_000);
        assert!(fcc <= 2_000_000, "battery safeguard must cap FCC ≤ 2A, got {}", fcc);
    }

    #[test]
    fn sic_battery_safeguard_releases() {
        let mut a = make_algo();
        a.update_battery_safeguard(44_000);
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
            a.run_with_feedforward(temp_mc, 38_000, 15_480_000, 860_000,
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
        assert!(t_skin < 45.0, "skin temp {:.1}°C too high", t_skin);
        assert!(t_skin > 30.0, "skin temp {:.1}°C — throttle never engaged?", t_skin);
        let final_a = a.applied_fcc as f64 / 1_000_000.0;
        assert!(final_a >= 2.0 && final_a <= 9.0,
            "final current {:.1}A outside stable band [2,9]A", final_a);
    }

    #[test]
    fn sim_30w_slow_charge_stable() {
        let mut a = make_algo();
        let mut t_skin: f64 = 28.0;
        let (t_amb, c, r, v) = (25.0, 15.0, 0.30, 4.0);
        let mut min_current = f64::MAX;

        for _ in 0..300 {
            let temp_mc = (t_skin * 1000.0) as i64;
            a.run_with_feedforward(temp_mc, 38_000, 15_480_000, 860_000,
                                   3_000_000, 4_000_000);
            let current_a = a.applied_fcc as f64 / 1_000_000.0;
            min_current = min_current.min(current_a);
            let p = v * current_a;
            let pd = (t_skin - t_amb) / r;
            t_skin += (p - pd) / c;
        }

        assert!(min_current >= 5.0, "slow charge dipped to {:.1}A", min_current);
        assert!(t_skin < 40.0, "slow charge overheated to {:.1}°C", t_skin);
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
            a.run_with_feedforward((t*1000.0) as i64, 38000, 15_480_000, 860_000, 5_000_000, 4_000_000);
            let cur = a.applied_fcc as f64 / 1e6;
            if cur < 1.0 { below_1a += 1; }
            let p = 4.2 * cur;
            t += (p - (t - 28.0)/0.35) / 12.0;
        }
        assert_eq!(below_1a, 0, "custom profile must not oscillate below 1A");
    }
}
