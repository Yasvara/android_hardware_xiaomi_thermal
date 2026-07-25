// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Smart Input Current (SIC) charging PID with adaptive coefficients.
//!
//! SOTA improvements over mi_thermald:
//! - Adaptive PID: auto-tunes ks/ki/kc based on thermal inertia
//! - Thermal momentum cooldown: gradual release prevents thermal spikes
//! - Same 9-level FCC table as RE'd from mi_thermald config

use log::info;

use crate::io::SysfsFd;

// ---- SIC Level Table (RE'd from mi_thermald decrypt.txt) ----
// Level 0 = max charging, Level 8 = minimum (trickle)
const SIC_FCC_TABLE: [i64; 9] = [
    15_600_000,  // Level 0: 15.6A (90W)
    10_000_000,  // Level 1: 10.0A (58W)
    5_000_000,   // Level 2: 5.0A  (29W)
    3_000_000,   // Level 3: 3.0A  (18W)
    2_000_000,   // Level 4: 2.0A  (12W)
    1_500_000,   // Level 5: 1.5A  (9W)
    1_200_000,   // Level 6: 1.2A  (7W)
    900_000,     // Level 7: 0.9A  (5W)
    600_000,     // Level 8: 0.6A  (3W) — trickle
];

// ---- PID Coefficients (RE'd from mi_thermald binary) ----
// Velocity PID formula:
//   delta = (ks*(Ek - Ek_1) + ki*Ek + kc*(Ek - 2*Ek_1 + Ek_2)) / 1000
//   output = output_now - delta  (note: subtract! higher output = lower FCC level)
const DEFAULT_KS: i64 = 200;  // Proportional gain
const DEFAULT_KI: i64 = 50;   // Integral gain
const DEFAULT_KC: i64 = 100;  // Derivative gain (damping)
const PID_DIVISOR: i64 = 1000;

// ---- Adaptive PID tuning parameters ----
// Track how long it takes for temp to settle after FCC change
const ADAPTIVE_WINDOW_CYCLES: usize = 30;  // 30 seconds of history
#[allow(dead_code)]
const MIN_KI: i64 = 20;
const MAX_KI: i64 = 150;
#[allow(dead_code)]
const MIN_KC: i64 = 50;
const MAX_KC: i64 = 300;

// ---- Thermal momentum ----
// Tracks integral of (temp - target) over time to prevent oscillation
const MOMENTUM_DECAY: f64 = 0.95;  // Decay factor per cycle
const MOMENTUM_FAST_RELEASE: i32 = 2;   // Steps per cycle when momentum is low
const MOMENTUM_SLOW_RELEASE: i32 = 1;   // Steps per cycle when momentum is high
const MOMENTUM_THRESHOLD: f64 = 5000.0; // mC·s threshold for "high momentum"

// ---- Feedforward (charger plug anticipation, Phase 1.2) ----
// A 90W plug slams input power 0→90W in one cycle; pure feedback must wait for
// VSENSOR0 to actually rise before reacting (30-60s of +3-5°C overshoot).
// We pre-bias the throttle level the moment a large current step is seen.
const FEEDFORWARD_CURRENT_DELTA_UA: i64 = 3_000_000; // >3A step ⇒ HV charger plug
const FEEDFORWARD_OFFSET_LEVELS: i32 = 2;            // pre-throttle by 2 FCC levels
const FEEDFORWARD_DECAY_CYCLES: u32 = 10;            // hold, then let feedback own it
// CC→CV crossover (LPDDR/Li-ion): above this pack voltage the charger tapers
// current on its own, so integral wind-up is counter-productive.
const CV_VOLTAGE_THRESHOLD_UV: i64 = 4_350_000;
// Below this in-flowing current we treat the pack as not charging.
const CHARGING_MIN_CURRENT_UA: i64 = 50_000;

/// Charging phase — selects PID gain schedule.
#[derive(Clone, Copy, PartialEq, Debug)]
enum ChargePhase {
    Discharging, // no charger / trickle — integral disabled
    CC,          // constant-current bulk — full gains
    CV,          // constant-voltage taper — reduced integral, more damping
}

/// SIC charging algorithm state.
pub struct SicAlgo {
    // Sysfs nodes
    fcc_fd1: Option<SysfsFd>,
    fcc_fd2: Option<SysfsFd>,
    suspend_fd: Option<SysfsFd>,

    // PID state
    ek_1: i64,        // Previous error
    ek_2: i64,        // Error two cycles ago
    output_level: i64, // Current FCC level (0-8), scaled by 1000 for precision

    // Adaptive coefficients
    ks: i64,
    ki: i64,
    kc: i64,

    // Applied state
    applied_fcc: i64,
    applied_level: i32,
    emergency_suspend: bool,

    // Thermal momentum (integral of overshoot for smooth release)
    momentum: f64,

    // Adaptive tuning history
    error_history: [i64; ADAPTIVE_WINDOW_CYCLES],
    history_idx: usize,
    history_full: bool,

    // Feedforward state (Phase 1.2)
    prev_charge_current_ua: i64,
    feedforward_offset: i32,
    feedforward_decay_cycles: u32,
    charge_phase: ChargePhase,
}

impl SicAlgo {
    pub fn new() -> Self {
        // Use resolved device paths — class symlinks traverse sysfs:s0 dirs
        // which may lack search permission for mi_thermal_engine context
        let fcc_fd1 = SysfsFd::open_rw(
            "/sys/devices/platform/soc/soc:mca_charger_thermal/wired_chg_curr"
        );
        let fcc_fd2 = SysfsFd::open_rw(
            "/sys/devices/platform/soc/soc:mca_charger_thermal/wired_chg_curr2"
        );
        let suspend_fd = SysfsFd::open_rw(
            "/sys/devices/platform/soc/soc:mca_charge_interface/input_suspend"
        );

        SicAlgo {
            fcc_fd1,
            fcc_fd2,
            suspend_fd,
            ek_1: 0,
            ek_2: 0,
            output_level: 0,
            ks: DEFAULT_KS,
            ki: DEFAULT_KI,
            kc: DEFAULT_KC,
            applied_fcc: -1,
            applied_level: -1,
            emergency_suspend: false,
            momentum: 0.0,
            error_history: [0; ADAPTIVE_WINDOW_CYCLES],
            history_idx: 0,
            history_full: false,
            prev_charge_current_ua: 0,
            feedforward_offset: 0,
            feedforward_decay_cycles: 0,
            charge_phase: ChargePhase::CC,
        }
    }

    /// Run the SIC algorithm (pure-feedback fallback / compatibility shim).
    /// Delegates to `run_with_feedforward` with no battery telemetry, which
    /// reproduces the original behaviour bit-for-bit (CC phase, base gains).
    #[allow(dead_code)]
    pub fn run(&mut self, vsensor0_mc: i64, target_mc: i64, max_fcc_ua: i64, min_fcc_ua: i64) {
        self.run_with_feedforward(vsensor0_mc, target_mc, max_fcc_ua, min_fcc_ua, 0, 0);
    }

    /// Run the SIC algorithm with charger feedforward + phase-aware gains.
    /// `vsensor0_mc`:     current VSENSOR0 temperature (mC)
    /// `target_mc`:       target temperature from config (mC, e.g. 38000)
    /// `max_fcc_ua`:      maximum allowed FCC from user setting (µA)
    /// `min_fcc_ua`:      minimum allowed FCC — SIC never goes below this (µA)
    /// `charge_current_ua`: in-flowing charge current magnitude (µA, ≥0)
    /// `battery_voltage_uv`: pack voltage (µV); ≤0 ⇒ telemetry unavailable
    pub fn run_with_feedforward(
        &mut self,
        vsensor0_mc: i64,
        target_mc: i64,
        max_fcc_ua: i64,
        min_fcc_ua: i64,
        charge_current_ua: i64,
        battery_voltage_uv: i64,
    ) {
        // ---- Phase detection ----
        // voltage ≤ 0 is the "no telemetry" sentinel (missing fd or run() shim):
        // fall back to CC/base gains so behaviour is identical to pure feedback.
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
            info!("SIC: charge phase {:?} → {:?} (I={}µA V={}µV)",
                  self.charge_phase, phase, charge_current_ua, battery_voltage_uv);
            self.charge_phase = phase;
        }

        // ---- Feedforward: detect a high-power charger plug (large current step) ----
        if telemetry
            && charge_current_ua - self.prev_charge_current_ua > FEEDFORWARD_CURRENT_DELTA_UA
        {
            self.feedforward_offset = FEEDFORWARD_OFFSET_LEVELS;
            self.feedforward_decay_cycles = FEEDFORWARD_DECAY_CYCLES;
            info!("SIC: charger plug (ΔI={}µA) → feedforward +{} levels",
                  charge_current_ua - self.prev_charge_current_ua, self.feedforward_offset);
        }
        self.prev_charge_current_ua = charge_current_ua;

        // ---- Phase-aware gains, derived from the adaptive ks/ki/kc ----
        //   CC:          (ks, ki,   kc)    — full authority during bulk charge
        //   CV:          (ks, ki/3, kc*2)  — charger already tapering; damp, don't wind up
        //   Discharging: (ks, 0,    kc)    — no integral when there's no charge power in
        let (eff_ks, eff_ki, eff_kc) = match phase {
            ChargePhase::CC => (self.ks, self.ki, self.kc),
            ChargePhase::CV => (self.ks, self.ki / 3, self.kc * 2),
            ChargePhase::Discharging => (self.ks, 0, self.kc),
        };

        // Error: positive = too hot, negative = too cold
        let ek = vsensor0_mc - target_mc;

        // Update thermal momentum
        if ek > 0 {
            self.momentum += ek as f64;
        } else {
            self.momentum *= MOMENTUM_DECAY;
        }

        // Velocity PID: delta = (ks*(Ek-Ek_1) + ki*Ek + kc*(Ek-2*Ek_1+Ek_2)) / 1000
        let delta = (eff_ks * (ek - self.ek_1)
                   + eff_ki * ek
                   + eff_kc * (ek - 2 * self.ek_1 + self.ek_2))
                   / PID_DIVISOR;

        // Update PID output (higher level = more throttle)
        // When hot (ek > 0): delta > 0 → output increases → FCC decreases
        self.output_level = (self.output_level + delta)
            .max(0)
            .min((SIC_FCC_TABLE.len() as i64 - 1) * 1000);

        // Convert output to FCC level index, then add the feedforward bias
        // BEFORE clamping (pre-emptive throttle on plug; decays back to feedback).
        let base_level = (self.output_level / 1000) as i32;
        let target_level = (base_level + self.feedforward_offset)
            .max(0)
            .min(SIC_FCC_TABLE.len() as i32 - 1) as usize;

        // Apply thermal momentum for smooth release
        let actual_level = if target_level < self.applied_level as usize {
            // Releasing throttle — apply momentum-based rate limiting
            let release_steps = if self.momentum > MOMENTUM_THRESHOLD {
                MOMENTUM_SLOW_RELEASE
            } else {
                MOMENTUM_FAST_RELEASE
            };
            let new_level = (self.applied_level - release_steps).max(target_level as i32);
            new_level as usize
        } else {
            target_level // Throttling up — apply immediately
        };

        // Determine FCC value
        let mut target_fcc = SIC_FCC_TABLE[actual_level];

        // Clamp to user-configured range [min_fcc_ua, max_fcc_ua]
        if target_fcc > max_fcc_ua {
            target_fcc = max_fcc_ua;
        }
        if target_fcc < min_fcc_ua {
            target_fcc = min_fcc_ua;
        }

        // Apply if changed — with anti-desync check
        let mut needs_write = target_fcc != self.applied_fcc;
        if !needs_write {
            // Guard against kernel resetting sysfs (charger plug/PD renegotiation)
            if let Some(ref fd) = self.fcc_fd1 {
                if let Some(actual) = fd.read_int() {
                    if actual != target_fcc {
                        needs_write = true;
                    }
                }
            }
        }

        if needs_write {
            self.write_fcc(target_fcc);
            info!("SIC: VSENSOR0={:.1}°C target={:.1}°C err={}mC {:?} ff={} → level {} FCC={} µA momentum={:.0}",
                  vsensor0_mc as f64 / 1000.0,
                  target_mc as f64 / 1000.0,
                  ek,
                  phase,
                  self.feedforward_offset,
                  actual_level,
                  target_fcc,
                  self.momentum);
            self.applied_fcc = target_fcc;
            self.applied_level = actual_level as i32;
        }

        // Decay the feedforward bias each cycle; once the hold window elapses the
        // feedback PID (which has now seen the real temperature rise) takes over.
        if self.feedforward_decay_cycles > 0 {
            self.feedforward_decay_cycles -= 1;
            if self.feedforward_decay_cycles == 0 {
                self.feedforward_offset = 0;
            }
        }

        // Update error history for adaptive tuning
        self.error_history[self.history_idx] = ek;
        self.history_idx = (self.history_idx + 1) % ADAPTIVE_WINDOW_CYCLES;
        if self.history_idx == 0 {
            self.history_full = true;
        }

        // Run adaptive tuning periodically
        if self.history_full && self.history_idx == 0 {
            self.adapt_pid();
        }

        // Update PID state
        self.ek_2 = self.ek_1;
        self.ek_1 = ek;
    }

    /// Check for emergency conditions.
    /// `vsensor0_mc`: VSENSOR0 temperature (mC)
    /// Returns true if emergency charge suspend is active.
    pub fn check_emergency(&mut self, vsensor0_mc: i64) -> bool {
        const EMERGENCY_SUSPEND_MC: i64 = 48000; // 48°C
        const EMERGENCY_RESUME_MC: i64 = 45000;  // 45°C

        if vsensor0_mc > EMERGENCY_SUSPEND_MC && !self.emergency_suspend {
            self.emergency_suspend = true;
            if let Some(ref fd) = self.suspend_fd {
                fd.write_str("mipower buck 1");
            }
            info!("EMERGENCY: {:.1}°C → charge suspended!", vsensor0_mc as f64 / 1000.0);
        } else if vsensor0_mc < EMERGENCY_RESUME_MC && self.emergency_suspend {
            self.emergency_suspend = false;
            if let Some(ref fd) = self.suspend_fd {
                fd.write_str("mipower buck 0");
            }
            info!("Emergency cleared: {:.1}°C → charging resumed", vsensor0_mc as f64 / 1000.0);
        }

        self.emergency_suspend
    }

    /// Adaptive PID tuning: analyze error history and adjust coefficients.
    fn adapt_pid(&mut self) {
        if !self.history_full {
            return;
        }

        // Count sign changes (oscillation indicator)
        let mut sign_changes = 0;
        for i in 1..ADAPTIVE_WINDOW_CYCLES {
            if (self.error_history[i] > 0) != (self.error_history[i - 1] > 0) {
                sign_changes += 1;
            }
        }

        // Count sustained overshoot (integral error indicator)
        let avg_error: i64 = self.error_history.iter().sum::<i64>()
            / ADAPTIVE_WINDOW_CYCLES as i64;

        // Adapt:
        // High oscillation (>5 sign changes in 30s) → increase kc (damping)
        // Sustained overshoot (avg error > 2°C) → increase ki (integral)
        if sign_changes > 5 {
            self.kc = (self.kc + 20).min(MAX_KC);
            info!("Adaptive PID: oscillation detected ({} changes) → kc={}", sign_changes, self.kc);
        } else if self.kc > DEFAULT_KC {
            self.kc = (self.kc - 5).max(DEFAULT_KC); // Slowly decay back
        }

        if avg_error > 2000 { // 2°C sustained overshoot
            self.ki = (self.ki + 10).min(MAX_KI);
            info!("Adaptive PID: sustained overshoot ({:.1}°C) → ki={}",
                  avg_error as f64 / 1000.0, self.ki);
        } else if avg_error < 500 && self.ki > DEFAULT_KI {
            self.ki = (self.ki - 5).max(DEFAULT_KI); // Slowly decay back
        }
    }

    fn write_fcc(&self, fcc_ua: i64) {
        if let Some(ref fd) = self.fcc_fd1 {
            fd.write_int(fcc_ua);
        }
        if let Some(ref fd) = self.fcc_fd2 {
            fd.write_int(fcc_ua);
        }
    }
}
