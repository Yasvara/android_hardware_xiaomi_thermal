// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! MiThermalEngine — Thermal management daemon for POCO F7 (onyx).
//!
//! Open-source replacement for Xiaomi's proprietary mi_thermald.
//! Handles:
//! - SS (Step-wise Scaling) CPU frequency throttling
//! - SIC (Smart Intelligent Charging) PID control
//! - Auxiliary monitors (GPU, modem, hotplug, brightness)
//!
//! SOTA features:
//! - Kalman filter for noise-robust temperature estimation
//! - Adaptive SIC PID with auto-tuning coefficients
//! - SS algo with decrypted vendor profiles + custom profile support
//! - Zero-allocation hot path with pre-opened fd pool
//! - timerfd-based precise 1Hz polling (no drift)
//!
//! Architecture:
//!   GameSpace ThermalController (Kotlin) → system properties → mi_thermal_engine (Rust)
//!   mi_thermal_engine → sysfs → CPU freq limits + charging control + monitors

mod budget;
mod config;
mod io;
mod monitor;
mod sensor;
mod sic;
mod ss;

use std::time::Duration;

use log::{error, info, warn, LevelFilter};
use nix::sys::timerfd::{ClockId, Expiration, TimerFd, TimerFlags, TimerSetTimeFlags};
use nix::sys::time::TimeSpec;

use budget::ThermalBudget;
use config::Config;
use io::{open_thermal_zones, SysfsFd};
use monitor::MonitorAlgo;
use sensor::SensorPool;
use sic::SicAlgo;
use ss::SsAlgo;


const TAG: &str = "MiThermal";

/// Polling interval while the screen is on (seconds).
const POLL_INTERVAL_SECS: u64 = 1;

/// Relaxed polling interval while the screen is off (seconds).
/// SS + SIC still run (charging control matters most while idle-charging);
/// only the auxiliary monitors are skipped. The vendor mi_thermald/joyose
/// stack keeps its full-rate loop with the display off — this doesn't.
const POLL_INTERVAL_IDLE_SECS: u64 = 3;

/// Screen state node — written by GameSpace ThermalController on
/// ACTION_SCREEN_ON/OFF ("1" = on). Missing node ⇒ assume on.
const SCREEN_STATE_PATH: &str = "/sys/class/thermal/thermal_message/screen_state";

/// Emergency shutdown protection threshold (mC).
const EMERGENCY_SHUTDOWN_MC: i64 = 55000; // 55°C VSENSOR0

/// Look-ahead (seconds) for the Kalman predictive emergency pre-warning.
const PREDICT_WARN_HORIZON_S: f64 = 4.0;

fn main() {
    // Initialize Android logger
    android_logger::init_once(
        android_logger::Config::default()
            .with_tag(TAG)
            .with_max_level(LevelFilter::Info),
    );

    info!("MiThermalEngine starting — Rust SOTA thermal daemon");
    info!("Built: {}", compile_time());

    // ---- Phase 1: Initialize I/O pool ----
    let thermal_zones = open_thermal_zones();
    info!("Opened {} thermal zones", thermal_zones.len());

    if thermal_zones.is_empty() {
        error!("No thermal zones found! Exiting.");
        std::process::exit(1);
    }

    // ---- Phase 2: Initialize subsystems ----
    let mut sensors = SensorPool::new(thermal_zones);

    let mut sic_algo = SicAlgo::new();
    let mut monitor = MonitorAlgo::new();
    let mut ss_algo = SsAlgo::new();
    let mut config = Config::new();
    let mut budget = ThermalBudget::new();

    // Sync the monitor theme to the SS profile that was active at startup, so
    // the first cycles don't run GLOBAL thresholds under a gaming profile.
    monitor.apply_theme(ss_algo.active_profile_name());

    // Initial sensor read
    sensors.poll();
    info!("Initial VSENSOR0: {:.1}°C, battery: {:.1}°C",
          sensors.vsensor0 as f64 / 1000.0,
          sensors.battery_temp as f64 / 1000.0);

    // ---- Phase 3: Create timerfd for precise 1Hz polling ----
    let timer_fd = match TimerFd::new(ClockId::CLOCK_MONOTONIC, TimerFlags::empty()) {
        Ok(fd) => fd,
        Err(e) => {
            error!("Failed to create timerfd: {} — falling back to sleep", e);
            // Fallback: run with std::thread::sleep
            run_with_sleep(&mut sensors, &mut sic_algo, &mut monitor, &mut ss_algo,
                           &mut config, &mut budget);
            return;
        }
    };

    // Set recurring 1-second timer
    let interval = TimeSpec::from_duration(Duration::from_secs(POLL_INTERVAL_SECS));
    if let Err(e) = timer_fd.set(
        Expiration::Interval(interval),
        TimerSetTimeFlags::empty(),
    ) {
        error!("Failed to set timerfd interval: {}", e);
        run_with_sleep(&mut sensors, &mut sic_algo, &mut monitor, &mut ss_algo,
                       &mut config, &mut budget);
        return;
    }

    info!("Thermal engine started — polling at {}s via timerfd (SS + SIC + monitors)", POLL_INTERVAL_SECS);

    // Screen-state fd for adaptive polling (fd-cached like everything else)
    let screen_fd = SysfsFd::open_ro(SCREEN_STATE_PATH);
    let mut screen_on = true;

    // ---- Phase 4: Main loop ----
    loop {
        // Block until timer fires (exact interval, no drift)
        match timer_fd.wait() {
            Ok(_) => {}
            Err(nix::errno::Errno::EINTR) => continue, // Signal interrupted
            Err(e) => {
                error!("timerfd wait error: {}", e);
                std::thread::sleep(Duration::from_secs(1));
            }
        }

        let now_on = screen_fd
            .as_ref()
            .and_then(|fd| fd.read_int())
            .map(|v| v != 0)
            .unwrap_or(true);

        if now_on != screen_on {
            screen_on = now_on;
            let secs = if screen_on { POLL_INTERVAL_SECS } else { POLL_INTERVAL_IDLE_SECS };
            let interval = TimeSpec::from_duration(Duration::from_secs(secs));
            if let Err(e) = timer_fd.set(
                Expiration::Interval(interval),
                TimerSetTimeFlags::empty(),
            ) {
                error!("Failed to re-arm timerfd: {}", e);
            }
            info!("Screen {} -> {}s poll interval",
                  if screen_on { "ON" } else { "OFF" }, secs);
        }

        poll_cycle(&mut sensors, &mut sic_algo, &mut monitor, &mut ss_algo,
                   &mut config, &mut budget, screen_on);
    }
}

/// Fallback main loop using std::thread::sleep (if timerfd fails).
fn run_with_sleep(
    sensors: &mut SensorPool,
    sic: &mut SicAlgo,
    monitor: &mut MonitorAlgo,
    ss: &mut SsAlgo,
    config: &mut Config,
    budget: &mut ThermalBudget,
) {
    warn!("Running with sleep fallback (timerfd unavailable)");
    let screen_fd = SysfsFd::open_ro(SCREEN_STATE_PATH);
    loop {
        let screen_on = screen_fd
            .as_ref()
            .and_then(|fd| fd.read_int())
            .map(|v| v != 0)
            .unwrap_or(true);
        let secs = if screen_on { POLL_INTERVAL_SECS } else { POLL_INTERVAL_IDLE_SECS };
        std::thread::sleep(Duration::from_secs(secs));
        poll_cycle(sensors, sic, monitor, ss, config, budget, screen_on);
    }
}

/// Execute one polling cycle — the hot path.
/// ZERO heap allocations in this function (except profile switch which is rare).
#[inline(never)] // Prevent inlining for profiling
// This is the single per-cycle coordinator for every daemon subsystem; grouping
// them behind a struct would only move the same references around.
#[allow(clippy::too_many_arguments)]
fn poll_cycle(
    sensors: &mut SensorPool,
    sic: &mut SicAlgo,
    monitor: &mut MonitorAlgo,
    ss: &mut SsAlgo,
    config: &mut Config,
    budget: &mut ThermalBudget,
    screen_on: bool,
) {
    // Reload config from system properties (every 5 cycles)
    config.maybe_reload();

    // Reload custom charging profile if mode="custom" and seq changed
    sic.maybe_reload_custom();

    // Skip all actions if disabled (but still poll sensors for monitoring)
    let enabled = config.enabled;

    // Read all sensors: VSENSOR0 (Kalman-fused) + battery power telemetry
    sensors.poll();

    if !enabled {
        return;
    }

    let vsensor0 = sensors.vsensor0;

    // Emergency shutdown protection, with a Kalman look-ahead (predict()) that
    // pre-warns while there is still headroom to react before the threshold.
    let predicted_vsensor0 = sensors.vsensor_fused_filter.predict(PREDICT_WARN_HORIZON_S);
    if vsensor0 > EMERGENCY_SHUTDOWN_MC {
        error!("CRITICAL: VSENSOR0={:.1}°C exceeds shutdown threshold!",
               vsensor0 as f64 / 1000.0);
        // Kernel will handle actual shutdown via thermal_core
    } else if predicted_vsensor0 > EMERGENCY_SHUTDOWN_MC {
        warn!("PREDICTIVE: VSENSOR0={:.1}°C trending to {:.1}°C within {:.0}s",
              vsensor0 as f64 / 1000.0,
              predicted_vsensor0 as f64 / 1000.0,
              PREDICT_WARN_HORIZON_S);
    }

    // Feature B (aggressiveness): subtract the user's headroom offset from the
    // sensor value the throttle logic sees. Positive offset = perceived cooler =
    // throttles later. Emergency/SIC/budget still use the RAW vsensor0.
    let throttle_temp = (vsensor0 - config.temp_offset_mc).max(0);

    // Manage profile + sync the monitor theme BEFORE applying CPU levels, so SS
    // and the monitor never disagree within a cycle. maybe_switch_profile() only
    // touches the heap on an actual change; take_config_dirty() fires once after
    // a built-in switch OR a custom (re)load.
    ss.maybe_switch_profile();
    if ss.take_config_dirty() {
        if let Some(extras) = ss.custom_extras() {
            // Custom profile with extended GPU/monitor knobs → dynamic theme
            // (Feature A GPU curve, Feature C boost/hotplug/brightness).
            monitor.apply_custom_theme(extras);
        } else {
            // Built-in, or a CPU-only custom profile. A no-limit SS profile must
            // also relax the monitor (NOLIMITS via the "performance" key).
            let theme_key = if ss.is_no_limit() { "performance" } else { ss.active_profile_name() };
            monitor.apply_theme(theme_key);
        }
    }

    // Phase 1.1: predictive SS — Kalman dT/dt look-ahead, evaluated on VSENSOR0
    // (minus the aggressiveness offset).
    let rate = sensors.vsensor_fused_filter.rate();
    ss.run_predictive(throttle_temp, rate);

    // Phase 1.2 + Feature D: SIC charging PID. A custom profile may override the
    // skin-temp target and max FCC; otherwise the global config values are used.
    let (sic_target, sic_max_fcc) = match ss.custom_extras() {
        Some(e) => (
            if e.sic_target_mc > 0 { e.sic_target_mc } else { config.temp_limit_mc },
            if e.sic_max_fcc_ua > 0 { e.sic_max_fcc_ua } else { config.max_fcc_ua },
        ),
        None => (config.temp_limit_mc, config.max_fcc_ua),
    };

    // Battery safeguard: PHYSICAL NTC, not fuel gauge.
    sic.update_battery_safeguard(sensors.batt_ntc);

    // Charging PID: charge fusion + NTC + pump governor.
    sic.run_with_feedforward(
        sensors.charge_temp,
        sensors.batt_ntc,
        sensors.charger_therm,
        sic_target,
        sic_max_fcc,
        config.min_fcc_ua,
        sensors.charge_current_ua(),
        sensors.battery_voltage_uv,
    );
    sic.check_emergency(sensors.batt_ntc);

    // Phase 3.1: advisory thermal-budget telemetry (computes cpu/gpu fraction
    // split; does not actuate — SS and GPU DVFS own the actuators).
    budget.update(vsensor0, config.temp_limit_mc);

    // Run monitor algo (GPU DVFS, modem, NPU, hotplug, boost, brightness, DDR).
    // Uses the offset-adjusted temp so the aggressiveness knob relaxes GPU/boost
    // /hotplug/brightness in lockstep with the CPU. Pointless with display off.
    if screen_on {
        monitor.run(throttle_temp);
    }
}

/// Compile time string for logging.
fn compile_time() -> &'static str {
    // This is filled at compile time
    "2026"
}
