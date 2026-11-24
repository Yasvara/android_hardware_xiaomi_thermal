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
//! - Drift-free 1Hz polling on a monotonic deadline
//! - Skin-temperature loop for the default profile, caps lifted while touching
//!
//! Architecture:
//!   GameSpace ThermalController (Kotlin) → system properties → mi_thermal_engine (Rust)
//!   mi_thermal_engine → sysfs → CPU freq limits + charging control + monitors

mod budget;
mod config;
mod interactive;
mod io;
mod monitor;
mod policy;
mod sensor;
mod sic;
#[cfg(test)]
mod sim;
mod skin;
mod ss;

use std::time::{Duration, Instant};
use std::sync::atomic::{AtomicU32, Ordering};

/// Ignore deficits under this magnitude (µA) — measurement noise around 0 A.
const DEFICIT_THRESHOLD_UA: i64 = 50_000;
/// Log a sustained deficit every N cycles (1 Hz), not every cycle.
const DEFICIT_LOG_PERIOD: u32 = 30;
static DEFICIT_TICKS: AtomicU32 = AtomicU32::new(0);

use log::{error, info, warn, LevelFilter};

use budget::ThermalBudget;
use config::Config;
use interactive::Interactive;
use io::{discover_thermal_zones, SysfsFd};
use monitor::MonitorAlgo;
use policy::PolicyPusher;
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

    // ---- Phase 1: Discover thermal zones (lazy-open only active roles) ----
    let thermal_zones = discover_thermal_zones();
    info!("Discovered {} thermal zones", thermal_zones.len());

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
    // Static charging limits (JEITA bands, per-adapter ceilings, derate ladder,
    // bypass tunables). Pushed to the mca driver on profile change only; the
    // per-second FCC vote stays with SicAlgo.
    let mut policy = PolicyPusher::new();

    // Sync initial user CPU/GPU clock ceilings and charging tuning before first tick.
    if config.take_chg_dirty() {
        sensors.apply_tuning(&config.chg);
        sic_algo.apply_tuning(&config.chg);
    }
    if config.take_caps_dirty() {
        ss_algo.set_user_caps(config.user_cpu_caps);
        monitor.set_user_gpu_cap(config.user_gpu_cap_hz);
    }

    // Sync the monitor theme to the SS profile that was active at startup, so
    // the first cycles don't run GLOBAL thresholds under a gaming profile.
    monitor.apply_theme(ss_algo.active_profile_name());

    // Initial sensor read
    sensors.poll();
    info!("Initial VSENSOR0: {:.1}°C, battery: {:.1}°C",
          sensors.vsensor0 as f64 / 1000.0,
          sensors.battery_temp as f64 / 1000.0);

    info!("Thermal engine started — polling at {}s (SS + SIC + monitors)", POLL_INTERVAL_SECS);

    // Screen-state fd for adaptive polling (fd-cached like everything else)
    let screen_fd = SysfsFd::open_ro(SCREEN_STATE_PATH);
    let mut screen_on = true;
    let mut interactive = Interactive::new();
    let mut interval = Duration::from_secs(POLL_INTERVAL_SECS);
    let mut next_tick = Instant::now();
    let mut last_poll = Instant::now();

    // ---- Phase 3: Main loop ----
    // Thermal work runs on a 1 Hz (screen-on) / 3 Hz⁻¹ (screen-off) deadline;
    // only watch the power HAL interaction property when a thermal cap is actually active.
    loop {
        let now = Instant::now();
        if now >= next_tick {
            let dt_s = now.saturating_duration_since(last_poll).as_secs_f64().max(0.1);
            last_poll = now;

            let now_on = screen_fd
                .as_ref()
                .and_then(|fd| fd.read_int())
                .map(|v| v != 0)
                .unwrap_or(true);
            if now_on != screen_on {
                screen_on = now_on;
                let secs = if screen_on { POLL_INTERVAL_SECS } else { POLL_INTERVAL_IDLE_SECS };
                interval = Duration::from_secs(secs);
                info!("Screen {} -> {}s poll interval",
                      if screen_on { "ON" } else { "OFF" }, secs);
            }

            poll_cycle(&mut sensors, &mut sic_algo, &mut monitor, &mut ss_algo,
                       &mut config, &mut budget, &mut policy, screen_on, dt_s);

            next_tick += interval;
            if next_tick <= now {
                next_tick = now + interval; // fell behind (suspend); don't burst
            }
        }

        let allowed = screen_on && config.enabled
            && ss_algo.lifts_on_interaction() && monitor.boost_allowed()
            && (ss_algo.capped() || monitor.gpu_capped());
        let lift = interactive.update(allowed, Instant::now());
        ss_algo.set_lift(lift);
        monitor.set_lift(lift);

        let deadline = interactive.exhausted_at().map_or(next_tick, |t| t.min(next_tick));
        if allowed {
            interactive.wait(deadline);
        } else {
            interactive.sleep_until(deadline);
        }
    }
}

/// Execute one polling cycle — the hot path.
/// ZERO heap allocations in steady state.
#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn poll_cycle(
    sensors: &mut SensorPool,
    sic: &mut SicAlgo,
    monitor: &mut MonitorAlgo,
    ss: &mut SsAlgo,
    config: &mut Config,
    budget: &mut ThermalBudget,
    policy: &mut PolicyPusher,
    screen_on: bool,
    dt_s: f64,
) {
    // Watch user CPU/GPU caps (0-alloc every cycle) and reload slow config every 5 cycles.
    config.maybe_reload();

    if config.take_chg_dirty() {
        sensors.apply_tuning(&config.chg);
        sic.apply_tuning(&config.chg);
    }
    if config.take_caps_dirty() {
        ss.set_user_caps(config.user_cpu_caps);
        monitor.set_user_gpu_cap(config.user_gpu_cap_hz);
    }

    // Reload custom charging profile if mode="custom" and seq changed (0-alloc when unchanged).
    sic.maybe_reload_custom();
    policy.maybe_reload();

    // Skip all actions if disabled (but still poll sensors for monitoring)
    let enabled = config.enabled;

    // Read all active sensors with true elapsed dt_s.
    sensors.poll_dt(dt_s);

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
    } else if predicted_vsensor0 > EMERGENCY_SHUTDOWN_MC {
        warn!("PREDICTIVE: VSENSOR0={:.1}°C trending to {:.1}°C within {:.0}s",
              vsensor0 as f64 / 1000.0,
              predicted_vsensor0 as f64 / 1000.0,
              PREDICT_WARN_HORIZON_S);
    }

    let throttle_temp = (vsensor0 - config.temp_offset_mc).max(0);

    ss.maybe_switch_profile();
    if ss.take_config_dirty() {
        if let Some(extras) = ss.custom_extras() {
            monitor.apply_custom_theme(extras);
        } else {
            let theme_key = if ss.is_no_limit() { "performance" } else { ss.active_profile_name() };
            monitor.apply_theme(theme_key);
        }
    }

    let rate = sensors.vsensor_fused_filter.rate();
    ss.run_predictive(throttle_temp, rate);

    let (sic_target, sic_max_fcc) = match ss.custom_extras() {
        Some(e) => (
            if e.sic_target_mc > 0 { e.sic_target_mc } else { config.temp_limit_mc },
            if e.sic_max_fcc_ua > 0 { e.sic_max_fcc_ua } else { config.max_fcc_ua },
        ),
        None => (config.temp_limit_mc, config.max_fcc_ua),
    };

    sic.update_battery_safeguard(sensors.cell_temp);
    sic.run_with_feedforward_dt(
        sensors.charge_temp,
        sensors.cell_temp,
        sensors.charger_therm,
        sic_target,
        sic_max_fcc,
        config.min_fcc_ua,
        sensors.charge_current_ua(),
        sensors.battery_voltage_uv,
        dt_s,
    );
    sic.check_emergency(
        sensors.cell_temp, sensors.charger_therm, sensors.charge_current_ua());

    let net_ua = sensors.net_current_ua();
    if net_ua < -DEFICIT_THRESHOLD_UA && sensors.battery_voltage_uv > 0 {
        let n = DEFICIT_TICKS.fetch_add(1, Ordering::Relaxed);
        if n % DEFICIT_LOG_PERIOD == 0 {
            warn!("Charging deficit: pack draining {:.2}A ({:.1}W) at cell {:.1}°C \
                   for {}s — input path is not carrying the load",
                -net_ua as f64 / 1_000_000.0,
                -net_ua as f64 * sensors.battery_voltage_uv as f64 / 1e12,
                sensors.cell_temp as f64 / 1000.0,
                n + 1);
        }
    } else {
        DEFICIT_TICKS.store(0, Ordering::Relaxed);
    }

    // Screen-on only: sample CPU/GPU utilisation and drive demand-aware GPU skin scaling.
    monitor.set_skin_output(ss.skin_output(), ss.skin_gpu_floor_hz());
    if screen_on {
        if ss.skin_output().is_some() {
            budget.update(vsensor0, ss.skin_target_mc());
            monitor.set_gpu_share(budget.gpu_share());
        }
        monitor.run(throttle_temp);
    }
}

/// Compile time string for logging.
fn compile_time() -> &'static str {
    // This is filled at compile time
    "2026"
}
