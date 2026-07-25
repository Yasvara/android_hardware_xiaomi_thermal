// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Monitor algo for GPU, modem, NPU, CPU hotplug, backlight, and boost.
//!
//! Uses the same threshold tables as RE'd from mi_thermald config.
//! SOTA improvement: Smart cooldown — gradual release prevents thermal spikes.
//!
//! Profile-aware: monitor thresholds switch when thermal profile changes.
//! Built-in themes: GLOBAL, MGAME, NOLIMITS. Custom profiles can override.

use log::{info, warn};

use crate::io::SysfsFd;
use crate::ss::{CustomExtras, CUSTOM_GPU_MAX};

// ================================================================
// Monitor Theme — profile-aware thresholds
// ================================================================

/// Maximum GPU monitor levels.
const MAX_GPU_LEVELS: usize = 4;

/// Monitor thresholds that vary per thermal profile.
/// `Copy` so the active theme can be stored by value in `MonitorAlgo` and a
/// dynamic per-custom-profile theme can be built at runtime.
#[derive(Clone, Copy)]
pub struct MonitorTheme {
    pub name: &'static str,

    // GPU throttle
    pub gpu_trig: [i64; MAX_GPU_LEVELS],
    pub gpu_clr: [i64; MAX_GPU_LEVELS],
    pub gpu_frac: [f64; MAX_GPU_LEVELS],
    pub gpu_level_count: usize,

    // Boost disable
    pub boost_trig: i64,
    pub boost_clr: i64,

    // Hotplug
    pub hotplug_trig: i64,
    pub hotplug_clr: i64,

    // Backlight cap
    pub bl_trig: i64,
    pub bl_clr: i64,
    pub bl_cap: i64,
}

/// GLOBAL theme (default) — 3-level GPU, standard boost/hotplug/backlight
const THEME_GLOBAL: MonitorTheme = MonitorTheme {
    name: "GLOBAL",
    gpu_trig: [15000, 40000, 45000, 0],
    gpu_clr:  [10000, 38000, 43000, 0],
    gpu_frac: [0.0,   0.5,   1.0,   0.0],
    gpu_level_count: 3,
    boost_trig: 48000,
    boost_clr:  46000,
    hotplug_trig: 50000,
    hotplug_clr:  48000,
    bl_trig: 51000,
    bl_clr:  49000,
    bl_cap:  160,
};

/// MGAME theme — relaxed GPU (single level at low), higher boost/hotplug thresholds
const THEME_MGAME: MonitorTheme = MonitorTheme {
    name: "MGAME",
    gpu_trig: [15000, 0, 0, 0],
    gpu_clr:  [13000, 0, 0, 0],
    gpu_frac: [0.3,   0.0, 0.0, 0.0],  // 30% of max (light throttle)
    gpu_level_count: 1,
    boost_trig: 50000,  // Higher threshold — let game boost stay longer
    boost_clr:  48000,
    hotplug_trig: 52000, // Don't park cores during gaming
    hotplug_clr:  50000,
    bl_trig: 51000,
    bl_clr:  49000,
    bl_cap:  160,
};

/// NOLIMITS theme — no GPU throttle, no boost limit, no hotplug, no BL cap
const THEME_NOLIMITS: MonitorTheme = MonitorTheme {
    name: "NOLIMITS",
    gpu_trig: [0, 0, 0, 0],
    gpu_clr:  [0, 0, 0, 0],
    gpu_frac: [0.0, 0.0, 0.0, 0.0],
    gpu_level_count: 0,
    boost_trig: 55000,  // Only at emergency
    boost_clr:  53000,
    hotplug_trig: 55000,
    hotplug_clr:  53000,
    bl_trig: 55000,
    bl_clr:  53000,
    bl_cap:  160,
};

/// Battery Saver theme — aggressive GPU throttle, early boost disable
const THEME_BATTERY: MonitorTheme = MonitorTheme {
    name: "BATTERY",
    gpu_trig: [15000, 35000, 40000, 45000],
    gpu_clr:  [10000, 33000, 38000, 43000],
    gpu_frac: [0.0,   0.3,   0.6,   1.0],
    gpu_level_count: 4,
    boost_trig: 42000,  // Disable boost early to save battery
    boost_clr:  40000,
    hotplug_trig: 45000,
    hotplug_clr:  43000,
    bl_trig: 45000,
    bl_clr:  43000,
    bl_cap:  128,  // Lower brightness cap
};

/// Map SS profile name → monitor theme.
pub fn get_monitor_theme(profile_name: &str) -> &'static MonitorTheme {
    match profile_name {
        "performance" => &THEME_NOLIMITS,
        "gaming" | "heavy_gaming" | "genshin" | "hsr" | "high_fps" => &THEME_MGAME,
        "battery_saver" => &THEME_BATTERY,
        _ => &THEME_GLOBAL,
    }
}

// ---- Modem 5G→4G (GLOBAL-MONITOR-MODEM-5GTO4G) ----
// Modem thresholds are not profile-dependent (always GLOBAL)
const MON_MODEM5G_TRIG: i64 = 47000; // mC
const MON_MODEM5G_CLR: i64  = 45000; // mC

// ---- Modem Power Levels (GLOBAL-MONITOR-MODEM-LEVEL) ----
const MON_MODEM_TRIG: [i64; 4] = [37000, 40000, 44000, 47000]; // mC
const MON_MODEM_CLR: [i64; 4]  = [35000, 38000, 42000, 45000]; // mC

// ---- NPU/CDSP Throttle ----
// cooling_device type="cdsp", max_state=8
const MON_NPU_TRIG: [i64; 6] = [25000, 35000, 39000, 43000, 47000, 50000]; // mC
const MON_NPU_CLR: [i64; 6]  = [23000, 33000, 37000, 41000, 45000, 48000]; // mC
const MON_NPU_CDEV: [i64; 6] = [0, 1, 2, 3, 5, 8];

// ================================================================
// GPU direct DVFS (Phase 2.2) — Adreno 830 via kgsl devfreq
// ================================================================
// cooling_device/cur_state gives only max_state+1 coarse steps; the Adreno 830
// exposes 11 discrete frequencies (verified on onyx: 1162→150 MHz). Capping
// devfreq/max_freq snaps to any of them for far finer thermal control.
// kgsl nodes are labelled vendor_sysfs_kgsl, which mi_thermal_engine.te already
// grants rw — so this is the primary path, with cooling_device as fallback.
const GPU_AVAIL_FREQ_PATH: &str = "/sys/class/kgsl/kgsl-3d0/gpu_available_frequencies";
const GPU_DEVFREQ_MAX_PATH: &str = "/sys/class/kgsl/kgsl-3d0/devfreq/max_freq";

/// Direct GPU DVFS control via kgsl devfreq/max_freq.
struct GpuDvfs {
    devfreq_max_fd: Option<SysfsFd>,
    /// Available GPU frequencies (Hz), sorted ascending. Empty ⇒ unavailable.
    available_freqs: Vec<i64>,
    hw_max_hz: i64,
    last_written_hz: i64,
}

impl GpuDvfs {
    fn new() -> Self {
        // Parse the space-separated available-frequency list (init only).
        let mut available_freqs: Vec<i64> = Vec::new();
        if let Ok(s) = std::fs::read_to_string(GPU_AVAIL_FREQ_PATH) {
            for tok in s.split_whitespace() {
                if let Ok(hz) = tok.parse::<i64>() {
                    if hz > 0 {
                        available_freqs.push(hz);
                    }
                }
            }
        }
        available_freqs.sort_unstable();
        available_freqs.dedup();
        let hw_max_hz = available_freqs.last().copied().unwrap_or(0);

        let devfreq_max_fd = SysfsFd::open_rw(GPU_DEVFREQ_MAX_PATH);

        let available = !available_freqs.is_empty() && devfreq_max_fd.is_some();
        if available {
            info!("GPU DVFS: {} steps, hw_max={}Hz (devfreq/max_freq)",
                  available_freqs.len(), hw_max_hz);
        } else {
            warn!("GPU DVFS unavailable (freqs={}, fd={}) — using cooling_device fallback",
                  available_freqs.len(), devfreq_max_fd.is_some());
        }

        GpuDvfs {
            devfreq_max_fd,
            available_freqs,
            hw_max_hz,
            last_written_hz: -1,
        }
    }

    /// True when direct DVFS can be used; false ⇒ caller must use cooling_device.
    fn is_available(&self) -> bool {
        !self.available_freqs.is_empty() && self.devfreq_max_fd.is_some()
    }

    /// Cap the GPU to `fraction` (0.0..1.0) of hw_max, snapping to the nearest
    /// available frequency. 1.0 = unrestricted. ZERO heap allocation.
    fn set_fraction(&mut self, fraction: f64) {
        if self.available_freqs.is_empty() {
            return;
        }
        let f = if fraction.is_finite() { fraction.clamp(0.0, 1.0) } else { 1.0 };
        let target_hz = (self.hw_max_hz as f64 * f) as i64;

        // Snap to nearest available frequency (never below the lowest step).
        let mut best = self.available_freqs[0];
        let mut best_d = (best - target_hz).abs();
        for &cand in &self.available_freqs {
            let d = (cand - target_hz).abs();
            if d < best_d {
                best_d = d;
                best = cand;
            }
        }

        if best != self.last_written_hz {
            if let Some(ref fd) = self.devfreq_max_fd {
                fd.write_int(best);
            }
            self.last_written_hz = best;
        }
    }
}

// ================================================================
// DDR thermal monitor (Phase 2.3) — LPDDR5X
// ================================================================
const DDR_WARN_MC: i64 = 90000;      // 90°C warning
const DDR_CRITICAL_MC: i64 = 95000;  // 95°C LPDDR5X throttle threshold
const DDR_HYST_MC: i64 = 2000;       // 2°C hysteresis for log de-spam

/// Passive DDR temperature monitor. Currently observe-and-warn only; a DDR
/// cooling device (if one is ever exposed) would be actuated here.
struct DdrMonitor {
    zone_fd: Option<SysfsFd>,
    warned: bool,
    critical: bool,
}

impl DdrMonitor {
    fn new() -> Self {
        DdrMonitor {
            zone_fd: find_ddr_zone_fd(),
            warned: false,
            critical: false,
        }
    }

    fn check(&mut self) {
        let temp = match self.zone_fd.as_ref().and_then(|f| f.read_int()) {
            Some(t) => t,
            None => return,
        };

        if temp >= DDR_CRITICAL_MC {
            if !self.critical {
                warn!("DDR CRITICAL: {:.1}°C ≥ LPDDR5X throttle threshold!",
                      temp as f64 / 1000.0);
                self.critical = true;
            }
        } else if temp < DDR_CRITICAL_MC - DDR_HYST_MC {
            self.critical = false;
        }

        if temp >= DDR_WARN_MC {
            if !self.warned {
                info!("DDR warning: {:.1}°C", temp as f64 / 1000.0);
                self.warned = true;
            }
        } else if temp < DDR_WARN_MC - DDR_HYST_MC {
            self.warned = false;
        }
    }
}

/// Find the DDR/DRAM thermal zone by `type` substring (init only).
fn find_ddr_zone_fd() -> Option<SysfsFd> {
    use std::fs::File;
    for i in 0..128 {
        let type_path = format!("/sys/class/thermal/thermal_zone{}/type", i);
        let f = match File::open(&type_path) {
            Ok(f) => f,
            Err(_) => break,
        };
        let mut buf = [0u8; 64];
        let name = match nix::sys::uio::pread(&f, &mut buf, 0) {
            Ok(n) if n > 0 => String::from_utf8_lossy(&buf[..n]).trim().to_lowercase(),
            _ => continue,
        };
        if name.contains("ddr") || name.contains("dram") || name.contains("mem") {
            let p = format!("/sys/class/thermal/thermal_zone{}/temp", i);
            let path: &'static str = Box::leak(p.into_boxed_str());
            info!("DDR monitor: zone '{}' at thermal_zone{}", name, i);
            return SysfsFd::open_ro(path);
        }
    }
    warn!("DDR monitor: no ddr/dram/mem thermal zone found");
    None
}

/// Monitor algo state.
pub struct MonitorAlgo {
    // Sysfs fds
    gpu_cdev_fd: Option<SysfsFd>,
    gpu_max_state: i64,
    gpu_dvfs: GpuDvfs,
    ddr_monitor: DdrMonitor,
    npu_cdev_fd: Option<SysfsFd>,
    hotplug_fds: Vec<SysfsFd>,
    modem_limit_fd: Option<SysfsFd>,
    modem_level_fd: Option<SysfsFd>,
    boost_fd: Option<SysfsFd>,
    brightness_fd: Option<SysfsFd>,

    // Current states
    gpu_level: i32,
    npu_level: i32,
    modem_5g_limited: bool,
    modem_level: i32,
    hotplug_active: bool,
    boost_disabled: bool,
    brightness_capped: bool,

    // Active monitor theme (owned by value so custom profiles can supply a
    // dynamically-built theme).
    theme: MonitorTheme,
}

/// Find a cooling device by type name and open its cur_state for rw.
/// Returns (SysfsFd, max_state) or None.
fn find_cdev(type_name: &str) -> Option<(SysfsFd, i64)> {
    use std::fs;
    for i in 0..128 {
        let type_path = format!("/sys/class/thermal/cooling_device{}/type", i);
        if let Ok(t) = fs::read_to_string(&type_path) {
            if t.trim() == type_name {
                let max_path = format!("/sys/class/thermal/cooling_device{}/max_state", i);
                let max_state = fs::read_to_string(&max_path)
                    .ok()
                    .and_then(|s| s.trim().parse::<i64>().ok())
                    .unwrap_or(0);
                let cur_path_str = format!("/sys/class/thermal/cooling_device{}/cur_state", i);
                let cur_path: &'static str = Box::leak(cur_path_str.into_boxed_str());
                if let Some(fd) = SysfsFd::open_rw(cur_path) {
                    info!("Found cdev '{}' at cooling_device{} (max_state={})",
                          type_name, i, max_state);
                    return Some((fd, max_state));
                }
            }
        }
    }
    None
}

/// Find a hotplug cooling device by type name.
fn find_hotplug_cdev(type_name: &str) -> Option<SysfsFd> {
    find_cdev(type_name).map(|(fd, _)| fd)
}

impl MonitorAlgo {
    pub fn new() -> Self {
        // Discover cooling devices by type name — handles OC'd device numbering
        let (gpu_cdev_fd, gpu_max_state) = find_cdev("gpu")
            .map(|(fd, ms)| (Some(fd), ms))
            .unwrap_or((None, 0));

        let (npu_cdev_fd, _) = find_cdev("cdsp")
            .map(|(fd, ms)| (Some(fd), ms))
            .unwrap_or((None, 0));

        // Hotplug: park big/prime cores under extreme heat
        let hotplug_names = ["cpu-hotplug5", "cpu-hotplug6", "cpu-hotplug7"];
        let hotplug_fds: Vec<SysfsFd> = hotplug_names.iter()
            .filter_map(|name| find_hotplug_cdev(name))
            .collect();

        let modem_limit_fd = SysfsFd::open_rw(
            "/sys/devices/virtual/thermal/thermal_message/modem_limit"
        );
        let modem_level_fd = SysfsFd::open_rw(
            "/sys/devices/virtual/thermal/thermal_message/modem_level"
        );
        let boost_fd = SysfsFd::open_rw(
            "/sys/devices/virtual/thermal/thermal_message/boost"
        );
        let brightness_fd = SysfsFd::open_rw(
            "/sys/devices/virtual/thermal/thermal_message/thermal_max_brightness"
        );

        MonitorAlgo {
            gpu_cdev_fd,
            gpu_max_state,
            gpu_dvfs: GpuDvfs::new(),
            ddr_monitor: DdrMonitor::new(),
            npu_cdev_fd,
            hotplug_fds,
            modem_limit_fd,
            modem_level_fd,
            boost_fd,
            brightness_fd,
            gpu_level: -1,
            npu_level: -1,
            modem_5g_limited: false,
            modem_level: -1,
            hotplug_active: false,
            boost_disabled: false,
            brightness_capped: false,
            theme: THEME_GLOBAL,
        }
    }

    /// Switch to a built-in monitor theme by profile name.
    pub fn apply_theme(&mut self, profile_name: &str) {
        let new_theme = *get_monitor_theme(profile_name);
        self.set_theme(new_theme, false);
    }

    /// Build and apply a dynamic monitor theme from a custom profile's extended
    /// knobs (Feature A: GPU curve, Feature C: boost/hotplug/brightness temps).
    /// Starts from NOLIMITS (nothing throttles) and only tightens what the user
    /// configured; unset (zero) fields stay at the emergency-only defaults.
    pub fn apply_custom_theme(&mut self, extras: &CustomExtras) {
        let mut t = THEME_NOLIMITS;
        t.name = "CUSTOM";

        // Feature A — GPU curve: convert each target freq (Hz) to a throttle
        // fraction (1.0 - freq/hw_max). check_gpu inverts it back to a perf
        // fraction and snaps to the nearest available DVFS step.
        let hw = self.gpu_dvfs.hw_max_hz.max(1);
        let n = extras.gpu_count.min(MAX_GPU_LEVELS).min(CUSTOM_GPU_MAX);
        for i in 0..n {
            t.gpu_trig[i] = extras.gpu_trig[i];
            t.gpu_clr[i] = extras.gpu_clr[i];
            let frac = 1.0 - (extras.gpu_freq_hz[i] as f64 / hw as f64);
            t.gpu_frac[i] = frac.clamp(0.0, 1.0);
        }
        t.gpu_level_count = n;

        // Feature C — monitor thresholds (0 = leave at NOLIMITS emergency default).
        if extras.boost_trig > 0 {
            t.boost_trig = extras.boost_trig;
            t.boost_clr = (extras.boost_trig - 2000).max(0);
        }
        if extras.hotplug_trig > 0 {
            t.hotplug_trig = extras.hotplug_trig;
            t.hotplug_clr = (extras.hotplug_trig - 2000).max(0);
        }
        if extras.bl_trig > 0 {
            t.bl_trig = extras.bl_trig;
            t.bl_clr = (extras.bl_trig - 2000).max(0);
            if extras.bl_cap > 0 {
                t.bl_cap = extras.bl_cap;
            }
        }

        // Always (re)apply — a custom reload keeps the name "CUSTOM", so the
        // name-based skip would otherwise ignore edited thresholds.
        self.set_theme(t, true);
    }

    /// Install `new_theme`, resetting monitor state so thresholds re-evaluate.
    /// `force` bypasses the same-name skip (needed for edited custom themes).
    fn set_theme(&mut self, new_theme: MonitorTheme, force: bool) {
        if !force && self.theme.name == new_theme.name {
            return; // Same theme, no change
        }

        info!("Monitor: theme switch {} → {}", self.theme.name, new_theme.name);
        self.theme = new_theme;

        // Reset all states to force re-evaluation with new thresholds
        self.gpu_level = -1;
        self.hotplug_active = false;
        self.boost_disabled = false;
        self.brightness_capped = false;

        // Restore GPU to unrestricted (DVFS primary, cooling_device fallback)
        self.restore_gpu();
        // Restore hotplug
        for fd in &self.hotplug_fds {
            fd.write_int(0);
        }
        // Restore boost
        if let Some(ref fd) = self.boost_fd {
            fd.write_int(1);
        }
        // Restore brightness
        if let Some(ref fd) = self.brightness_fd {
            fd.write_int(-1);
        }
    }

    /// Run all monitor checks.
    /// `vsensor0_mc`: current VSENSOR0 temperature (mC)
    pub fn run(&mut self, vsensor0_mc: i64) {
        self.check_gpu(vsensor0_mc);
        self.check_npu(vsensor0_mc);
        self.check_modem_5g(vsensor0_mc);
        self.check_modem_level(vsensor0_mc);
        self.check_hotplug(vsensor0_mc);
        self.check_boost(vsensor0_mc);
        self.check_brightness(vsensor0_mc);
        self.ddr_monitor.check();
    }

    fn check_gpu(&mut self, temp: i64) {
        if self.theme.gpu_level_count == 0 {
            // No GPU throttling for this theme — restore unrestricted once.
            if self.gpu_level >= 0 {
                self.restore_gpu();
                self.gpu_level = -1;
            }
            return;
        }

        let count = self.theme.gpu_level_count;
        let new_level = compute_level(
            &self.theme.gpu_trig[..count],
            &self.theme.gpu_clr[..count],
            self.gpu_level, temp
        );
        if new_level != self.gpu_level {
            // theme.gpu_frac is a THROTTLE fraction (1.0 = maximum throttle).
            let throttle_frac = if new_level >= 0 {
                self.theme.gpu_frac[new_level as usize]
            } else {
                0.0
            };
            self.apply_gpu_throttle(throttle_frac, temp, new_level);
            self.gpu_level = new_level;
        }
    }

    /// Apply a GPU throttle fraction (0.0 = unrestricted, 1.0 = max throttle)
    /// via direct DVFS when available, else the cooling_device fallback.
    fn apply_gpu_throttle(&mut self, throttle_frac: f64, temp: i64, level: i32) {
        if self.gpu_dvfs.is_available() {
            // DVFS takes a PERFORMANCE fraction, so invert the throttle fraction.
            let freq_frac = (1.0 - throttle_frac).clamp(0.0, 1.0);
            self.gpu_dvfs.set_fraction(freq_frac);
            info!("Monitor GPU(dvfs): {:.1}°C → level {} perf={:.2} maxfreq={}Hz",
                  temp as f64 / 1000.0, level, freq_frac, self.gpu_dvfs.last_written_hz);
        } else if let Some(ref fd) = self.gpu_cdev_fd {
            let cdev_state = (throttle_frac * self.gpu_max_state as f64) as i64;
            fd.write_int(cdev_state);
            info!("Monitor GPU(cdev): {:.1}°C → level {} cdev={}/{}",
                  temp as f64 / 1000.0, level, cdev_state, self.gpu_max_state);
        }
    }

    /// Restore the GPU to its unrestricted maximum frequency.
    fn restore_gpu(&mut self) {
        if self.gpu_dvfs.is_available() {
            self.gpu_dvfs.set_fraction(1.0);
        } else if let Some(ref fd) = self.gpu_cdev_fd {
            fd.write_int(0);
        }
    }

    fn check_npu(&mut self, temp: i64) {
        let new_level = compute_level(
            &MON_NPU_TRIG, &MON_NPU_CLR, self.npu_level, temp
        );
        if new_level != self.npu_level {
            let cdev_state = if new_level >= 0 {
                MON_NPU_CDEV[new_level as usize]
            } else {
                0
            };
            if let Some(ref fd) = self.npu_cdev_fd {
                fd.write_int(cdev_state);
            }
            info!("Monitor NPU: {:.1}°C → level {} cdev={}",
                  temp as f64 / 1000.0, new_level, cdev_state);
            self.npu_level = new_level;
        }
    }

    fn check_modem_5g(&mut self, temp: i64) {
        if temp >= MON_MODEM5G_TRIG && !self.modem_5g_limited {
            self.modem_5g_limited = true;
            if let Some(ref fd) = self.modem_limit_fd {
                fd.write_int(1); // 1 = limit to 4G
            }
            info!("Monitor Modem: {:.1}°C → 5G→4G limited", temp as f64 / 1000.0);
        } else if temp < MON_MODEM5G_CLR && self.modem_5g_limited {
            self.modem_5g_limited = false;
            if let Some(ref fd) = self.modem_limit_fd {
                fd.write_int(0); // 0 = allow 5G
            }
            info!("Monitor Modem: {:.1}°C → 5G restored", temp as f64 / 1000.0);
        }
    }

    fn check_modem_level(&mut self, temp: i64) {
        let new_level = compute_level(
            &MON_MODEM_TRIG, &MON_MODEM_CLR, self.modem_level, temp
        );
        if new_level != self.modem_level {
            if let Some(ref fd) = self.modem_level_fd {
                fd.write_int(if new_level >= 0 { new_level as i64 + 1 } else { 0 });
            }
            self.modem_level = new_level;
        }
    }

    fn check_hotplug(&mut self, temp: i64) {
        if temp >= self.theme.hotplug_trig && !self.hotplug_active {
            self.hotplug_active = true;
            for fd in &self.hotplug_fds {
                fd.write_int(1); // Park core
            }
            info!("Monitor Hotplug: {:.1}°C → cores parked", temp as f64 / 1000.0);
        } else if temp < self.theme.hotplug_clr && self.hotplug_active {
            self.hotplug_active = false;
            for fd in &self.hotplug_fds {
                fd.write_int(0); // Restore core
            }
            info!("Monitor Hotplug: {:.1}°C → cores restored", temp as f64 / 1000.0);
        }
    }

    fn check_boost(&mut self, temp: i64) {
        if temp >= self.theme.boost_trig && !self.boost_disabled {
            self.boost_disabled = true;
            if let Some(ref fd) = self.boost_fd {
                fd.write_int(0); // Disable boost
            }
            info!("Monitor Boost: {:.1}°C → boost disabled", temp as f64 / 1000.0);
        } else if temp < self.theme.boost_clr && self.boost_disabled {
            self.boost_disabled = false;
            if let Some(ref fd) = self.boost_fd {
                fd.write_int(1); // Enable boost
            }
            info!("Monitor Boost: {:.1}°C → boost enabled", temp as f64 / 1000.0);
        }
    }

    fn check_brightness(&mut self, temp: i64) {
        if temp >= self.theme.bl_trig && !self.brightness_capped {
            self.brightness_capped = true;
            if let Some(ref fd) = self.brightness_fd {
                fd.write_int(self.theme.bl_cap);
            }
            info!("Monitor Brightness: {:.1}°C → capped to {}", temp as f64 / 1000.0, self.theme.bl_cap);
        } else if temp < self.theme.bl_clr && self.brightness_capped {
            self.brightness_capped = false;
            if let Some(ref fd) = self.brightness_fd {
                fd.write_int(-1); // -1 = no cap
            }
            info!("Monitor Brightness: {:.1}°C → cap removed", temp as f64 / 1000.0);
        }
    }
}

/// Compute the highest active level using trigger/clear thresholds with hysteresis.
fn compute_level(trigs: &[i64], clrs: &[i64], current_level: i32, temp: i64) -> i32 {
    let mut new_level: i32 = -1;
    for i in 0..trigs.len() {
        let i32_i = i as i32;
        if i32_i <= current_level {
            // Already at or above — use clear threshold
            if temp >= clrs[i] {
                new_level = i32_i;
            }
        } else {
            // Below — use trigger threshold
            if temp >= trigs[i] {
                new_level = i32_i;
            }
        }
    }
    new_level
}

