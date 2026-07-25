// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Dynamic thermal budget allocation (Phase 3.1).
//!
//! Computes a recommended CPU/GPU frequency-fraction split from live
//! utilisation and thermal headroom.
//!
//! This module is intentionally **advisory**: it samples `/proc/stat` and
//! `gpu_busy_percentage` (both zero-allocation, fd-cached) and exposes a
//! `(cpu_frac, gpu_frac)` recommendation for logging/telemetry. It does NOT
//! write `scaling_max_freq` or `devfreq/max_freq` — SS (CPU) and the GPU DVFS
//! monitor already own those actuators, and a second writer would fight them.
//! The public `cpu_frac`/`gpu_frac` fields are the seam a future unified
//! allocator would consume (e.g. SS taking `cpu_frac * hw_max` as its floor).
//!
//! NOTE: reading `/proc/stat` (label `proc_stat`) requires the sepolicy rule
//! `allow mi_thermal_engine proc_stat:file r_file_perms;` (added to
//! mi_thermal_engine.te). Without it the fd fails to open and this module
//! degrades to GPU-only utilisation — it never crashes.

use log::info;

use crate::io::SysfsFd;

const PROC_STAT_PATH: &str = "/proc/stat";
const GPU_BUSY_PATH: &str = "/sys/class/kgsl/kgsl-3d0/gpu_busy_percentage";

/// EMA smoothing factor for utilisation (α=0.3, per spec).
const UTIL_ALPHA: f64 = 0.3;

/// Output fraction clamps.
const FRAC_MIN: f64 = 0.3;
const FRAC_MAX: f64 = 1.0;

/// Thermal pressure sensitivity, per °C over target.
const PRESSURE_K: f64 = 0.1;

/// Log the recommendation once every N update() calls (de-spam).
const LOG_EVERY: u32 = 30;

pub struct ThermalBudget {
    stat_fd: Option<SysfsFd>,
    gpu_busy_fd: Option<SysfsFd>,

    // /proc/stat cumulative jiffies from the previous sample.
    prev_busy: i64,
    prev_total: i64,
    have_prev: bool,

    // Utilisation EMAs (0.0..1.0).
    cpu_util_ema: f64,
    gpu_util_ema: f64,

    // Last computed recommendation (0.3..1.0).
    pub cpu_frac: f64,
    pub gpu_frac: f64,

    log_counter: u32,
}

impl ThermalBudget {
    pub fn new() -> Self {
        ThermalBudget {
            stat_fd: SysfsFd::open_ro(PROC_STAT_PATH),
            gpu_busy_fd: SysfsFd::open_ro(GPU_BUSY_PATH),
            prev_busy: 0,
            prev_total: 0,
            have_prev: false,
            cpu_util_ema: 0.0,
            gpu_util_ema: 0.0,
            cpu_frac: FRAC_MAX,
            gpu_frac: FRAC_MAX,
            log_counter: 0,
        }
    }

    /// Sample utilisation and recompute the budget split. ZERO heap allocation.
    /// `temp_mc`/`target_mc` drive the thermal-pressure factor.
    pub fn update(&mut self, temp_mc: i64, target_mc: i64) {
        let cpu_util = self.read_cpu_util();
        let gpu_util = self.read_gpu_util();

        self.cpu_util_ema = UTIL_ALPHA * cpu_util + (1.0 - UTIL_ALPHA) * self.cpu_util_ema;
        self.gpu_util_ema = UTIL_ALPHA * gpu_util + (1.0 - UTIL_ALPHA) * self.gpu_util_ema;

        // Split the shared budget proportional to each domain's demand.
        let sum = self.cpu_util_ema + self.gpu_util_ema;
        let (cpu_share, gpu_share) = if sum > 0.001 {
            (self.cpu_util_ema / sum, self.gpu_util_ema / sum)
        } else {
            (0.5, 0.5)
        };

        // Thermal pressure: 1.0 at/below target, shrinking as we overshoot.
        let over_c = ((temp_mc - target_mc) as f64 / 1000.0).max(0.0);
        let pressure = 1.0 / (1.0 + over_c * PRESSURE_K);

        // Each domain's fraction = headroom (pressure) biased toward the busier
        // domain, so the one actually driving heat keeps more of the budget.
        self.cpu_frac = clamp_frac(pressure * (0.75 + 0.5 * cpu_share));
        self.gpu_frac = clamp_frac(pressure * (0.75 + 0.5 * gpu_share));

        self.log_counter += 1;
        if self.log_counter >= LOG_EVERY {
            self.log_counter = 0;
            info!("Budget: cpu_util={:.0}% gpu_util={:.0}% pressure={:.2} → cpu_frac={:.2} gpu_frac={:.2}",
                  self.cpu_util_ema * 100.0, self.gpu_util_ema * 100.0,
                  pressure, self.cpu_frac, self.gpu_frac);
        }
    }

    /// Aggregate CPU utilisation from /proc/stat (delta busy / delta total).
    /// Zero-allocation: reads the first line into a stack buffer and parses it.
    fn read_cpu_util(&mut self) -> f64 {
        let fd = match self.stat_fd.as_ref() {
            Some(f) => f,
            None => return self.cpu_util_ema,
        };
        let mut buf = [0u8; 192];
        let s = match fd.read_str(&mut buf) {
            Some(s) => s,
            None => return self.cpu_util_ema,
        };

        // First line: "cpu  user nice system idle iowait irq softirq steal ..."
        let mut nums = [0i64; 10];
        let mut count = 0usize;
        for tok in s.split_whitespace() {
            if tok == "cpu" {
                continue;
            }
            if count >= nums.len() {
                break;
            }
            match tok.parse::<i64>() {
                Ok(v) => {
                    nums[count] = v;
                    count += 1;
                }
                Err(_) => break, // reached the per-core "cpu0" line label
            }
        }
        if count < 5 {
            return self.cpu_util_ema;
        }

        let total: i64 = nums[..count].iter().sum();
        let idle = nums[3] + nums[4]; // idle + iowait
        let busy = total - idle;

        let util = if self.have_prev {
            let dt = total - self.prev_total;
            let db = busy - self.prev_busy;
            if dt > 0 {
                (db as f64 / dt as f64).clamp(0.0, 1.0)
            } else {
                self.cpu_util_ema
            }
        } else {
            0.0
        };

        self.prev_total = total;
        self.prev_busy = busy;
        self.have_prev = true;
        util
    }

    /// GPU utilisation from gpu_busy_percentage ("49 %" → 0.49).
    fn read_gpu_util(&self) -> f64 {
        self.gpu_busy_fd
            .as_ref()
            .and_then(|f| f.read_int())
            .map(|v| (v as f64 / 100.0).clamp(0.0, 1.0))
            .unwrap_or(self.gpu_util_ema)
    }
}

fn clamp_frac(v: f64) -> f64 {
    if v.is_finite() {
        v.clamp(FRAC_MIN, FRAC_MAX)
    } else {
        FRAC_MAX
    }
}
