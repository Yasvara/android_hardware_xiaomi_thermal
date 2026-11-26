// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Closed-loop SM8735 (Snapdragon 8s Gen 4 / POCO F7 `onyx`) silicon & thermal simulator.
//!
//! Exercises the real production `SsAlgo`, `SkinController`, `MonitorAlgo`, `GpuDvfs`,
//! `KalmanFilter`, and `SicAlgo` against real hardware OPP tables dumped from `onyx`:
//!   - Cluster 0 (policy0): 2× Cortex-A720 (0xd81), 364.8 – 2016.0 MHz (15 OPPs)
//!   - Cluster 1 (policy2): 3× Cortex-A720 (0xd81), 480.0 – 3014.4 MHz (23 OPPs)
//!   - Cluster 2 (policy5): 2× Cortex-A720 (0xd81), 480.0 – 2803.2 MHz (21 OPPs)
//!   - Cluster 3 (policy7): 1× Cortex-X4   (0xd82), 480.0 – 3206.4 MHz (20 OPPs + turbo)
//!   - GPU (Adreno 825):    11 OPPs, 150 – 1162 MHz
//!
//! Models a calibrated 3-node RC thermal network (die → PCB/vapor chamber → skin/VSENSOR0),
//! super-linear dynamic power P ∝ f^α, thermal leakage, charger/cooler heat injection,
//! ±150 mC sensor noise, and frame-rate / 1%-low FPS pacing under fluctuating game workloads.

use crate::monitor::MonitorAlgo;
use crate::sensor::KalmanFilter;
use crate::ss::SsAlgo;

// ── Real SM8735 (`onyx`) OPP tables verified from /sys/devices/system/cpu/cpufreq & kgsl ──

const OPPS_CPU0: &[i64] = &[
    364_800, 441_600, 614_400, 748_800, 883_200, 979_200, 1_075_200, 1_190_400,
    1_286_400, 1_401_600, 1_516_800, 1_632_000, 1_785_600, 1_900_800, 2_016_000,
];

const OPPS_CPU2: &[i64] = &[
    480_000, 633_600, 787_200, 940_800, 1_056_000, 1_190_400, 1_286_400, 1_401_600,
    1_516_800, 1_632_000, 1_728_000, 1_824_000, 1_920_000, 2_073_600, 2_208_000,
    2_323_200, 2_438_400, 2_515_200, 2_611_200, 2_707_200, 2_803_200, 2_918_400, 3_014_400,
];

const OPPS_CPU5: &[i64] = &[
    480_000, 633_600, 787_200, 940_800, 1_056_000, 1_190_400, 1_286_400, 1_401_600,
    1_516_800, 1_632_000, 1_728_000, 1_824_000, 1_920_000, 2_073_600, 2_208_000,
    2_323_200, 2_438_400, 2_515_200, 2_611_200, 2_707_200, 2_803_200,
];

const OPPS_CPU7: &[i64] = &[
    480_000, 633_600, 787_200, 960_000, 1_094_400, 1_228_800, 1_363_200, 1_478_400,
    1_670_400, 1_785_600, 1_920_000, 2_054_400, 2_169_600, 2_284_800, 2_400_000,
    2_515_200, 2_630_400, 2_745_600, 2_841_600, 2_956_800, 3_206_400,
];

const HW_MAX_CPU: [i64; 4] = [2_016_000, 3_014_400, 2_803_200, 3_206_400];
const CPU_OPPS: [&[i64]; 4] = [OPPS_CPU0, OPPS_CPU2, OPPS_CPU5, OPPS_CPU7];

const OPPS_GPU_HZ: &[i64] = &[
    150_000_000, 300_000_000, 515_000_000, 685_000_000, 763_000_000, 873_000_000,
    915_000_000, 965_000_000, 1_090_000_000, 1_150_000_000, 1_162_000_000,
];

/// Instantaneous game/app compute demand for one simulated second.
#[derive(Clone, Copy)]
struct WorkloadFrame {
    /// Per-cluster utilisation demand (0.0..1.0) at reference frequency.
    cpu_util: [f64; 4],
    /// Adreno 825 GPU utilisation demand (0.0..1.0).
    gpu_util: f64,
    /// Target framerate (e.g. 60.0 for Genshin/HSR, 120.0 for MOBA/FPS).
    target_fps: f64,
    /// Normalized CPU work units needed for `target_fps` (1.0 = needs heavy big+prime clocks).
    cpu_need: f64,
    /// Normalized GPU work units needed for `target_fps`.
    gpu_need: f64,
}

/// Calibrated 3-node RC thermal model of POCO F7 (`onyx`, SM8735 + vapor chamber).
struct Sm8735Plant {
    ambient_c: f64,
    t_die_c: f64,
    t_board_c: f64,
    t_skin_c: f64,
    /// Extra heat (+W from charger PMIC/CP) or cooling (-W from Peltier/fan cooler).
    aux_watts: f64,
    /// Effective board-to-ambient thermal resistance (°C/W): ~2.65 bare hand, ~1.35 with active cooler.
    r_board_amb: f64,
    rng_state: u64,
}

impl Sm8735Plant {
    fn new(ambient_c: f64, start_c: f64, aux_watts: f64, r_board_amb: f64) -> Self {
        Self {
            ambient_c,
            t_die_c: start_c + 6.0,
            t_board_c: start_c + 1.5,
            t_skin_c: start_c,
            aux_watts,
            r_board_amb,
            rng_state: 0x5EED_8735_C0DE_CAFE,
        }
    }

    /// Deterministic zero-dependency sensor noise in `[-150, +150]` mC.
    fn next_noise_mc(&mut self) -> i64 {
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng_state = x;
        ((x % 301) as i64) - 150
    }

    /// Compute SM8735 SoC power (W) for current per-cluster CPU & GPU frequency ceilings.
    /// Power exponents reflect TSMC N4P voltage-frequency curves (X4 steepest, A720 moderate).
    fn compute_power_w(&self, cpu_khz: [i64; 4], gpu_hz: i64, w: WorkloadFrame) -> f64 {
        // Peak dynamic power per cluster at 100% util & hw_max:
        //   cpu0 (2×A720@2.02G): 1.2 W
        //   cpu2 (3×A720@3.01G): 3.6 W
        //   cpu5 (2×A720@2.80G): 2.2 W
        //   cpu7 (1×X4@3.21G):   3.4 W
        //   Adreno 825@1.16G:    4.4 W
        const CPU_PEAK_W: [f64; 4] = [1.2, 3.6, 2.2, 3.4];
        const CPU_ALPHA: [f64; 4] = [2.15, 2.35, 2.30, 2.65];
        const GPU_PEAK_W: f64 = 4.4;
        const GPU_ALPHA: f64 = 2.20;
        const BASE_DISPLAY_MEM_W: f64 = 1.15;

        let mut p = BASE_DISPLAY_MEM_W;
        for i in 0..4 {
            let f_ratio = (cpu_khz[i] as f64 / HW_MAX_CPU[i] as f64).clamp(0.15, 1.0);
            p += CPU_PEAK_W[i] * w.cpu_util[i] * f_ratio.powf(CPU_ALPHA[i]);
        }
        let g_ratio = (gpu_hz as f64 / 1_162_000_000.0).clamp(0.12, 1.0);
        p += GPU_PEAK_W * w.gpu_util * g_ratio.powf(GPU_ALPHA);

        // Temperature-dependent silicon leakage (+0.025 W/°C above 45°C die).
        let leak = 0.025 * (self.t_die_c - 45.0).max(0.0);
        p + leak
    }

    /// Estimate achieved FPS given SM8735 CPU & GPU ceilings and scene demand.
    fn achieved_fps(cpu_khz: [i64; 4], gpu_hz: i64, w: WorkloadFrame) -> f64 {
        // Weighted game compute capacity:
        //   X4 main thread (35%), 3×A720 workers (35%), 2×A720 workers (20%), 2×A720 bg (10%)
        let c0 = cpu_khz[0] as f64 / HW_MAX_CPU[0] as f64;
        let c1 = cpu_khz[1] as f64 / HW_MAX_CPU[1] as f64;
        let c2 = cpu_khz[2] as f64 / HW_MAX_CPU[2] as f64;
        let c3 = cpu_khz[3] as f64 / HW_MAX_CPU[3] as f64;
        let cpu_cap = 0.10 * c0 + 0.35 * c1 + 0.20 * c2 + 0.35 * c3;
        let gpu_cap = gpu_hz as f64 / 1_162_000_000.0;

        let cpu_fps = if w.cpu_need > 0.01 {
            w.target_fps * (cpu_cap / w.cpu_need).min(1.0)
        } else {
            w.target_fps
        };
        let gpu_fps = if w.gpu_need > 0.01 {
            w.target_fps * (gpu_cap / w.gpu_need).min(1.0)
        } else {
            w.target_fps
        };
        cpu_fps.min(gpu_fps).clamp(5.0, w.target_fps)
    }

    /// Advance thermal RC state by `dt_s` seconds and return noisy `VSENSOR0` (mC).
    fn step(&mut self, cpu_khz: [i64; 4], gpu_hz: i64, w: WorkloadFrame, dt_s: f64) -> i64 {
        let soc_w = self.compute_power_w(cpu_khz, gpu_hz, w);
        const R_DIE_BOARD: f64 = 3.2;

        let total_board_w = (soc_w + self.aux_watts).max(0.2);
        let die_target = self.t_board_c + R_DIE_BOARD * soc_w;
        let board_target = self.ambient_c + self.r_board_amb * total_board_w;

        self.t_die_c += (die_target - self.t_die_c) * (dt_s / 4.0).min(1.0);
        self.t_board_c += (board_target - self.t_board_c) * (dt_s / 35.0).min(1.0);
        // Vapor chamber spreads board heat into skin (`VSENSOR0`).
        self.t_skin_c += (self.t_board_c - self.t_skin_c) * (dt_s / 105.0).min(1.0);

        ((self.t_skin_c * 1000.0) as i64) + self.next_noise_mc()
    }
}

/// Summary telemetry collected across a closed-loop run.
struct SimMetrics {
    peak_skin_c: f64,
    final_skin_c: f64,
    avg_fps: f64,
    low_1pct_fps: f64,
    min_big_khz: i64,
    min_gpu_mhz: i64,
    max_big_1s_swing_khz: i64,
    cliff_drops: usize,
    avg_power_w: f64,
}

/// Realistic fluctuating 30-minute Genshin Impact workload trace (1800 s).
fn genshin_workload_at(sec: usize) -> WorkloadFrame {
    match sec {
        // 0..180s: Open-world Fontaine/Natlan exploration (bursty streaming & lighting)
        0..=179 => {
            let burst = ((sec / 12) % 3 == 0) as i32 as f64;
            WorkloadFrame {
                cpu_util: [0.50, 0.55 + 0.15 * burst, 0.50 + 0.15 * burst, 0.65 + 0.20 * burst],
                gpu_util: 0.72 + 0.14 * burst,
                target_fps: 60.0,
                cpu_need: 0.58 + 0.12 * burst,
                gpu_need: 0.62 + 0.14 * burst,
            }
        }
        // 180..420s: Spiral Abyss heavy combat (alternating elemental burst waves every 15s)
        180..=419 => {
            let wave = if (sec % 20) < 12 { 1.0 } else { 0.35 };
            WorkloadFrame {
                cpu_util: [0.60, 0.65 + 0.25 * wave, 0.60 + 0.25 * wave, 0.75 + 0.22 * wave],
                gpu_util: 0.78 + 0.18 * wave,
                target_fps: 60.0,
                cpu_need: 0.64 + 0.18 * wave,
                gpu_need: 0.68 + 0.16 * wave,
            }
        }
        // 420..460s: Teleport + shader/asset compile spike (heavily CPU-bound!)
        420..=459 => WorkloadFrame {
            cpu_util: [0.85, 0.95, 0.92, 0.98],
            gpu_util: 0.22,
            target_fps: 60.0,
            cpu_need: 0.85,
            gpu_need: 0.20,
        },
        // 460..600s: Story dialogue / cutscene cooldown
        460..=599 => WorkloadFrame {
            cpu_util: [0.25, 0.22, 0.20, 0.30],
            gpu_util: 0.36,
            target_fps: 60.0,
            cpu_need: 0.25,
            gpu_need: 0.30,
        },
        // 600..1500s: Sustained Natlan boss & co-op combat (heavy alternating load)
        600..=1499 => {
            let phase = ((sec as f64) * 0.21).sin() * 0.5 + 0.5;
            WorkloadFrame {
                cpu_util: [0.62, 0.68 + 0.22 * phase, 0.64 + 0.22 * phase, 0.78 + 0.20 * phase],
                gpu_util: 0.80 + 0.16 * phase,
                target_fps: 60.0,
                cpu_need: 0.65 + 0.16 * phase,
                gpu_need: 0.70 + 0.15 * phase,
            }
        }
        // 1500..1800s: Pause menu / cooldown
        _ => WorkloadFrame {
            cpu_util: [0.18, 0.12, 0.10, 0.15],
            gpu_util: 0.20,
            target_fps: 60.0,
            cpu_need: 0.15,
            gpu_need: 0.18,
        },
    }
}

fn run_scenario_r<F>(
    profile: &str,
    force_ladder: bool,
    ambient_c: f64,
    start_c: f64,
    aux_watts: f64,
    r_board_amb: f64,
    secs: usize,
    workload: F,
) -> SimMetrics
where
    F: Fn(usize) -> WorkloadFrame,
{
    let mut ss = SsAlgo::new_simulated(profile, HW_MAX_CPU, CPU_OPPS, force_ladder);
    let mut mon = MonitorAlgo::new_simulated(profile, OPPS_GPU_HZ);
    let mut kf = KalmanFilter::new();
    let mut plant = Sm8735Plant::new(ambient_c, start_c, aux_watts, r_board_amb);

    let mut fps_samples = Vec::with_capacity(secs);
    let mut peak_skin_c = start_c;
    let mut min_big_khz = i64::MAX;
    let mut min_gpu_mhz = i64::MAX;
    let mut prev_big_khz = HW_MAX_CPU[1];
    let mut max_big_1s_swing_khz = 0i64;
    let mut cliff_drops = 0usize;
    let mut power_sum = 0.0f64;

    for sec in 0..secs {
        let w = workload(sec);
        let cpu_caps = ss.applied_caps_khz();
        let gpu_cap = mon.applied_gpu_cap_hz();

        power_sum += plant.compute_power_w(cpu_caps, gpu_cap, w);
        let fps = Sm8735Plant::achieved_fps(cpu_caps, gpu_cap, w);
        fps_samples.push(fps);

        let raw_vsensor_mc = plant.step(cpu_caps, gpu_cap, w, 1.0);
        peak_skin_c = peak_skin_c.max(plant.t_skin_c);

        let filtered_mc = kf.update_dt(raw_vsensor_mc, 1.0);
        ss.run_predictive_dt(filtered_mc, kf.rate(), 1.0);

        let cpu_avg_util =
            (w.cpu_util[0] * 2.0 + w.cpu_util[1] * 3.0 + w.cpu_util[2] * 2.0 + w.cpu_util[3]) / 8.0;
        let gpu_share = if cpu_avg_util + w.gpu_util > 0.05 {
            w.gpu_util / (cpu_avg_util + w.gpu_util)
        } else {
            0.5
        };
        mon.set_skin_output(ss.skin_output(), ss.skin_gpu_floor_hz(), ss.skin_gpu_ceil_hz());
        mon.set_gpu_share(gpu_share);
        mon.run(filtered_mc);

        let next_caps = ss.applied_caps_khz();
        let big = next_caps[1];
        min_big_khz = min_big_khz.min(big);
        min_gpu_mhz = min_gpu_mhz.min(mon.applied_gpu_cap_hz() / 1_000_000);
        let swing = (big - prev_big_khz).abs();
        max_big_1s_swing_khz = max_big_1s_swing_khz.max(swing);
        if prev_big_khz - big >= 1_200_000 {
            cliff_drops += 1;
        }
        prev_big_khz = big;
    }

    fps_samples.sort_by(|a, b| a.total_cmp(b));
    let low_idx = (fps_samples.len() / 100).max(1) - 1;
    let low_1pct_fps = fps_samples[low_idx];
    let avg_fps = fps_samples.iter().sum::<f64>() / fps_samples.len() as f64;

    SimMetrics {
        peak_skin_c,
        final_skin_c: plant.t_skin_c,
        avg_fps,
        low_1pct_fps,
        min_big_khz,
        min_gpu_mhz,
        max_big_1s_swing_khz,
        cliff_drops,
        avg_power_w: power_sum / secs as f64,
    }
}

fn run_scenario<F>(
    profile: &str,
    force_ladder: bool,
    ambient_c: f64,
    start_c: f64,
    aux_watts: f64,
    secs: usize,
    workload: F,
) -> SimMetrics
where
    F: Fn(usize) -> WorkloadFrame,
{
    run_scenario_r(profile, force_ladder, ambient_c, start_c, aux_watts, 2.65, secs, workload)
}

#[test]
fn sim_genshin_30min_pi_vs_legacy_cliff() {
    let legacy = run_scenario("genshin", true, 28.0, 34.0, 0.0, 1800, genshin_workload_at);
    let smooth = run_scenario("genshin", false, 28.0, 34.0, 0.0, 1800, genshin_workload_at);

    eprintln!(
        "\n[SM8735 Genshin 30m] LEGACY CLIFF: peak={:.1}C avg={:.1}fps 1%low={:.1}fps minBig={}MHz maxSwing={}MHz cliffs={} avgW={:.2}W",
        legacy.peak_skin_c, legacy.avg_fps, legacy.low_1pct_fps,
        legacy.min_big_khz / 1000, legacy.max_big_1s_swing_khz / 1000,
        legacy.cliff_drops, legacy.avg_power_w
    );
    eprintln!(
        "[SM8735 Genshin 30m] YASVARA PI  : peak={:.1}C avg={:.1}fps 1%low={:.1}fps minBig={}MHz maxSwing={}MHz cliffs={} avgW={:.2}W",
        smooth.peak_skin_c, smooth.avg_fps, smooth.low_1pct_fps,
        smooth.min_big_khz / 1000, smooth.max_big_1s_swing_khz / 1000,
        smooth.cliff_drops, smooth.avg_power_w
    );

    // Proof that legacy Joyose-cliff collapses to 633 MHz with severe 1%-low drops.
    assert!(legacy.cliff_drops >= 1, "legacy cliff should reproduce Joyose cliff drop");
    assert_eq!(legacy.min_big_khz, 633_600);

    // Yasvara smooth PI eliminates cliffs, keeps skin safe, and dramatically improves 1% lows.
    assert_eq!(smooth.cliff_drops, 0, "PI must never cliff-drop");
    assert!(smooth.peak_skin_c < 47.0, "Genshin PI overshoot: {:.1}C", smooth.peak_skin_c);
    assert!(smooth.min_big_khz >= 1_516_800, "big cluster dropped too low: {}", smooth.min_big_khz);
    assert!(smooth.min_gpu_mhz >= 685, "GPU dropped too low: {} MHz", smooth.min_gpu_mhz);
    assert!(
        smooth.low_1pct_fps > legacy.low_1pct_fps + 10.0,
        "PI 1% low ({:.1}) should beat legacy cliff ({:.1}) by >10 FPS",
        smooth.low_1pct_fps, legacy.low_1pct_fps
    );
    assert!(
        smooth.max_big_1s_swing_khz < 500_000,
        "PI 1s swing too large: {} kHz",
        smooth.max_big_1s_swing_khz
    );
}

#[test]
fn sim_hsr_gpu_bound_prime_first_budget() {
    let mut ss = SsAlgo::new_simulated("hsr", HW_MAX_CPU, CPU_OPPS, false);
    let mut mon = MonitorAlgo::new_simulated("hsr", OPPS_GPU_HZ);
    let mut kf = KalmanFilter::new();
    let mut plant = Sm8735Plant::new(28.0, 35.0, 0.0, 2.65);

    // GPU-bound Honkai: Star Rail heavy combat (92% GPU, 45% CPU).
    let hsr = WorkloadFrame {
        cpu_util: [0.40, 0.45, 0.42, 0.55],
        gpu_util: 0.92,
        target_fps: 60.0,
        cpu_need: 0.48,
        gpu_need: 0.78,
    };

    for _ in 0..1200 {
        let raw = plant.step(ss.applied_caps_khz(), mon.applied_gpu_cap_hz(), hsr, 1.0);
        let filt = kf.update_dt(raw, 1.0);
        ss.run_predictive_dt(filt, kf.rate(), 1.0);
        mon.set_skin_output(ss.skin_output(), ss.skin_gpu_floor_hz(), ss.skin_gpu_ceil_hz());
        mon.set_gpu_share(0.68); // GPU-bound
        mon.run(filt);
    }

    let caps = ss.applied_caps_khz();
    let gpu_mhz = mon.applied_gpu_cap_hz() / 1_000_000;
    let silver_frac = caps[0] as f64 / HW_MAX_CPU[0] as f64;
    let prime_frac = caps[3] as f64 / HW_MAX_CPU[3] as f64;
    eprintln!(
        "[SM8735 HSR 20m] skin={:.1}C caps={}/{}/{}/{}MHz gpu={}MHz",
        plant.t_skin_c, caps[0] / 1000, caps[1] / 1000, caps[2] / 1000, caps[3] / 1000, gpu_mhz
    );

    assert!(plant.t_skin_c < 47.0, "HSR skin too hot: {:.1}C", plant.t_skin_c);
    assert!(gpu_mhz >= 763, "HSR GPU floor violated: {} MHz", gpu_mhz);
    assert!(
        prime_frac <= silver_frac,
        "Prime X4 ({:.2}) must shed heat before Silver A720 ({:.2})",
        prime_frac, silver_frac
    );
}

#[test]
fn sim_high_fps_120hz_warm_ambient_no_cold_throttle() {
    let workload = |sec: usize| {
        let teamfight = ((sec % 30) < 15) as i32 as f64;
        WorkloadFrame {
            cpu_util: [0.45, 0.55 + 0.25 * teamfight, 0.50 + 0.25 * teamfight, 0.55 + 0.25 * teamfight],
            gpu_util: 0.55 + 0.20 * teamfight,
            target_fps: 120.0,
            cpu_need: 0.55 + 0.15 * teamfight,
            gpu_need: 0.50 + 0.15 * teamfight,
        }
    };
    let legacy = run_scenario("high_fps", true, 33.0, 36.0, 0.0, 900, workload);
    let smooth = run_scenario("high_fps", false, 33.0, 36.0, 0.0, 900, workload);
    eprintln!(
        "[SM8735 HighFPS 15m @33C] LEGACY: peak={:.1}C avg={:.1}fps 1%low={:.1}fps minBig={}MHz",
        legacy.peak_skin_c, legacy.avg_fps, legacy.low_1pct_fps, legacy.min_big_khz / 1000
    );
    eprintln!(
        "[SM8735 HighFPS 15m @33C] PID   : peak={:.1}C avg={:.1}fps 1%low={:.1}fps minBig={}MHz",
        smooth.peak_skin_c, smooth.avg_fps, smooth.low_1pct_fps, smooth.min_big_khz / 1000
    );
    assert_eq!(smooth.cliff_drops, 0);
    assert!(smooth.peak_skin_c < 47.0, "high_fps skin overshoot: {:.1}C", smooth.peak_skin_c);
    assert!(smooth.min_big_khz >= 1_516_800, "high_fps big cluster floor violated: {}", smooth.min_big_khz);
    assert!(smooth.avg_fps > legacy.avg_fps + 10.0, "PID high_fps ({:.1}) must beat legacy ({:.1})", smooth.avg_fps, legacy.avg_fps);
}

#[test]
fn sim_nolimit_cooler_and_50c_unthrottled() {
    // Part A: Active magnetic Peltier cooler (-4.5 W TEC extraction + forced convection R=1.35 °C/W).
    let cooler = run_scenario_r("performance", false, 28.0, 32.0, -4.5, 1.35, 900, genshin_workload_at);
    eprintln!(
        "[SM8735 No-Limit + Cooler 15m] peak={:.1}C avg={:.1}fps 1%low={:.1}fps minBig={}MHz minGpu={}MHz",
        cooler.peak_skin_c, cooler.avg_fps, cooler.low_1pct_fps, cooler.min_big_khz / 1000, cooler.min_gpu_mhz
    );
    assert_eq!(cooler.min_big_khz, HW_MAX_CPU[1], "No-Limit + cooler must stay at hw_max");
    assert_eq!(cooler.min_gpu_mhz, 1162, "No-Limit + cooler GPU must stay at 1162 MHz");
    assert!(cooler.peak_skin_c < 45.0, "Peltier cooler should hold skin cool: {:.1}C", cooler.peak_skin_c);

    // Part B: Uncooled No-Limit pushing right through 48°C–50.5°C with rising Kalman rate.
    let mut ss = SsAlgo::new_simulated("performance", HW_MAX_CPU, CPU_OPPS, false);
    let mut mon = MonitorAlgo::new_simulated("performance", OPPS_GPU_HZ);
    for temp_mc in (46_000..=51_500).step_by(500) {
        ss.run_predictive_dt(temp_mc, 400.0, 1.0); // fast +0.4°C/s ramp
        mon.set_skin_output(ss.skin_output(), ss.skin_gpu_floor_hz(), ss.skin_gpu_ceil_hz());
        mon.run(temp_mc);
        assert_eq!(
            ss.applied_caps_khz(), HW_MAX_CPU,
            "Performance (No-Limit) throttled CPU at {:.1}C", temp_mc as f64 / 1000.0
        );
        assert_eq!(
            mon.applied_gpu_cap_hz(), 1_162_000_000,
            "Performance (No-Limit) throttled GPU at {:.1}C", temp_mc as f64 / 1000.0
        );
    }
}

#[test]
fn sim_gaming_while_fast_charging_stable() {
    // Inject +2.2 W PMIC/charge-pump dissipation alongside heavy gaming.
    let chg = run_scenario("gaming", false, 28.0, 36.0, 2.2, 900, genshin_workload_at);
    eprintln!(
        "[SM8735 Gaming+Charging 15m] peak={:.1}C final={:.1}C avg={:.1}fps minBig={}MHz",
        chg.peak_skin_c, chg.final_skin_c, chg.avg_fps, chg.min_big_khz / 1000
    );
    assert_eq!(chg.cliff_drops, 0);
    assert!(chg.peak_skin_c < 47.0, "Gaming+charging skin runaway: {:.1}C", chg.peak_skin_c);
    assert!(chg.min_big_khz >= 1_190_400, "Balanced gaming big cluster below floor");
}

#[test]
fn sim_user_pentas_caps_and_touch_lift_invariant() {
    let mut ss = SsAlgo::new_simulated("default", HW_MAX_CPU, CPU_OPPS, false);
    let mut mon = MonitorAlgo::new_simulated("default", OPPS_GPU_HZ);

    // User caps Prime X4 to 2.28 GHz and GPU to 873 MHz via Pentas.
    ss.set_user_caps([0, 2_438_400, 0, 2_284_800]);
    mon.set_user_gpu_cap(873_000_000);

    // Even when cold (34°C) or lifted by touch, user ceiling must NEVER be exceeded.
    ss.run_predictive_dt(34_000, 0.0, 1.0);
    ss.set_lift(true);
    mon.set_skin_output(ss.skin_output(), ss.skin_gpu_floor_hz(), ss.skin_gpu_ceil_hz());
    mon.set_lift(true);
    mon.run(34_000);

    let caps = ss.applied_caps_khz();
    assert_eq!(caps[0], HW_MAX_CPU[0]);
    assert_eq!(caps[1], 2_438_400);
    assert_eq!(caps[3], 2_284_800);
    assert_eq!(mon.applied_gpu_cap_hz(), 873_000_000);

    // Under hot thermal throttle (46°C), thermal cap beats user cap when lower.
    ss.set_lift(false);
    mon.set_lift(false);
    for _ in 0..120 {
        ss.run_predictive_dt(46_000, 0.0, 1.0);
        mon.set_skin_output(ss.skin_output(), ss.skin_gpu_floor_hz(), ss.skin_gpu_ceil_hz());
        mon.run(46_000);
    }
    assert!(ss.applied_caps_khz()[3] <= 2_284_800);
    assert!(mon.applied_gpu_cap_hz() <= 873_000_000);

    // Releasing user cap (0) while cold cleanly restores hw_max.
    ss.set_user_caps([0, 0, 0, 0]);
    mon.set_user_gpu_cap(0);
    for _ in 0..200 {
        ss.run_predictive_dt(35_000, 0.0, 1.0);
        mon.set_skin_output(ss.skin_output(), ss.skin_gpu_floor_hz(), ss.skin_gpu_ceil_hz());
        mon.run(35_000);
    }
    assert_eq!(ss.applied_caps_khz(), HW_MAX_CPU);
    assert_eq!(mon.applied_gpu_cap_hz(), 1_162_000_000);
}

#[test]
fn sim_kalman_variable_dt_and_suspend_recovery() {
    let mut kf = KalmanFilter::new();
    // 1 Hz active ramp (+200 mC/s)
    for i in 0..15 {
        kf.update_dt(35_000 + i * 200, 1.0);
    }
    assert!(kf.rate() > 100.0 && kf.rate() < 300.0, "1Hz slope wrong: {}", kf.rate());

    // 3 s screen-off cooling (-100 mC/s = -300 mC per 3s tick)
    let base = 38_000;
    for i in 1..=15 {
        kf.update_dt(base - i * 300, 3.0);
    }
    assert!(kf.rate() < -40.0 && kf.rate() > -160.0, "3s idle slope wrong: {}", kf.rate());

    // 45 s deep suspend gap -> must reset rate to 0 (no phantom spike on wake!)
    let wake = kf.update_dt(31_000, 45.0);
    assert_eq!(wake, 31_000);
    assert_eq!(kf.rate(), 0.0, "suspend resume must zero stale Kalman rate");
}

#[test]
fn sim_gaming_efficiency_ceilings_and_user_target() {
    let ceil = [1_900_800, 2_323_200, 2_323_200, 2_169_600];
    let mut ss = SsAlgo::new_simulated("gaming", HW_MAX_CPU, CPU_OPPS, false);
    let mut mon = MonitorAlgo::new_simulated("gaming", OPPS_GPU_HZ);
    let tick = |ss: &mut SsAlgo, mon: &mut MonitorAlgo, mc: i64, n: usize| {
        for _ in 0..n {
            ss.run_predictive_dt(mc, 0.0, 1.0);
            mon.set_skin_output(ss.skin_output(), ss.skin_gpu_floor_hz(), ss.skin_gpu_ceil_hz());
            mon.run(mc);
        }
    };

    // Cool phone: the knee, not hardware max.
    tick(&mut ss, &mut mon, 33_000, 5);
    assert_eq!(ss.applied_caps_khz(), ceil);
    assert_eq!(mon.applied_gpu_cap_hz(), 965_000_000);

    // Ceilings off: hardware max again.
    ss.set_ceilings(false);
    tick(&mut ss, &mut mon, 33_000, 5);
    assert_eq!(ss.applied_caps_khz(), HW_MAX_CPU);
    assert_eq!(mon.applied_gpu_cap_hz(), 1_162_000_000);
    ss.set_ceilings(true);
    tick(&mut ss, &mut mon, 33_000, 1);
    assert_eq!(ss.applied_caps_khz(), ceil);

    // 38.5 °C sits under the 39 °C default: no throttle once the step's rate brake has
    // recovered. A 38 °C user target throttles.
    tick(&mut ss, &mut mon, 38_500, 180);
    assert_eq!(ss.applied_caps_khz(), ceil);
    ss.set_game_target(38_000);
    tick(&mut ss, &mut mon, 38_500, 60);
    assert!(ss.applied_caps_khz()[1] < ceil[1], "user target ignored");
    assert!(ss.applied_caps_khz()[1] >= 1_190_400, "below the floor");

    // Hot: never under the floors, never over the ceilings.
    tick(&mut ss, &mut mon, 46_000, 300);
    let caps = ss.applied_caps_khz();
    assert!(caps[1] >= 1_190_400 && caps[1] <= ceil[1]);
    assert!(mon.applied_gpu_cap_hz() >= 515_000_000 && mon.applied_gpu_cap_hz() <= 965_000_000);

    // The daily profile has no ceilings and ignores the game target.
    let mut daily = SsAlgo::new_simulated("default", HW_MAX_CPU, CPU_OPPS, false);
    daily.set_game_target(36_000);
    for _ in 0..5 { daily.run_predictive_dt(38_000, 0.0, 1.0); }
    assert_eq!(daily.applied_caps_khz(), HW_MAX_CPU);
}
