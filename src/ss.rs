// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! SS (Step-wise Scaling) algorithm for CPU frequency throttling.
//!
//! Replaces the vendor mi_thermald SS algo with our own implementation.
//! ALL 15 vendor profiles fully decrypted from thermal-map.conf + 1 custom Battery Saver.
//! Profile selection via system property (set by GameSpace ThermalController).
//!
//! Design:
//! - Hardcoded profiles from decrypted vendor thermal-map.conf (zero file I/O at runtime)
//! - Zero heap allocation on hot path (profiles loaded at init/switch)
//! - Hysteresis via separate trigger/clear thresholds per level
//! - Profile switch via system property `persist.sys.mithermal.ss_profile`
//! - The default profile runs the skin-temperature loop (skin.rs) instead of its ladder

use std::time::Instant;

use log::{info, warn};
use rustutils::android::system_properties::PropertyWatcher;

use crate::io::SysfsFd;
use crate::skin::{SkinController, TARGET_MC};

/// Maximum number of SS levels per CPU cluster.
const MAX_SS_LEVELS: usize = 16;

/// Maximum number of CPU clusters.
const MAX_CLUSTERS: usize = 4;

/// A single SS threshold level.
#[derive(Clone, Copy)]
struct SsLevel {
    trig_mc: i64,
    clr_mc: i64,
    freq: i64,
}

/// Per-cluster SS state.
struct ClusterSs {
    fd: Option<SysfsFd>,
    hw_max_freq: i64,
    current_level: i32,
    /// Thermal cap from the ladder or the skin loop (kHz); 0 until first set.
    cap: i64,
    /// Optional user ceiling from Pentas (`persist.sys.mithermal.user_cpu_caps`), kHz; 0 = uncapped.
    user_cap: i64,
    /// Value last written to scaling_max_freq.
    written: i64,
    /// scaling_available_frequencies, ascending.
    steps: Vec<i64>,
    /// Skin-loop cap at output 0.
    floor: i64,
    /// Skin-loop cap at output 1 (kHz); 0 = hardware max.
    ceil: i64,
    levels: [SsLevel; MAX_SS_LEVELS],
    level_count: usize,
    name: &'static str,
}

impl ClusterSs {
    /// Writes `min(thermal_cap, user_cap)` (or `min(hw_max, user_cap)` while lifted) if changed.
    /// Single writer guarantees thermal protection is never overridden and user ceilings are never
    /// busted by thermal recovery or interaction lifts.
    fn apply(&mut self, lift: bool) {
        let thermal = if lift && self.hw_max_freq > 0 {
            self.hw_max_freq
        } else if self.cap > 0 {
            self.cap
        } else {
            self.hw_max_freq
        };
        let want = match (thermal > 0, self.user_cap > 0) {
            (true, true) => thermal.min(self.user_cap),
            (false, true) => self.user_cap,
            (true, false) => thermal,
            (false, false) => 0,
        };
        if want > 0 && want != self.written {
            if let Some(ref fd) = self.fd {
                fd.write_int(want);
            }
            self.written = want;
        }
    }

    /// Snap a user ceiling to the nearest supported OPP step (if steps were discovered).
    fn snap_freq(&self, req: i64) -> i64 {
        if req <= 0 || self.steps.is_empty() {
            return req.max(0);
        }
        let mut best = self.steps[0];
        let mut best_d = (best - req).abs();
        for &cand in &self.steps {
            let d = (cand - req).abs();
            if d < best_d {
                best_d = d;
                best = cand;
            }
        }
        best
    }

    /// Effective unrestricted maximum for this cluster (min of hardware max and user ceiling).
    fn effective_max(&self) -> i64 {
        if self.user_cap > 0 && self.hw_max_freq > 0 {
            self.hw_max_freq.min(self.user_cap)
        } else {
            self.hw_max_freq
        }
    }

    /// Skin-loop cap: `output` of the way from the floor to hardware max,
    /// snapped down to an available step.
    fn skin_cap(&self, output: f64) -> i64 {
        let top = if self.ceil > 0 { self.ceil.min(self.hw_max_freq) } else { self.hw_max_freq };
        if output >= 1.0 || top <= self.floor {
            return top;
        }
        let target = self.floor + ((top - self.floor) as f64 * output) as i64;
        match self.steps.iter().rev().find(|&&f| f <= target) {
            Some(&f) => f.max(self.floor),
            None => target,
        }
    }
}

/// A complete thermal profile.
struct ThermalProfile {
    name: &'static str,
    clusters: [ClusterLevels; MAX_CLUSTERS],
}

/// SS levels for one cluster within a profile.
struct ClusterLevels {
    levels: [SsLevel; MAX_SS_LEVELS],
    count: usize,
}

impl ClusterLevels {
    const fn empty() -> Self {
        ClusterLevels {
            levels: [SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 }; MAX_SS_LEVELS],
            count: 0,
        }
    }
}

// ================================================================
// Built-in profiles — ALL 15 decrypted from vendor thermal-map.conf
// + 1 custom Battery Saver profile
// ================================================================

/// Default (sconfig=0)
const PROFILE_GLOBAL: ThermalProfile = ThermalProfile {
    name: "Default",
    clusters: [
        // cpu0 (3 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 40000, clr_mc: 38000, freq: 1785600 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 1516800 },
                SsLevel { trig_mc: 48000, clr_mc: 46000, freq: 1075200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 3,
        },
        // cpu2 (13 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 2707200 },
                SsLevel { trig_mc: 32000, clr_mc: 30000, freq: 2323200 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 2073600 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1824000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1516800 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1401600 },
                SsLevel { trig_mc: 44000, clr_mc: 43000, freq: 1401600 },
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1401600 },
                SsLevel { trig_mc: 46000, clr_mc: 45000, freq: 1401600 },
                SsLevel { trig_mc: 47000, clr_mc: 46000, freq: 1286400 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 940800 },
                SsLevel { trig_mc: 49000, clr_mc: 48000, freq: 787200 },
                SsLevel { trig_mc: 50000, clr_mc: 49000, freq: 787200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 13,
        },
        // cpu5 (13 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 2707200 },
                SsLevel { trig_mc: 32000, clr_mc: 30000, freq: 2323200 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 2073600 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1824000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1401600 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1401600 },
                SsLevel { trig_mc: 44000, clr_mc: 43000, freq: 1401600 },
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1401600 },
                SsLevel { trig_mc: 46000, clr_mc: 45000, freq: 1401600 },
                SsLevel { trig_mc: 47000, clr_mc: 46000, freq: 1286400 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 940800 },
                SsLevel { trig_mc: 49000, clr_mc: 48000, freq: 787200 },
                SsLevel { trig_mc: 50000, clr_mc: 49000, freq: 480000 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 13,
        },
        // cpu7 (11 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 2745600 },
                SsLevel { trig_mc: 32000, clr_mc: 30000, freq: 2284800 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 2054400 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1785600 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1478400 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1363200 },
                SsLevel { trig_mc: 44000, clr_mc: 43000, freq: 1228800 },
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1094400 },
                SsLevel { trig_mc: 46000, clr_mc: 45000, freq: 1094400 },
                SsLevel { trig_mc: 47000, clr_mc: 46000, freq: 960000 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 787200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 11,
        },
    ],
};

/// Data Transfer (sconfig=1)
const PROFILE_HUANJI: ThermalProfile = ThermalProfile {
    name: "Data Transfer",
    clusters: [
        // cpu0 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 1190400 },
                SsLevel { trig_mc: 42500, clr_mc: 41000, freq: 614400 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu2 (9 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 13000, freq: 2073600 },
                SsLevel { trig_mc: 30000, clr_mc: 28000, freq: 1920000 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1824000 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1632000 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1401600 },
                SsLevel { trig_mc: 40000, clr_mc: 39000, freq: 1190400 },
                SsLevel { trig_mc: 41000, clr_mc: 40000, freq: 1056000 },
                SsLevel { trig_mc: 41500, clr_mc: 41000, freq: 940800 },
                SsLevel { trig_mc: 42500, clr_mc: 41500, freq: 787200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 9,
        },
        // cpu5 (9 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 13000, freq: 2208000 },
                SsLevel { trig_mc: 30000, clr_mc: 28000, freq: 2073600 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1920000 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1728000 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1632000 },
                SsLevel { trig_mc: 40000, clr_mc: 39000, freq: 1286400 },
                SsLevel { trig_mc: 41000, clr_mc: 40000, freq: 1190400 },
                SsLevel { trig_mc: 41500, clr_mc: 41000, freq: 940800 },
                SsLevel { trig_mc: 42500, clr_mc: 41500, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 9,
        },
        // cpu7 (9 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 13000, freq: 2400000 },
                SsLevel { trig_mc: 30000, clr_mc: 28000, freq: 2112000 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 2054400 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1920000 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1785600 },
                SsLevel { trig_mc: 40000, clr_mc: 39000, freq: 1478400 },
                SsLevel { trig_mc: 41000, clr_mc: 40000, freq: 1228800 },
                SsLevel { trig_mc: 41500, clr_mc: 41000, freq: 1094400 },
                SsLevel { trig_mc: 42500, clr_mc: 41500, freq: 787200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 9,
        },
    ],
};

/// Phone Call (sconfig=5)
const PROFILE_PHONE: ThermalProfile = ThermalProfile {
    name: "Phone Call",
    clusters: [
        // cpu0 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 1516800 },
                SsLevel { trig_mc: 48000, clr_mc: 46000, freq: 1190400 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu2 (3 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 10000, freq: 1516800 },
                SsLevel { trig_mc: 36000, clr_mc: 34000, freq: 1056000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 3,
        },
        // cpu5 (3 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 10000, freq: 1516800 },
                SsLevel { trig_mc: 36000, clr_mc: 34000, freq: 1056000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 3,
        },
        // cpu7 (3 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 10000, freq: 1478400 },
                SsLevel { trig_mc: 36000, clr_mc: 34000, freq: 1094400 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 3,
        },
    ],
};

/// Performance / No-Limit (sconfig=6) — true unrestricted mode for external coolers & benchmarks.
/// Hardware silicon protection remains active independently via kernel thermal_core (TSENS/LMh).
const PROFILE_NOLIMITS: ThermalProfile = ThermalProfile {
    name: "Performance",
    clusters: [
        ClusterLevels::empty(),
        ClusterLevels::empty(),
        ClusterLevels::empty(),
        ClusterLevels::empty(),
    ],
};

/// AR/VR (sconfig=9)
const PROFILE_ARVR: ThermalProfile = ThermalProfile {
    name: "AR/VR",
    clusters: [
        // cpu0 (1 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 1516800 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 1,
        },
        // cpu2 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1516800 },
                SsLevel { trig_mc: 52000, clr_mc: 50000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu5 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1516800 },
                SsLevel { trig_mc: 52000, clr_mc: 50000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu7 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1785600 },
                SsLevel { trig_mc: 52000, clr_mc: 50000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
    ],
};

/// Navigation (sconfig=10)
const PROFILE_NAVIGATION: ThermalProfile = ThermalProfile {
    name: "Navigation",
    clusters: [
        // cpu0 (4 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1516800 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1401600 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1286400 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 1190400 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 4,
        },
        // cpu2 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 2073600 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1401600 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1286400 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1056000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 940800 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 787200 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
        // cpu5 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 2073600 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1401600 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1286400 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1056000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 940800 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 787200 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
        // cpu7 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 2054400 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1478400 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1228800 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1094400 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 960000 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 960000 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
    ],
};

/// Video (sconfig=11)
const PROFILE_VIDEO: ThermalProfile = ThermalProfile {
    name: "Video",
    clusters: [
        // cpu0 (3 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 40000, clr_mc: 38000, freq: 1785600 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 1401600 },
                SsLevel { trig_mc: 48000, clr_mc: 46000, freq: 1190400 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 3,
        },
        // cpu2 (12 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 2707200 },
                SsLevel { trig_mc: 32000, clr_mc: 30000, freq: 2208000 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1728000 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1401600 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1401600 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1401600 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1286400 },
                SsLevel { trig_mc: 44000, clr_mc: 43000, freq: 1286400 },
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1190400 },
                SsLevel { trig_mc: 46000, clr_mc: 45000, freq: 1190400 },
                SsLevel { trig_mc: 47000, clr_mc: 46000, freq: 1056000 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 787200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 12,
        },
        // cpu5 (12 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 2707200 },
                SsLevel { trig_mc: 32000, clr_mc: 30000, freq: 2208000 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1728000 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1401600 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1401600 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1286400 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1286400 },
                SsLevel { trig_mc: 44000, clr_mc: 43000, freq: 1286400 },
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1190400 },
                SsLevel { trig_mc: 46000, clr_mc: 45000, freq: 1190400 },
                SsLevel { trig_mc: 47000, clr_mc: 46000, freq: 1056000 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 787200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 12,
        },
        // cpu7 (12 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 10000, freq: 2745600 },
                SsLevel { trig_mc: 32000, clr_mc: 30000, freq: 2112000 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1785600 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1478400 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1478400 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1363200 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1228800 },
                SsLevel { trig_mc: 44000, clr_mc: 43000, freq: 1228800 },
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1094400 },
                SsLevel { trig_mc: 46000, clr_mc: 45000, freq: 1094400 },
                SsLevel { trig_mc: 47000, clr_mc: 46000, freq: 787200 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 12,
        },
    ],
};

/// Video Chat (sconfig=14)
const PROFILE_VIDEOCHAT: ThermalProfile = ThermalProfile {
    name: "Video Chat",
    clusters: [
        // cpu0 (3 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 40000, clr_mc: 38000, freq: 1785600 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 1516800 },
                SsLevel { trig_mc: 48000, clr_mc: 46000, freq: 1190400 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 3,
        },
        // cpu2 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 20000, freq: 2073600 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1728000 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1401600 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1190400 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 940800 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 787200 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
        // cpu5 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 20000, freq: 2073600 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1728000 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1401600 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1190400 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 787200 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 787200 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
        // cpu7 (8 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 20000, freq: 2112000 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1785600 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1478400 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1363200 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1228800 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1094400 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 960000 },
                SsLevel { trig_mc: 48000, clr_mc: 46000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 8,
        },
    ],
};

/// Camera (sconfig=15)
const PROFILE_CAMERA: ThermalProfile = ThermalProfile {
    name: "Camera",
    clusters: [
        // cpu0 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 40000, clr_mc: 38000, freq: 1401600 },
                SsLevel { trig_mc: 48000, clr_mc: 46000, freq: 1190400 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu2 (6 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 13000, freq: 2707200 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1920000 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1824000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1632000 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1401600 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 6,
        },
        // cpu5 (6 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 13000, freq: 2707200 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1920000 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1824000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1632000 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1401600 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 6,
        },
        // cpu7 (6 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 13000, freq: 2956800 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1920000 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1785600 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1478400 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1228800 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 787200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 6,
        },
    ],
};

/// 4K Recording (sconfig=16)
const PROFILE_4K: ThermalProfile = ThermalProfile {
    name: "4K Recording",
    clusters: [
        // cpu0 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 40000, clr_mc: 38000, freq: 1286400 },
                SsLevel { trig_mc: 48000, clr_mc: 46000, freq: 1190400 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu2 (6 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 13000, freq: 2707200 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1920000 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1824000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1632000 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1401600 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 6,
        },
        // cpu5 (6 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 13000, freq: 2707200 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1920000 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1824000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1632000 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1401600 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 6,
        },
        // cpu7 (6 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 13000, freq: 3052800 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1920000 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1785600 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1478400 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1228800 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 787200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 6,
        },
    ],
};

/// Heavy Gaming (sconfig=18)
const PROFILE_TGAME: ThermalProfile = ThermalProfile {
    name: "Heavy Gaming",
    clusters: [
        // cpu0 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1516800 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 1075200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu2 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 43500, clr_mc: 42500, freq: 2208000 },
                SsLevel { trig_mc: 44500, clr_mc: 43500, freq: 2073600 },
                SsLevel { trig_mc: 45200, clr_mc: 44500, freq: 1920000 },
                SsLevel { trig_mc: 45900, clr_mc: 45200, freq: 1728000 },
                SsLevel { trig_mc: 46600, clr_mc: 45900, freq: 1516800 },
                SsLevel { trig_mc: 47300, clr_mc: 46600, freq: 1286400 },
                SsLevel { trig_mc: 48000, clr_mc: 47300, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
        // cpu5 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 43500, clr_mc: 42500, freq: 2208000 },
                SsLevel { trig_mc: 44500, clr_mc: 43500, freq: 2073600 },
                SsLevel { trig_mc: 45200, clr_mc: 44500, freq: 1920000 },
                SsLevel { trig_mc: 45900, clr_mc: 45200, freq: 1728000 },
                SsLevel { trig_mc: 46600, clr_mc: 45900, freq: 1516800 },
                SsLevel { trig_mc: 47300, clr_mc: 46600, freq: 1286400 },
                SsLevel { trig_mc: 48000, clr_mc: 47300, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
        // cpu7 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 43500, clr_mc: 42500, freq: 2400000 },
                SsLevel { trig_mc: 44500, clr_mc: 43500, freq: 2054400 },
                SsLevel { trig_mc: 45200, clr_mc: 44500, freq: 1920000 },
                SsLevel { trig_mc: 45900, clr_mc: 45200, freq: 1785600 },
                SsLevel { trig_mc: 46600, clr_mc: 45900, freq: 1478400 },
                SsLevel { trig_mc: 47300, clr_mc: 46600, freq: 1228800 },
                SsLevel { trig_mc: 48000, clr_mc: 47300, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
    ],
};

/// Balanced Gaming (sconfig=19)
const PROFILE_MGAME: ThermalProfile = ThermalProfile {
    name: "Balanced Gaming",
    clusters: [
        // cpu0 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 43000, clr_mc: 42000, freq: 1516800 },
                SsLevel { trig_mc: 46000, clr_mc: 44000, freq: 1075200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu2 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 36000, clr_mc: 34000, freq: 2208000 },
                SsLevel { trig_mc: 42500, clr_mc: 41500, freq: 2073600 },
                SsLevel { trig_mc: 43200, clr_mc: 42500, freq: 1920000 },
                SsLevel { trig_mc: 43900, clr_mc: 43200, freq: 1728000 },
                SsLevel { trig_mc: 44600, clr_mc: 43900, freq: 1516800 },
                SsLevel { trig_mc: 45300, clr_mc: 44600, freq: 1286400 },
                SsLevel { trig_mc: 46000, clr_mc: 45300, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
        // cpu5 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 36000, clr_mc: 34000, freq: 2323200 },
                SsLevel { trig_mc: 42500, clr_mc: 41500, freq: 2073600 },
                SsLevel { trig_mc: 43200, clr_mc: 42500, freq: 1920000 },
                SsLevel { trig_mc: 43900, clr_mc: 43200, freq: 1728000 },
                SsLevel { trig_mc: 44600, clr_mc: 43900, freq: 1516800 },
                SsLevel { trig_mc: 45300, clr_mc: 44600, freq: 1286400 },
                SsLevel { trig_mc: 46000, clr_mc: 45300, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
        // cpu7 (7 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 36000, clr_mc: 34000, freq: 2400000 },
                SsLevel { trig_mc: 42500, clr_mc: 41500, freq: 2054400 },
                SsLevel { trig_mc: 43200, clr_mc: 42500, freq: 1920000 },
                SsLevel { trig_mc: 43900, clr_mc: 43200, freq: 1785600 },
                SsLevel { trig_mc: 44600, clr_mc: 43900, freq: 1670400 },
                SsLevel { trig_mc: 45300, clr_mc: 44600, freq: 1478400 },
                SsLevel { trig_mc: 46000, clr_mc: 45300, freq: 787200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
    ],
};

/// Genshin Impact (sconfig=20)
const PROFILE_YUANSHEN: ThermalProfile = ThermalProfile {
    name: "Genshin Impact",
    clusters: [
        // cpu0 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1516800 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 1075200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu2 (1 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 48000, clr_mc: 47300, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 1,
        },
        // cpu5 (1 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 48000, clr_mc: 47300, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 1,
        },
        // cpu7 (1 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 48000, clr_mc: 47300, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 1,
        },
    ],
};

/// Honkai Star Rail (sconfig=25)
const PROFILE_XINGTIE: ThermalProfile = ThermalProfile {
    name: "Honkai Star Rail",
    clusters: [
        // cpu0 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1516800 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 1075200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu2 (1 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 50000, clr_mc: 49000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 1,
        },
        // cpu5 (1 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 50000, clr_mc: 49000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 1,
        },
        // cpu7 (1 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 50000, clr_mc: 49000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 1,
        },
    ],
};

/// High FPS (sconfig=26)
const PROFILE_HIGHFPS: ThermalProfile = ThermalProfile {
    name: "High FPS",
    clusters: [
        // cpu0 (2 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 40000, clr_mc: 38000, freq: 1516800 },
                SsLevel { trig_mc: 48000, clr_mc: 46000, freq: 1190400 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 2,
        },
        // cpu2 (8 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 2323200 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1516800 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1286400 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1190400 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1056000 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1056000 },
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1056000 },
                SsLevel { trig_mc: 46000, clr_mc: 45000, freq: 940800 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 8,
        },
        // cpu5 (8 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 25000, clr_mc: 23000, freq: 2323200 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1516800 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1286400 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1190400 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1056000 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1056000 },
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 1056000 },
                SsLevel { trig_mc: 46000, clr_mc: 45000, freq: 940800 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 8,
        },
        // cpu7 (11 levels)
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 15000, clr_mc: 10000, freq: 2054400 },
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1478400 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1363200 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1248000 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 1132800 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 1017600 },
                SsLevel { trig_mc: 44000, clr_mc: 43000, freq: 902400 },
                SsLevel { trig_mc: 45000, clr_mc: 44000, freq: 787200 },
                SsLevel { trig_mc: 46000, clr_mc: 45000, freq: 672000 },
                SsLevel { trig_mc: 47000, clr_mc: 46000, freq: 576000 },
                SsLevel { trig_mc: 48000, clr_mc: 47000, freq: 480000 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 11,
        },
    ],
};

/// Battery Saver — very aggressive throttle (custom, not from vendor)
const PROFILE_BATTERY_SAVER: ThermalProfile = ThermalProfile {
    name: "Battery Saver",
    clusters: [
        // cpu0
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 35000, clr_mc: 33000, freq: 1516800 },
                SsLevel { trig_mc: 40000, clr_mc: 38000, freq: 1075200 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 787200 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 3,
        },
        // cpu2
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 30000, clr_mc: 28000, freq: 2073600 },
                SsLevel { trig_mc: 34000, clr_mc: 32000, freq: 1824000 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1516800 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1286400 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 940800 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 787200 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 633600 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
        // cpu5
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 30000, clr_mc: 28000, freq: 2073600 },
                SsLevel { trig_mc: 34000, clr_mc: 32000, freq: 1824000 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1401600 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1075200 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 787200 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 633600 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 480000 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
        // cpu7
        ClusterLevels {
            levels: [
                SsLevel { trig_mc: 30000, clr_mc: 28000, freq: 2054400 },
                SsLevel { trig_mc: 34000, clr_mc: 32000, freq: 1785600 },
                SsLevel { trig_mc: 37000, clr_mc: 35000, freq: 1478400 },
                SsLevel { trig_mc: 39000, clr_mc: 37000, freq: 1094400 },
                SsLevel { trig_mc: 41000, clr_mc: 39000, freq: 960000 },
                SsLevel { trig_mc: 43000, clr_mc: 41000, freq: 787200 },
                SsLevel { trig_mc: 45000, clr_mc: 43000, freq: 614400 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
                SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 },
            ],
            count: 7,
        },
    ],
};

// ================================================================
// Profile lookup + SS Algorithm
// ================================================================

/// Property for active profile name (set by GameSpace ThermalController).
const PROP_ACTIVE_PROFILE: &str = "persist.sys.mithermal.ss_profile";

const PROFILE_NAME_DEFAULT: &str = "default";

fn get_profile(name: &str) -> &'static ThermalProfile {
    match name {
        "default" => &PROFILE_GLOBAL,
        "data_transfer" => &PROFILE_HUANJI,
        "phone_call" => &PROFILE_PHONE,
        "performance" => &PROFILE_NOLIMITS,
        "arvr" => &PROFILE_ARVR,
        "navigation" => &PROFILE_NAVIGATION,
        "video" => &PROFILE_VIDEO,
        "video_chat" => &PROFILE_VIDEOCHAT,
        "camera" => &PROFILE_CAMERA,
        "recording_4k" => &PROFILE_4K,
        "heavy_gaming" => &PROFILE_TGAME,
        "gaming" => &PROFILE_MGAME,
        "genshin" => &PROFILE_YUANSHEN,
        "hsr" => &PROFILE_XINGTIE,
        "high_fps" => &PROFILE_HIGHFPS,
        "battery_saver" => &PROFILE_BATTERY_SAVER,
        _ => {
            warn!("Unknown profile '{}', using default", name);
            &PROFILE_GLOBAL
        }
    }
}

/// CPU scaling_max_freq paths for each cluster.
const CPU_SCALING_PATHS: [&str; MAX_CLUSTERS] = [
    "/sys/devices/system/cpu/cpufreq/policy0/scaling_max_freq",
    "/sys/devices/system/cpu/cpufreq/policy2/scaling_max_freq",
    "/sys/devices/system/cpu/cpufreq/policy5/scaling_max_freq",
    "/sys/devices/system/cpu/cpufreq/policy7/scaling_max_freq",
];

const CPU_HWMAX_PATHS: [&str; MAX_CLUSTERS] = [
    "/sys/devices/system/cpu/cpufreq/policy0/cpuinfo_max_freq",
    "/sys/devices/system/cpu/cpufreq/policy2/cpuinfo_max_freq",
    "/sys/devices/system/cpu/cpufreq/policy5/cpuinfo_max_freq",
    "/sys/devices/system/cpu/cpufreq/policy7/cpuinfo_max_freq",
];

const CPU_STEPS_PATHS: [&str; MAX_CLUSTERS] = [
    "/sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies",
    "/sys/devices/system/cpu/cpufreq/policy2/scaling_available_frequencies",
    "/sys/devices/system/cpu/cpufreq/policy5/scaling_available_frequencies",
    "/sys/devices/system/cpu/cpufreq/policy7/scaling_available_frequencies",
];

const CLUSTER_NAMES: [&str; MAX_CLUSTERS] = ["cpu0", "cpu2", "cpu5", "cpu7"];

/// Continuous PI skin-loop parameters for daily and bare-hand gaming profiles.
/// Replaces rigid Xiaomi Joyose-cliff step-ladders (`48°C -> 633 MHz`) with smooth
/// asymmetric Prime-first throttling tailored to SM8735's 2+3+2 A720 + 1× X4 silicon.
#[derive(Clone, Copy)]
struct SkinProfile {
    target_mc: i64,
    floors_khz: [i64; MAX_CLUSTERS],
    cluster_exp: [f64; MAX_CLUSTERS],
    /// Caps at output 1; 0 = hardware max. Set at the efficiency knee so a cool
    /// phone does not burn the headroom on clocks that cost more than they give.
    ceil_khz: [i64; MAX_CLUSTERS],
    gpu_floor_hz: i64,
    gpu_ceil_hz: i64,
    lifts_on_interaction: bool,
    /// Follows `persist.sys.mithermal.game_target` when that is set.
    game: bool,
}

const SKIN_DEFAULT: SkinProfile = SkinProfile {
    target_mc: TARGET_MC, // 42.0°C
    floors_khz: [1_075_200, 787_200, 787_200, 787_200],
    cluster_exp: [0.55, 0.85, 1.00, 1.45],
    ceil_khz: [0; MAX_CLUSTERS],
    gpu_floor_hz: 515_000_000,
    gpu_ceil_hz: 0,
    game: false,
    lifts_on_interaction: true,
};

// Ceilings sit at the efficiency knee measured on onyx (fixed load, whole-device power):
// above them each cluster pays 2-3x the watts per extra MHz. Floors leave the loop room to
// reach the target: MLBB held 119 FPS with no jank at ~600 MHz GPU, and 685 MHz costs about
// twice the GPU watts of 515.
const SKIN_BALANCED_GAMING: SkinProfile = SkinProfile {
    target_mc: 39_000,
    floors_khz: [1_075_200, 1_190_400, 1_190_400, 1_094_400],
    cluster_exp: [0.45, 0.80, 0.95, 1.50],
    ceil_khz: [1_900_800, 2_323_200, 2_323_200, 2_169_600],
    gpu_floor_hz: 515_000_000,
    gpu_ceil_hz: 965_000_000,
    game: true,
    lifts_on_interaction: false,
};

const SKIN_HIGH_FPS: SkinProfile = SkinProfile {
    target_mc: 44_500, // 44.5°C — protects 5× A720 for rock-steady 120 FPS MOBA/FPS
    floors_khz: [1_286_400, 1_516_800, 1_401_600, 1_228_800],
    cluster_exp: [0.40, 0.70, 0.80, 1.55],
    ceil_khz: [0; MAX_CLUSTERS],
    gpu_floor_hz: 685_000_000,
    gpu_ceil_hz: 0,
    game: true,
    lifts_on_interaction: false,
};

const SKIN_HEAVY_GAMING: SkinProfile = SkinProfile {
    target_mc: 45_000, // 45.0°C — smooth Genshin / Heavy Gaming equilibrium (zero 633 MHz cliff)
    floors_khz: [1_401_600, 1_516_800, 1_401_600, 1_363_200],
    cluster_exp: [0.45, 0.80, 0.95, 1.50],
    ceil_khz: [0; MAX_CLUSTERS],
    gpu_floor_hz: 685_000_000,
    gpu_ceil_hz: 0,
    game: true,
    lifts_on_interaction: false,
};

const SKIN_HSR: SkinProfile = SkinProfile {
    target_mc: 45_500, // 45.5°C — GPU-heavy shader titles (Honkai: Star Rail)
    floors_khz: [1_401_600, 1_516_800, 1_401_600, 1_363_200],
    cluster_exp: [0.45, 0.80, 0.95, 1.50],
    ceil_khz: [0; MAX_CLUSTERS],
    gpu_floor_hz: 763_000_000,
    gpu_ceil_hz: 0,
    game: true,
    lifts_on_interaction: false,
};

/// Hard safety floor if a bare-hand PI profile still exceeds `SKIN_EMERGENCY_MC` (e.g. 40°C sun).
const SKIN_EMERGENCY_MC: i64 = 50_000;
const SKIN_EMERGENCY_FLOOR_KHZ: [i64; MAX_CLUSTERS] = [1_075_200, 940_800, 940_800, 787_200];

/// Return the smooth PI skin profile for `name`, or `None` for ladder / no-limit / custom profiles.
fn skin_profile_for(name: &str) -> Option<SkinProfile> {
    match name {
        "default" => Some(SKIN_DEFAULT),
        "gaming" => Some(SKIN_BALANCED_GAMING),
        "high_fps" => Some(SKIN_HIGH_FPS),
        "heavy_gaming" | "genshin" => Some(SKIN_HEAVY_GAMING),
        "hsr" => Some(SKIN_HSR),
        _ => None,
    }
}

/// Predictive horizon (seconds) for the Kalman-rate look-ahead in run_predictive.
const PREDICT_HORIZON_S: f64 = 4.0;

/// Max GPU throttle levels a custom profile may define (matches monitor).
pub const CUSTOM_GPU_MAX: usize = 4;

// Custom-profile side-channel props (set by GameSpace alongside the clusters
// chunks). All are re-read on a custom-seq bump. CSV formats:
//   custom.gpu: "trigMc,clrMc,freqMHz;…"   (up to CUSTOM_GPU_MAX; MHz keeps it
//                                            under the 88-byte prop cap)
//   custom.mon: "boostTrig,hotplugTrig,blTrig,blCap"  (mC; 0 = no limit)
//   custom.sic: "targetMc,maxFccUa"                   (0 = use global config)
const PROP_CUSTOM_GPU: &str = "persist.sys.mithermal.custom.gpu";
const PROP_CUSTOM_MON: &str = "persist.sys.mithermal.custom.mon";
const PROP_CUSTOM_SIC: &str = "persist.sys.mithermal.custom.sic";

/// Extended custom-profile knobs beyond the CPU cluster levels (Features A/C/D).
/// Every zero field means "unset" → the corresponding layer stays unrestricted.
pub struct CustomExtras {
    // Feature A — GPU throttle curve. gpu_freq_hz is the target GPU max freq (Hz)
    // to snap to at each level.
    pub gpu_trig: [i64; CUSTOM_GPU_MAX],
    pub gpu_clr: [i64; CUSTOM_GPU_MAX],
    pub gpu_freq_hz: [i64; CUSTOM_GPU_MAX],
    pub gpu_count: usize,
    // Feature C — monitor thresholds (mC). 0 = never engage (no limit).
    pub boost_trig: i64,
    pub hotplug_trig: i64,
    pub bl_trig: i64,
    pub bl_cap: i64,
    // Feature D — SIC overrides. 0 = fall back to the global config value.
    pub sic_target_mc: i64,
    pub sic_max_fcc_ua: i64,
}

impl CustomExtras {
    fn new() -> Self {
        CustomExtras {
            gpu_trig: [0; CUSTOM_GPU_MAX],
            gpu_clr: [0; CUSTOM_GPU_MAX],
            gpu_freq_hz: [0; CUSTOM_GPU_MAX],
            gpu_count: 0,
            boost_trig: 0,
            hotplug_trig: 0,
            bl_trig: 0,
            bl_cap: 0,
            sic_target_mc: 0,
            sic_max_fcc_ua: 0,
        }
    }

    /// True when nothing extra is configured (pure CPU-only custom profile).
    fn is_empty(&self) -> bool {
        self.gpu_count == 0
            && self.boost_trig == 0
            && self.hotplug_trig == 0
            && self.bl_trig == 0
            && self.sic_target_mc == 0
            && self.sic_max_fcc_ua == 0
    }
}

/// Main SS algo state.
pub struct SsAlgo {
    clusters: [ClusterSs; MAX_CLUSTERS],
    active_profile_name: String,
    check_counter: u32,
    /// Heap-allocated custom profile (only used when profile = "custom")
    custom_profile: Option<Box<ThermalProfile>>,
    /// Extended custom knobs (GPU curve / monitor / SIC) — None for built-ins
    /// and for CPU-only custom profiles.
    custom_extras: Option<CustomExtras>,
    /// Last applied custom-profile sequence (persist.sys.mithermal.custom.seq).
    custom_seq: i64,
    /// Set whenever the active profile config changes (built-in switch, custom
    /// (re)load). Consumed by the caller to re-sync monitor theme + SIC.
    config_dirty: bool,
    profile_watcher: Option<PropertyWatcher>,
    /// Present while the continuous PI skin loop owns the caps (daily & bare-hand gaming).
    skin: Option<(SkinController, SkinProfile)>,
    skin_output: f64,
    skin_logged: f64,
    last_run: Option<Instant>,
    /// Caps lifted to hardware max for an interaction.
    lift: bool,
    /// User skin target for game profiles (mC); 0 = the profile's own.
    game_target_mc: i64,
    /// Efficiency ceilings of the skin profiles in force.
    ceilings: bool,
}

impl SsAlgo {
    pub fn new() -> Self {
        let cluster_init = || ClusterSs {
            fd: None,
            hw_max_freq: 0,
            current_level: -1,
            cap: 0,
            user_cap: 0,
            written: 0,
            steps: Vec::new(),
            floor: 0,
            ceil: 0,
            levels: [SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 }; MAX_SS_LEVELS],
            level_count: 0,
            name: "",
        };
        let mut clusters: [ClusterSs; MAX_CLUSTERS] = [
            cluster_init(), cluster_init(), cluster_init(), cluster_init(),
        ];

        for (i, cluster) in clusters.iter_mut().enumerate() {
            let scaling_path: &'static str = CPU_SCALING_PATHS[i];
            cluster.fd = SysfsFd::open_rw(scaling_path);

            let hwmax_path: &'static str = CPU_HWMAX_PATHS[i];
            cluster.hw_max_freq = SysfsFd::open_ro(hwmax_path)
                .and_then(|fd| fd.read_int())
                .unwrap_or(0);

            cluster.name = CLUSTER_NAMES[i];
            cluster.floor = SKIN_DEFAULT.floors_khz[i];
            cluster.steps = std::fs::read_to_string(CPU_STEPS_PATHS[i])
                .map(|s| s.split_whitespace().filter_map(|t| t.parse().ok()).collect())
                .unwrap_or_default();
            cluster.steps.sort_unstable();

            info!("SS {}: hw_max={}kHz fd={}",
                  cluster.name, cluster.hw_max_freq,
                  if cluster.fd.is_some() { "OK" } else { "FAIL" });
        }

        let profile_name = match rustutils::android::system_properties::read(PROP_ACTIVE_PROFILE) {
            Ok(Some(val)) if !val.is_empty() => val,
            _ => PROFILE_NAME_DEFAULT.to_string(),
        };
        let profile = get_profile(&profile_name);
        info!("SS: initial profile = '{}' ({})", profile_name, profile.name);

        let skin = skin_profile_for(&profile_name).map(|sp| (SkinController::new(), sp));
        let mut algo = SsAlgo {
            clusters,
            active_profile_name: profile_name,
            check_counter: 0,
            custom_profile: None,
            custom_extras: None,
            custom_seq: -1,
            config_dirty: false,
            // A name without interior NULs cannot fail.
            profile_watcher: PropertyWatcher::new(PROP_ACTIVE_PROFILE).ok(),
            skin,
            skin_output: 1.0,
            skin_logged: 1.0,
            last_run: None,
            lift: false,
            game_target_mc: 0,
            ceilings: true,
        };

        algo.apply_profile(profile);
        algo.sync_skin_floors();
        algo
    }

    fn sync_skin_floors(&mut self) {
        let skin = self.skin.as_ref().map(|(_, sp)| *sp);
        let ceilings = self.ceilings;
        for (i, cluster) in self.clusters.iter_mut().enumerate() {
            match skin {
                Some(sp) => {
                    cluster.floor = sp.floors_khz[i];
                    cluster.ceil = if ceilings { sp.ceil_khz[i] } else { 0 };
                }
                None => cluster.ceil = 0,
            }
        }
    }

    fn apply_profile(&mut self, profile: &ThermalProfile) {
        for (i, cluster) in self.clusters.iter_mut().enumerate() {
            let src = &profile.clusters[i];
            cluster.levels = src.levels;
            cluster.level_count = src.count;
            cluster.current_level = -1;
            if src.count == 0 {
                cluster.cap = cluster.hw_max_freq;
                cluster.apply(self.lift);
            }
        }
        // A built-in profile carries no extended knobs; drop any custom extras.
        self.custom_extras = None;
        self.config_dirty = true;
    }

    /// Reactive convenience wrapper (rate = 0). Kept for the fallback path and
    /// tests; the hot path calls `maybe_switch_profile()` + `run_predictive()`
    /// separately so the monitor theme can be synced between the two.
    #[allow(dead_code)]
    pub fn run(&mut self, vsensor0_mc: i64) {
        self.maybe_switch_profile();
        self.run_predictive(vsensor0_mc, 0.0);
    }

    /// Profile management: every call, compare the active-profile property and
    /// (re)load the profile if it changed, so leaving a game hands the caps back
    /// within a cycle. Split out of the level evaluation so the caller can sync
    /// the monitor theme BEFORE the CPU levels are applied (Phase 1.4 — closes
    /// the 1-cycle theme/profile skew window).
    ///
    /// Heap allocation only occurs on an actual profile change (rare).
    pub fn maybe_switch_profile(&mut self) {
        if let Some(new_name) = self.profile_prop_if_changed() {
            if new_name == "custom" {
                self.load_custom_profile();
            } else {
                let profile = get_profile(&new_name);
                info!("SS: profile switch '{}' -> '{}' ({})",
                      self.active_profile_name, new_name, profile.name);
                self.apply_profile(profile);
                self.custom_profile = None;
            }
            self.skin = skin_profile_for(&new_name).map(|sp| (SkinController::new(), sp));
            self.sync_skin_floors();
            self.skin_output = 1.0;
            self.skin_logged = 1.0;
            self.active_profile_name = new_name;
            return;
        }

        self.check_counter += 1;
        if self.check_counter < 5 {
            return;
        }
        self.check_counter = 0;
        if self.active_profile_name == "custom"
            && read_prop_i64(PROP_CUSTOM_SEQ).unwrap_or(-1) != self.custom_seq
        {
            // Custom profile edited in place — GameSpace bumped the seq
            self.load_custom_profile();
        }
    }

    /// The active-profile property when it differs from the running profile.
    /// Compared in place: a String is only made for an actual change.
    fn profile_prop_if_changed(&mut self) -> Option<String> {
        let watcher = self.profile_watcher.as_mut()?;
        let current = self.active_profile_name.as_str();
        let changed = match watcher.read(|_, v| effective_profile(v) != current) {
            Ok(changed) => changed,
            // Absent property → "default"
            Err(_) => current != PROFILE_NAME_DEFAULT,
        };
        if !changed {
            return None;
        }
        match watcher.read(|_, v| effective_profile(v).to_string()) {
            Ok(name) => Some(name),
            Err(_) => Some(PROFILE_NAME_DEFAULT.to_string()),
        }
    }

    pub fn run_predictive(&mut self, vsensor0_mc: i64, rate_mc_per_s: f64) {
        let now = Instant::now();
        let dt = self.last_run
            .map_or(1.0, |t| now.duration_since(t).as_secs_f64())
            .clamp(0.1, 5.0);
        self.last_run = Some(now);
        self.run_predictive_dt(vsensor0_mc, rate_mc_per_s, dt);
    }

    /// Core predictive / PI thermal evaluation with explicit elapsed `dt` (seconds).
    /// Split out so closed-loop simulations exercise the exact production control math.
    pub fn run_predictive_dt(&mut self, vsensor0_mc: i64, rate_mc_per_s: f64, dt: f64) {
        let lift = self.lift;

        if let Some((skin, sp)) = self.skin.as_mut() {
            let target_mc = if sp.game && self.game_target_mc > 0 { self.game_target_mc } else { sp.target_mc };
            let out = skin.update(vsensor0_mc, target_mc, dt);
            let emergency = vsensor0_mc >= SKIN_EMERGENCY_MC;
            for (i, cluster) in self.clusters.iter_mut().enumerate() {
                let cluster_out = if out >= 1.0 {
                    1.0
                } else if out <= 0.0 {
                    0.0
                } else {
                    out.powf(sp.cluster_exp[i])
                };
                cluster.cap = if emergency {
                    SKIN_EMERGENCY_FLOOR_KHZ[i]
                } else {
                    cluster.skin_cap(cluster_out)
                };
                cluster.apply(lift);
            }
            self.skin_output = out;
            if (out - self.skin_logged).abs() >= 0.1 || (out == 1.0) != (self.skin_logged == 1.0) {
                info!("Skin({}): {:.1}°C/{:.1}°C → output {:.2}, caps {}/{}/{}/{} kHz",
                      self.active_profile_name,
                      vsensor0_mc as f64 / 1000.0,
                      target_mc as f64 / 1000.0,
                      out,
                      self.clusters[0].cap, self.clusters[1].cap,
                      self.clusters[2].cap, self.clusters[3].cap);
                self.skin_logged = out;
            }
            return;
        }

        // Sanitise the rate: NaN/±inf → 0, and clamp to a physically sane band
        // so the look-ahead can never overflow when added to the base temp.
        let rate = if rate_mc_per_s.is_finite() {
            rate_mc_per_s.clamp(-60_000.0, 60_000.0)
        } else {
            0.0
        };
        let predicted = vsensor0_mc.saturating_add((rate * PREDICT_HORIZON_S) as i64);
        let trigger_temp = vsensor0_mc.max(predicted); // aggressive engage
        let clear_temp = vsensor0_mc;                   // conservative release

        for cluster in self.clusters.iter_mut() {
            if cluster.level_count == 0 {
                continue;
            }

            let new_level = compute_ss_level_predictive(
                &cluster.levels[..cluster.level_count],
                cluster.current_level,
                trigger_temp,
                clear_temp,
            );

            let target_freq = if new_level >= 0 {
                cluster.levels[new_level as usize].freq
            } else {
                cluster.hw_max_freq
            };

            if target_freq != cluster.cap && target_freq > 0 {
                if new_level != cluster.current_level {
                    info!("SS {}: {:.1}°C (pred {:.1}°C) → level {} freq={}kHz (was level {} {}kHz)",
                          cluster.name,
                          vsensor0_mc as f64 / 1000.0,
                          trigger_temp as f64 / 1000.0,
                          new_level,
                          target_freq,
                          cluster.current_level,
                          cluster.cap);
                }
                cluster.cap = target_freq;
            }
            cluster.apply(lift);

            cluster.current_level = new_level;
        }
    }

    #[allow(dead_code)]
    pub fn restore_all(&mut self) {
        for cluster in self.clusters.iter_mut() {
            if cluster.hw_max_freq > 0 {
                cluster.cap = cluster.hw_max_freq;
                cluster.apply(false);
                cluster.current_level = -1;
            }
        }
        info!("SS: all clusters restored to hw max");
    }

    /// Skin target for game profiles from the user (mC); 0 restores each profile's own.
    pub fn set_game_target(&mut self, target_mc: i64) {
        self.game_target_mc = target_mc;
    }

    /// Lifts every cluster to hardware max (an interaction) or back to its cap.
    pub fn set_lift(&mut self, lift: bool) {
        self.lift = lift;
        for cluster in self.clusters.iter_mut() {
            cluster.apply(lift);
        }
    }

    /// True when the active profile wants its caps lifted for interactions:
    /// the daily-use profile only (games keep smooth steady caps under continuous touch).
    pub fn lifts_on_interaction(&self) -> bool {
        self.skin.as_ref().is_some_and(|(_, sp)| sp.lifts_on_interaction)
    }

    /// Active skin-loop target temperature (mC), or `TARGET_MC` when ladder-controlled.
    pub fn skin_target_mc(&self) -> i64 {
        self.skin.as_ref().map_or(TARGET_MC, |(_, sp)| sp.target_mc)
    }

    /// Active skin-loop GPU floor (Hz), for demand-aware GPU scaling in `MonitorAlgo`.
    pub fn skin_gpu_floor_hz(&self) -> i64 {
        self.skin.as_ref().map_or(SKIN_DEFAULT.gpu_floor_hz, |(_, sp)| sp.gpu_floor_hz)
    }

    /// GPU cap the skin loop allows at output 1 (Hz); 0 = hardware max.
    pub fn skin_gpu_ceil_hz(&self) -> i64 {
        if !self.ceilings {
            return 0;
        }
        self.skin.as_ref().map_or(0, |(_, sp)| sp.gpu_ceil_hz)
    }

    /// Turns the skin profiles' efficiency ceilings on or off (off = hardware max at output 1).
    pub fn set_ceilings(&mut self, enabled: bool) {
        if enabled != self.ceilings {
            self.ceilings = enabled;
            self.sync_skin_floors();
        }
    }

    /// Apply per-cluster user CPU ceilings from Pentas (kHz; `0` = uncapped).
    pub fn set_user_caps(&mut self, caps: [i64; MAX_CLUSTERS]) {
        let lift = self.lift;
        for (cluster, &req) in self.clusters.iter_mut().zip(caps.iter()) {
            cluster.user_cap = if req > 0 { cluster.snap_freq(req) } else { 0 };
            cluster.apply(lift);
        }
    }

    /// True when any cluster's thermal cap is below its effective unrestricted maximum.
    pub fn capped(&self) -> bool {
        self.clusters.iter().any(|c| {
            let eff = c.effective_max();
            c.cap > 0 && eff > 0 && c.cap < eff
        })
    }

    /// Skin-loop output (0 floor .. 1 hardware max) while it owns the caps.
    pub fn skin_output(&self) -> Option<f64> {
        self.skin.as_ref().map(|_| self.skin_output)
    }

    /// Pure in-memory constructor for SM8735 closed-loop unit-test simulations (zero sysfs I/O).
    #[cfg(test)]
    pub fn new_simulated(
        profile_name: &str,
        hw_max: [i64; MAX_CLUSTERS],
        opps: [&[i64]; MAX_CLUSTERS],
        force_ladder: bool,
    ) -> Self {
        let mut clusters: [ClusterSs; MAX_CLUSTERS] = std::array::from_fn(|i| ClusterSs {
            fd: None,
            hw_max_freq: hw_max[i],
            current_level: -1,
            cap: hw_max[i],
            user_cap: 0,
            written: hw_max[i],
            steps: opps[i].to_vec(),
            floor: SKIN_DEFAULT.floors_khz[i],
            ceil: 0,
            levels: [SsLevel { trig_mc: 0, clr_mc: 0, freq: 0 }; MAX_SS_LEVELS],
            level_count: 0,
            name: CLUSTER_NAMES[i],
        });
        let profile = get_profile(profile_name);
        for (i, c) in clusters.iter_mut().enumerate() {
            c.levels = profile.clusters[i].levels;
            c.level_count = profile.clusters[i].count;
            c.cap = hw_max[i];
            c.written = hw_max[i];
        }
        let skin = if force_ladder {
            None
        } else {
            skin_profile_for(profile_name).map(|sp| (SkinController::new(), sp))
        };
        let mut algo = SsAlgo {
            clusters,
            active_profile_name: profile_name.to_string(),
            check_counter: 0,
            custom_profile: None,
            custom_extras: None,
            custom_seq: -1,
            config_dirty: false,
            profile_watcher: None,
            skin,
            skin_output: 1.0,
            skin_logged: 1.0,
            last_run: None,
            lift: false,
            game_target_mc: 0,
            ceilings: true,
        };
        algo.sync_skin_floors();
        algo
    }

    /// Effective per-cluster CPU ceilings last applied (`min(thermal, user)` in kHz).
    #[cfg(test)]
    pub fn applied_caps_khz(&self) -> [i64; MAX_CLUSTERS] {
        std::array::from_fn(|i| self.clusters[i].written)
    }

    /// Get the currently active profile name.
    pub fn active_profile_name(&self) -> &str {
        &self.active_profile_name
    }

    /// True when the active profile throttles NO cluster (every cluster has zero
    /// levels) — i.e. a "no limit" profile. The caller uses this to also relax
    /// the monitor (NOLIMITS theme), so GPU/boost/hotplug/brightness stay
    /// unrestricted too; otherwise a no-limit custom profile would still be
    /// throttled by the GLOBAL monitor theme during gaming.
    pub fn is_no_limit(&self) -> bool {
        self.clusters.iter().all(|c| c.level_count == 0)
    }

    /// Extended custom knobs (GPU curve / monitor overrides / SIC overrides),
    /// or None for built-in profiles and CPU-only custom profiles.
    pub fn custom_extras(&self) -> Option<&CustomExtras> {
        self.custom_extras.as_ref()
    }

    /// Consume the "config changed this cycle" flag (set on profile switch or
    /// custom reload). The caller re-syncs the monitor theme + SIC when true.
    pub fn take_config_dirty(&mut self) -> bool {
        let d = self.config_dirty;
        self.config_dirty = false;
        d
    }

    /// (Re)load the custom profile from chunked system properties.
    fn load_custom_profile(&mut self) {
        let seq = read_prop_i64(PROP_CUSTOM_SEQ).unwrap_or(0);
        // Per-cluster hardware ceilings, so parse validation can clamp freqs.
        let hw_max = [
            self.clusters[0].hw_max_freq,
            self.clusters[1].hw_max_freq,
            self.clusters[2].hw_max_freq,
            self.clusters[3].hw_max_freq,
        ];
        let parsed = read_custom_profile_props()
            .and_then(|json| parse_custom_profile(&json, &hw_max));
        match parsed {
            Some(profile) => {
                info!("SS: custom profile loaded (seq={})", seq);
                // apply_profile() clears custom_extras + sets config_dirty; parse
                // the extended side-channel props AFTERWARDS so they survive.
                self.apply_profile(&profile);
                self.custom_profile = Some(Box::new(profile));
                self.custom_seq = seq;
                let extras = parse_custom_extras();
                self.custom_extras = if extras.is_empty() { None } else { Some(extras) };
            }
            None => warn!("SS: failed to parse custom profile (seq={}), keeping current", seq),
        }
    }
}

/// Read + parse the GPU / monitor / SIC side-channel CSV props for a custom
/// profile. Missing/blank props leave the corresponding fields zero (unset).
/// Rare path (custom reload only) — heap allocation is fine here.
fn parse_custom_extras() -> CustomExtras {
    let mut e = CustomExtras::new();

    // custom.gpu: "trig,clr,freqHz;trig,clr,freqHz;…"
    if let Some(s) = read_prop_str(PROP_CUSTOM_GPU) {
        for lvl in s.split(';') {
            if e.gpu_count >= CUSTOM_GPU_MAX {
                break;
            }
            let mut it = lvl.split(',');
            let trig = it.next().and_then(|v| v.trim().parse::<i64>().ok()).unwrap_or(0);
            let clr = it.next().and_then(|v| v.trim().parse::<i64>().ok()).unwrap_or(0);
            let freq_mhz = it.next().and_then(|v| v.trim().parse::<i64>().ok()).unwrap_or(0);
            if trig > 0 && freq_mhz > 0 {
                let i = e.gpu_count;
                e.gpu_trig[i] = trig;
                e.gpu_clr[i] = if clr > 0 && clr < trig { clr } else { trig - 2000 };
                e.gpu_freq_hz[i] = freq_mhz * 1_000_000; // MHz → Hz
                e.gpu_count += 1;
            }
        }
    }

    // custom.mon: "boostTrig,hotplugTrig,blTrig,blCap" (mC; 0 = no limit)
    if let Some(s) = read_prop_str(PROP_CUSTOM_MON) {
        let mut it = s.split(',');
        e.boost_trig = it.next().and_then(|v| v.trim().parse::<i64>().ok()).unwrap_or(0).max(0);
        e.hotplug_trig = it.next().and_then(|v| v.trim().parse::<i64>().ok()).unwrap_or(0).max(0);
        e.bl_trig = it.next().and_then(|v| v.trim().parse::<i64>().ok()).unwrap_or(0).max(0);
        e.bl_cap = it.next().and_then(|v| v.trim().parse::<i64>().ok()).unwrap_or(0).max(0);
    }

    // custom.sic: "targetMc,maxFccUa" (0 = use global config)
    if let Some(s) = read_prop_str(PROP_CUSTOM_SIC) {
        let mut it = s.split(',');
        e.sic_target_mc = it.next().and_then(|v| v.trim().parse::<i64>().ok()).unwrap_or(0).max(0);
        e.sic_max_fcc_ua = it.next().and_then(|v| v.trim().parse::<i64>().ok()).unwrap_or(0).max(0);
    }

    e
}

/// Read a system property as a trimmed non-empty String (rare path).
fn read_prop_str(prop: &str) -> Option<String> {
    match rustutils::android::system_properties::read(prop) {
        Ok(Some(val)) if !val.trim().is_empty() => Some(val),
        _ => None,
    }
}

/// Asymmetric level selector for predictive SS.
/// Levels at or below the current level are evaluated against `clear_temp`
/// (the measured temperature) using their clear thresholds — conservative
/// release. Levels above the current level are evaluated against `trigger_temp`
/// (max of measured and predicted) using their trigger thresholds — aggressive
/// engage. The scan returns the highest satisfied level, i.e. the more
/// aggressive of the two evaluations. With `trigger_temp == clear_temp` this is
/// identical to the original reactive `compute_ss_level`.
fn compute_ss_level_predictive(
    levels: &[SsLevel],
    current_level: i32,
    trigger_temp: i64,
    clear_temp: i64,
) -> i32 {
    let mut new_level: i32 = -1;
    for (i, level) in levels.iter().enumerate() {
        let i32_i = i as i32;
        if i32_i <= current_level {
            if clear_temp >= level.clr_mc {
                new_level = i32_i;
            }
        } else if trigger_temp >= level.trig_mc {
            new_level = i32_i;
        }
    }
    new_level
}

/// Profile named by a property value: empty means the default.
fn effective_profile(value: &str) -> &str {
    if value.is_empty() { PROFILE_NAME_DEFAULT } else { value }
}

/// Custom profile IPC via chunked system properties (written by GameSpace).
/// Property values are capped at 92 bytes, so the JSON payload is split:
///   persist.sys.mithermal.custom.seq — bumped on every write (change detector)
///   persist.sys.mithermal.custom.n   — number of chunks
///   persist.sys.mithermal.custom.<i> — payload chunk i (0-based)
const PROP_CUSTOM_SEQ: &str = "persist.sys.mithermal.custom.seq";
const PROP_CUSTOM_COUNT: &str = "persist.sys.mithermal.custom.n";
const PROP_CUSTOM_CHUNK_PREFIX: &str = "persist.sys.mithermal.custom.";

/// Read a system property as i64 (rare path — profile switches only).
fn read_prop_i64(prop: &str) -> Option<i64> {
    match rustutils::android::system_properties::read(prop) {
        Ok(Some(val)) => val.trim().parse::<i64>().ok(),
        _ => None,
    }
}

/// Reassemble the custom-profile JSON from chunked properties.
fn read_custom_profile_props() -> Option<String> {
    let count = read_prop_i64(PROP_CUSTOM_COUNT).unwrap_or(0);
    if count <= 0 || count > 64 {
        warn!("SS: invalid custom profile chunk count {}", count);
        return None;
    }
    let mut json = String::with_capacity(count as usize * 92);
    for i in 0..count {
        let name = format!("{}{}", PROP_CUSTOM_CHUNK_PREFIX, i);
        match rustutils::android::system_properties::read(&name) {
            Ok(Some(chunk)) if !chunk.is_empty() => json.push_str(&chunk),
            _ => {
                warn!("SS: missing custom profile chunk {}/{}", i, count);
                return None;
            }
        }
    }
    Some(json)
}

/// Parse a custom profile from a JSON string, validating every level.
/// JSON format: [{"levels":[{"trig":40000,"clr":38000,"freq":1785600}]}, ...]
/// (array of 4 cluster objects). `hw_max_freqs` are the per-cluster hardware
/// ceilings (kHz) used to clamp `freq`; pass 0 for a cluster to skip clamping.
fn parse_custom_profile(
    json_str: &str,
    hw_max_freqs: &[i64; MAX_CLUSTERS],
) -> Option<ThermalProfile> {
    // Minimal JSON parser for our specific format
    // We parse: [{"levels":[{"trig":N,"clr":N,"freq":N},...]}, ...]
    let mut profile = ThermalProfile {
        name: "Custom",
        clusters: [
            ClusterLevels::empty(),
            ClusterLevels::empty(),
            ClusterLevels::empty(),
            ClusterLevels::empty(),
        ],
    };

    // Simple state machine JSON parser for our known format
    let bytes = json_str.as_bytes();
    let len = bytes.len();
    let mut pos = 0;
    let mut cluster_idx = 0;

    // Skip to first '['
    while pos < len && bytes[pos] != b'[' { pos += 1; }
    pos += 1; // skip '['

    while pos < len && cluster_idx < MAX_CLUSTERS {
        // Find start of cluster object '{'
        while pos < len && bytes[pos] != b'{' { pos += 1; }
        if pos >= len { break; }
        pos += 1; // skip '{'

        // Find "levels" array
        while pos < len {
            if pos + 6 < len && &json_str[pos..pos+6] == "levels" {
                // Skip to the '[' after "levels":
                while pos < len && bytes[pos] != b'[' { pos += 1; }
                pos += 1; // skip '['
                break;
            }
            pos += 1;
        }

        // Parse levels
        let mut level_idx = 0;
        while pos < len && level_idx < MAX_SS_LEVELS {
            // Find start of level object
            while pos < len && bytes[pos] != b'{' && bytes[pos] != b']' { pos += 1; }
            if pos >= len || bytes[pos] == b']' { pos += 1; break; }
            pos += 1; // skip '{'

            // Extract trig, clr, freq from this object
            let mut trig: i64 = 0;
            let mut clr: i64 = 0;
            let mut freq: i64 = 0;

            while pos < len && bytes[pos] != b'}' {
                // Find key
                if pos + 4 < len && &json_str[pos..pos+4] == "trig" {
                    pos += 4;
                    trig = parse_json_number(json_str, &mut pos);
                } else if pos + 3 < len && &json_str[pos..pos+3] == "clr" {
                    pos += 3;
                    clr = parse_json_number(json_str, &mut pos);
                } else if pos + 4 < len && &json_str[pos..pos+4] == "freq" {
                    pos += 4;
                    freq = parse_json_number(json_str, &mut pos);
                } else {
                    pos += 1;
                }
            }
            if pos < len { pos += 1; } // skip '}'

            // ---- Per-level validation (Phase 1.3) ----
            if trig <= 0 || freq <= 0 {
                // freq=0 would freeze the CPU; trig<=0 is a garbage threshold.
                warn!("SS custom: cluster {} level {} invalid (trig={} freq={}), skipped",
                      cluster_idx, level_idx, trig, freq);
            } else {
                // Clamp freq to the cluster's hardware ceiling.
                let hw_max = hw_max_freqs[cluster_idx];
                if hw_max > 0 && freq > hw_max {
                    warn!("SS custom: cluster {} level {} freq {}kHz > hw_max {}kHz, clamped",
                          cluster_idx, level_idx, freq, hw_max);
                    freq = hw_max;
                }
                // Inverted/absent hysteresis oscillates every cycle — repair it.
                if clr >= trig {
                    let fixed = trig - 1000; // default 1°C hysteresis
                    warn!("SS custom: cluster {} level {} clr {} >= trig {}, set clr={}",
                          cluster_idx, level_idx, clr, trig, fixed);
                    clr = fixed;
                }
                if clr <= 0 {
                    clr = trig - 2000; // default 2°C hysteresis
                    warn!("SS custom: cluster {} level {} non-positive clr, set clr={}",
                          cluster_idx, level_idx, clr);
                }
                profile.clusters[cluster_idx].levels[level_idx] = SsLevel {
                    trig_mc: trig, clr_mc: clr, freq,
                };
                level_idx += 1;
            }
        }
        profile.clusters[cluster_idx].count = level_idx;

        // Skip to end of cluster object
        let mut depth = 0;
        while pos < len {
            if bytes[pos] == b'{' { depth += 1; }
            if bytes[pos] == b'}' {
                if depth == 0 { pos += 1; break; }
                depth -= 1;
            }
            pos += 1;
        }

        cluster_idx += 1;
    }

    // A structurally broken/torn read yields zero cluster OBJECTS — only then do
    // we keep the current profile. An all-EMPTY profile (clusters parsed, zero
    // levels) is a VALID "no limit" config: every cluster runs at hw_max and is
    // never throttled. It MUST be applied, not rejected — rejecting it was the
    // bug that pinned no-limit customs to the previously-active throttle profile.
    if cluster_idx == 0 {
        warn!("SS custom: no cluster objects parsed from JSON (torn/garbage read?)");
        return None;
    }
    let total_levels: usize = profile.clusters.iter().map(|c| c.count).sum();
    if total_levels == 0 {
        info!("SS: custom profile is no-limit — all {} clusters unthrottled", cluster_idx);
    }

    info!("SS: parsed custom profile with {} clusters ({}/{}/{}/{} levels)",
          cluster_idx,
          profile.clusters[0].count,
          profile.clusters[1].count,
          profile.clusters[2].count,
          profile.clusters[3].count);

    Some(profile)
}

/// Extract a number after ':' in JSON (e.g. "trig":40000)
fn parse_json_number(s: &str, pos: &mut usize) -> i64 {
    let bytes = s.as_bytes();
    let len = bytes.len();

    // Skip to ':'
    while *pos < len && bytes[*pos] != b':' { *pos += 1; }
    *pos += 1; // skip ':'

    // Skip whitespace
    while *pos < len && bytes[*pos] == b' ' { *pos += 1; }

    // Parse number
    let mut num: i64 = 0;
    let mut negative = false;
    if *pos < len && bytes[*pos] == b'-' {
        negative = true;
        *pos += 1;
    }
    while *pos < len && bytes[*pos] >= b'0' && bytes[*pos] <= b'9' {
        num = num * 10 + (bytes[*pos] - b'0') as i64;
        *pos += 1;
    }
    if negative { -num } else { num }
}
