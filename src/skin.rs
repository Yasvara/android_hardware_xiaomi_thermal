// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Skin-temperature controller for the default profile.
//!
//! A step ladder caps the clocks by temperature alone, so a phone still warm
//! from a game stays slow while nothing is loading it. This PI loop holds
//! VSENSOR0 at a target instead: below it the caps sit at hardware max, above
//! it they come down only as far as the heat being produced requires.
//!
//! The output is one number, 0 (floor caps) to 1 (hardware max), that the
//! caller maps onto every CPU cluster and the GPU.

/// VSENSOR0 the loop holds under sustained load (mC).
pub const TARGET_MC: i64 = 42_000;

/// Output per °C of error.
const KP: f64 = 0.35;
/// Output per °C·s of accumulated error.
const KI: f64 = 0.006;
/// Derivative damping (output per °C/s of rising skin temperature) applied in
/// the approach band near/above target. Counteracts vapor-chamber thermal
/// inertia so heavy 10 W+ gaming bursts brake smoothly before overshooting.
const KD: f64 = 7.5;
/// Width (°C) below target over which rising-temperature damping fades in (0..1).
const D_BAND_C: f64 = 3.0;
/// Fastest the output may rise, per second. It falls as fast as the law asks,
/// so a load spike is met at once while release waits for the skin to follow.
const MAX_RISE: f64 = 0.02;
/// Time constant (s) of the smoothing applied to VSENSOR0 before the law.
const SMOOTH_S: f64 = 4.0;
/// Integral floor: deep enough to hold the output at 0 against a few °C of
/// headroom, shallow enough to recover within minutes once the load is gone.
const INTEGRAL_MIN: f64 = -0.5;

pub struct SkinController {
    integral: f64,
    output: f64,
    smoothed_c: Option<f64>,
    rate_c_per_s: f64,
}

impl SkinController {
    pub fn new() -> Self {
        SkinController { integral: 1.0, output: 1.0, smoothed_c: None, rate_c_per_s: 0.0 }
    }

    /// Advances the loop by `dt_s` seconds and returns the new output.
    pub fn update(&mut self, temp_mc: i64, target_mc: i64, dt_s: f64) -> f64 {
        let temp_c = temp_mc as f64 / 1000.0;
        let (smoothed, inst_rate) = match self.smoothed_c {
            Some(prev) => {
                let s = prev + (temp_c - prev) * dt_s / (SMOOTH_S + dt_s);
                let r = if dt_s > 0.01 { (s - prev) / dt_s } else { 0.0 };
                (s, r)
            }
            None => (temp_c, 0.0),
        };
        self.smoothed_c = Some(smoothed);
        // Smooth derivative estimate (tau = 6s) so ±150 mC ADC jitter never chatters.
        self.rate_c_per_s += (inst_rate - self.rate_c_per_s) * dt_s / (6.0 + dt_s);

        let target_c = target_mc as f64 / 1000.0;
        let error = target_c - smoothed;

        // Rising-rate damping fades in linearly from (target - D_BAND_C) to target,
        // and only brakes warming (never fights recovery while cooling).
        let approach = ((smoothed - (target_c - D_BAND_C)) / D_BAND_C).clamp(0.0, 1.0);
        let d_term = -KD * approach * self.rate_c_per_s.max(0.0);

        let law = KP * error + self.integral + d_term;
        // Integrate only while the output can still move that way; past a rail
        // the integral would wind up and hold the caps wrong long after.
        let saturated = (law >= 1.0 && error > 0.0) || (law <= 0.0 && error < 0.0);
        if !saturated {
            self.integral = (self.integral + KI * error * dt_s).clamp(INTEGRAL_MIN, 1.0);
        }

        let wanted = (KP * error + self.integral + d_term).clamp(0.0, 1.0);
        self.output = wanted.min(self.output + MAX_RISE * dt_s);
        self.output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Phone thermals reduced to two lags: the board heats with the SoC power,
    /// the skin follows the board. Power grows with the square of the clocks.
    struct Plant {
        board: f64,
        skin: f64,
        load_w: f64,
    }

    impl Plant {
        const AMBIENT: f64 = 30.0;
        const C_PER_W: f64 = 2.5;

        fn step(&mut self, output: f64) {
            let clocks = 0.3 + 0.7 * output;
            let power = 1.0 + self.load_w * clocks * clocks;
            self.board += (Self::AMBIENT + Self::C_PER_W * power - self.board) / 30.0;
            self.skin += (self.board - self.skin) / 120.0;
        }
    }

    fn run(load_w: f64, start_c: f64, secs: usize) -> (Vec<f64>, Vec<f64>) {
        let mut ctl = SkinController::new();
        let mut plant = Plant { board: start_c, skin: start_c, load_w };
        let (mut skin, mut out) = (Vec::new(), Vec::new());
        for _ in 0..secs {
            let o = ctl.update((plant.skin * 1000.0) as i64, TARGET_MC, 1.0);
            plant.step(o);
            skin.push(plant.skin);
            out.push(o);
        }
        (skin, out)
    }

    #[test]
    fn heavy_load_settles_at_target() {
        let (skin, out) = run(7.0, 35.0, 1500);
        let peak = skin.iter().cloned().fold(f64::MIN, f64::max);
        assert!(peak < 44.5, "overshoot to {peak}");
        assert!((skin[1499] - 42.0).abs() < 0.3, "settled at {}", skin[1499]);
        let tail = &out[1200..];
        let (lo, hi) = tail.iter().fold((1.0f64, 0.0f64), |(l, h), &o| (l.min(o), h.max(o)));
        assert!(hi - lo < 0.03, "output still moving {lo}..{hi}");
    }

    #[test]
    fn light_load_never_caps() {
        let (_, out) = run(1.0, 35.0, 900);
        assert!(out.iter().all(|&o| o == 1.0));
    }

    #[test]
    fn hot_start_caps_then_recovers_when_idle() {
        let (skin, out) = run(1.0, 47.0, 900);
        assert!(out[10] < 0.5, "hot phone left uncapped: {}", out[10]);
        let below = skin.iter().position(|&s| s < 41.0).unwrap();
        assert!(out[below + 120] == 1.0, "still capped 2 min after cooling");
    }

    #[test]
    fn output_rises_no_faster_than_limit() {
        let (_, out) = run(1.0, 47.0, 600);
        for w in out.windows(2) {
            assert!(w[1] - w[0] <= MAX_RISE + 1e-9);
        }
    }
}
