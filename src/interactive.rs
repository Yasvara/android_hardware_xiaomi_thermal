// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Interactive headroom: thermal caps step aside while the user is touching.
//!
//! Skin temperature moves over minutes, so a second or two at full clocks adds
//! almost no heat, while a frame rendered under a cap is visible jank. The power
//! HAL holds `vendor.powerhal.interaction` at "1" for ~1.4 s after each touch and
//! through app launches; while it does, the caps go to hardware max. A budget
//! bounds the lifted share, so a long continuous interaction still settles back
//! under the thermal caps.

use std::time::{Duration, Instant};

use log::info;
use rustutils::android::system_properties::PropertyWatcher;

const PROP_INTERACTION: &str = "vendor.powerhal.interaction";

/// Longest continuous lift (s).
const BUDGET_S: f64 = 30.0;
/// Budget regained per second without a lift: sustained, at most a fifth of
/// the time is lifted.
const REFILL_PER_S: f64 = 0.25;
/// Budget an exhausted lift must regain before lifting again, so it does not
/// flicker on each sliver of refill.
const RESUME_S: f64 = 5.0;

pub struct Interactive {
    watcher: PropertyWatcher,
    hint: bool,
    budget_s: f64,
    exhausted: bool,
    lifted: bool,
    last: Instant,
}

impl Interactive {
    pub fn new() -> Self {
        Interactive {
            // A name without interior NULs cannot fail.
            watcher: PropertyWatcher::new(PROP_INTERACTION).expect("property name"),
            hint: false,
            budget_s: BUDGET_S,
            exhausted: false,
            lifted: false,
            last: Instant::now(),
        }
    }

    /// Pure sleep until `deadline` without waking on touch property transitions
    /// (used while screen is off or when no thermal cap is active).
    pub fn sleep_until(&self, deadline: Instant) {
        let now = Instant::now();
        if deadline > now {
            std::thread::sleep(deadline - now);
        }
    }

    /// Sleeps until the hint changes or `deadline`, then re-reads it.
    pub fn wait(&mut self, deadline: Instant) {
        let now = Instant::now();
        if deadline > now {
            let timeout = deadline - now;
            if self.watcher.wait(Some(timeout)).is_err() {
                // A timeout is an error too; only a wait that failed outright
                // returns early, and must not turn the loop into a spin.
                let left = deadline.saturating_duration_since(Instant::now());
                if left > timeout / 2 {
                    std::thread::sleep(left);
                }
            }
        }
        self.hint = self.watcher.read(|_, value| value == "1").unwrap_or(false);
    }

    /// Whether the caps should be lifted now. `allowed` carries everything
    /// outside the hint and the budget: profile, screen, temperature, and
    /// whether any cap is below hardware max at all, so an uncapped phone
    /// does not spend the budget.
    pub fn update(&mut self, allowed: bool, now: Instant) -> bool {
        let dt = now.saturating_duration_since(self.last).as_secs_f64();
        self.last = now;
        self.budget_s = if self.lifted {
            (self.budget_s - dt).max(0.0)
        } else {
            (self.budget_s + dt * REFILL_PER_S).min(BUDGET_S)
        };

        if self.budget_s <= 0.0 && !self.exhausted {
            self.exhausted = true;
            info!("Interactive: budget spent, caps hold until {RESUME_S:.0}s refill");
        } else if self.exhausted && self.budget_s >= RESUME_S {
            self.exhausted = false;
            info!("Interactive: budget back");
        }

        self.lifted = self.hint && allowed && !self.exhausted;
        self.lifted
    }

    /// When the current lift runs out of budget, so the caller wakes for it.
    pub fn exhausted_at(&self) -> Option<Instant> {
        self.lifted.then(|| self.last + Duration::from_secs_f64(self.budget_s))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn with_hint(hint: bool) -> Interactive {
        let mut i = Interactive::new();
        i.hint = hint;
        i
    }

    #[test]
    fn lifts_only_with_hint_and_permission() {
        let t0 = Instant::now();
        let mut i = with_hint(false);
        assert!(!i.update(true, t0));
        i.hint = true;
        assert!(!i.update(false, t0));
        assert!(i.update(true, t0));
    }

    #[test]
    fn continuous_lift_ends_with_budget() {
        let t0 = Instant::now();
        let mut i = with_hint(true);
        assert!(i.update(true, t0));
        assert!(i.update(true, t0 + Duration::from_secs(29)));
        assert!(!i.update(true, t0 + Duration::from_secs(31)));
    }

    #[test]
    fn spent_budget_waits_for_resume_then_refills_at_a_quarter() {
        let t0 = Instant::now();
        let mut i = with_hint(true);
        i.update(true, t0);
        assert!(!i.update(true, t0 + Duration::from_secs(30)));
        // 4 s idle refills 1 s: below the resume mark, still held.
        assert!(!i.update(true, t0 + Duration::from_secs(34)));
        i.hint = false;
        i.update(true, t0 + Duration::from_secs(34));
        // 16 s more at a quarter reaches 5 s.
        i.hint = true;
        let t1 = t0 + Duration::from_secs(50);
        assert!(i.update(true, t1));
        assert!(i.update(true, t1 + Duration::from_millis(4900)));
        assert!(!i.update(true, t1 + Duration::from_millis(5100)));
    }
}
