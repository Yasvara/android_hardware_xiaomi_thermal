/* --- thermal/src/policy.rs --- */
// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Charging *policy* transport: profile JSON -> mca kernel nodes.
//!
//! Everything the mca driver used to hardcode at probe time - the JEITA band
//! table, the per-adapter input/charge ceilings, the thermal derate ladder, the
//! bypass regulator's tunables - is now runtime-writable sysfs. This module is
//! what actually drives those nodes from the active GameSpace profile.
//!
//! Split out of `sic.rs` deliberately. `sic.rs` runs a PID every second and
//! owns the *dynamic* FCC vote; this runs only when the profile changes and
//! owns the *static* limits the PID operates inside. Mixing them would put a
//! dozen rarely-touched string writes in the hot path.
//!
//! # Why reject instead of clamp
//!
//! The kernel clamps out-of-envelope values (mca_policy.h) because a driver
//! cannot refuse to have a float voltage. The daemon is the layer that can, so
//! it does: a profile carrying an out-of-envelope number is refused wholesale
//! for that section and logged with the offending field. Silently charging at
//! 4550 mV when the profile asked for 4700 would look like it worked, and the
//! next person to read the profile would believe 4700 was in effect.

use log::{error, info, warn};
use rustutils::android::system_properties::PropertyWatcher;

use crate::config;
use crate::io::SysfsFd;
use crate::sic::{
    extract_field_i64, read_charging_chunks, PROP_CHARGING_MODE, PROP_CHARGING_SEQ,
};

const PATH_JEITA_TABLE: &str = "/sys/class/xm_power/charger/jeita/jeita_table";
const PATH_JEITA_PARAMS: &str = "/sys/class/xm_power/charger/jeita/jeita_params";
const PATH_POLICY_ENVELOPE: &str = "/sys/class/xm_power/charger/jeita/policy_envelope";
const PATH_CHG_CAPS: &str = "/sys/class/xm_power/charger/buckchg/chg_caps";
const PATH_THERMAL_TABLE: &str =
    "/sys/class/xm_power/charger/charger_thermal/wired_thermal_table";
const PATH_BYPASS_PARAMS: &str = "/sys/class/xm_power/charger/smart_charge/bypass_params";
const PATH_BYPASS_MODE: &str = "/sys/class/xm_power/charger/smart_charge/bypass_charge";

/// Columns in a JEITA row: t_lo, t_hi, hys_lo, hys_hi, fcc_mA, fv_mV, iterm_mA.
const JEITA_COLS: usize = 7;
/// Rows the kernel table can hold (JEITA_DATA_MAX_GROUP).
const JEITA_MAX_ROWS: usize = 15;
/// A thermal ladder row is a level index plus the 10 per-path currents.
const LADDER_COLS: usize = 11;
const LADDER_MAX_ROWS: usize = 15;

/// Re-verify the kernel still holds our override this often (poll cycles).
/// Cheap - one 16-byte read - and it catches a charger driver re-probe, which
/// silently drops every override back to DT.
const VERIFY_INTERVAL: u32 = 30;

/// Which sections the currently-applied profile actually carried. A profile
/// that drops a section must put that section back to DT, otherwise the last
/// profile's JEITA table would outlive it.
#[derive(Default, Clone, Copy, PartialEq)]
struct AppliedSections {
    envelope: bool,
    jeita: bool,
    caps: bool,
    ladder: bool,
}

pub struct PolicyPusher {
    seq: i64,
    applied: AppliedSections,
    active: bool,
    verify_counter: u32,
    policy_envelope: Option<SysfsFd>,
    jeita_table: Option<SysfsFd>,
    jeita_params: Option<SysfsFd>,
    chg_caps: Option<SysfsFd>,
    thermal_table: Option<SysfsFd>,
    bypass_params: Option<SysfsFd>,
    bypass_mode: Option<SysfsFd>,
    mode_watcher: PropertyWatcher,
    seq_watcher: PropertyWatcher,
}

impl PolicyPusher {
    pub fn new() -> Self {
        let p = PolicyPusher {
            seq: -1,
            applied: AppliedSections::default(),
            active: false,
            verify_counter: 0,
            policy_envelope: SysfsFd::open_rw(PATH_POLICY_ENVELOPE),
            jeita_table: SysfsFd::open_rw(PATH_JEITA_TABLE),
            jeita_params: SysfsFd::open_rw(PATH_JEITA_PARAMS),
            chg_caps: SysfsFd::open_rw(PATH_CHG_CAPS),
            thermal_table: SysfsFd::open_rw(PATH_THERMAL_TABLE),
            bypass_params: SysfsFd::open_rw(PATH_BYPASS_PARAMS),
            bypass_mode: SysfsFd::open_rw(PATH_BYPASS_MODE),
            mode_watcher: PropertyWatcher::new(PROP_CHARGING_MODE).expect("prop name"),
            seq_watcher: PropertyWatcher::new(PROP_CHARGING_SEQ).expect("prop name"),
        };
        if p.jeita_table.is_none() {
            // Two very different causes, and the difference matters: ENOENT is
            // an older kernel without the override nodes, EACCES is the init
            // chown in mi_thermal_engine.rc not having run (or missing). Both
            // used to print "kernel predates runtime JEITA", which sent the
            // last debugging session looking at the wrong layer entirely.
            warn!("policy: jeita_table unavailable ({}) - charging policy stays at DT",
                match std::fs::metadata(PATH_JEITA_TABLE) {
                    Ok(_) => "present but not writable; check the .rc chown and sepolicy",
                    Err(_) => "node missing; kernel predates runtime JEITA",
                });
        }
        p
    }

    /// Called once per poll cycle from the coordinator. Does nothing unless the
    /// profile sequence moved or a re-verify is due.
    pub fn maybe_reload(&mut self) {
        let custom = self
            .mode_watcher
            .read(|_, v| v.trim() == "custom")
            .unwrap_or(false);

        if !custom {
            if self.active {
                info!("policy: profile -> GLOBAL, restoring DT charging policy");
                self.restore_all();
            }
            return;
        }

        let seq = self
            .seq_watcher
            .read(|_, v| v.trim().parse::<i64>().unwrap_or(-1))
            .unwrap_or(-1);
        if seq == self.seq && self.active {
            self.maybe_verify();
            return;
        }

        let json = match read_charging_chunks() {
            Some(j) => j,
            None => {
                warn!("policy: profile chunks unreadable (seq={}), keeping current", seq);
                return;
            }
        };

        self.apply(&json, seq);
    }

    /// A charger driver re-probe drops every override back to DT without
    /// telling anyone. The seq has not moved, so nothing would re-push it.
    fn maybe_verify(&mut self) {
        self.verify_counter += 1;
        if self.verify_counter < VERIFY_INTERVAL {
            return;
        }
        self.verify_counter = 0;

        if !self.applied.jeita {
            return;
        }
        let Some(ref fd) = self.jeita_table else { return };
        let mut buf = [0u8; 16];
        // The show handler leads with "src=dt" or "src=override".
        if let Some(s) = fd.read_str(&mut buf) {
            if s.starts_with("src=dt") {
                warn!("policy: kernel dropped our JEITA override (driver re-probe?), re-pushing");
                self.seq = -1;
                self.active = false;
            }
        }
    }

    fn apply(&mut self, json: &str, seq: i64) {
        let mut now = AppliedSections::default();

        // Envelope FIRST: it widens the kernel clamp the sections below run
        // inside. Pushing tables before the envelope would clamp an OC table
        // (e.g. 22000 mA) back down to the old 20000 ceiling.
        now.envelope = self.apply_envelope(json);
        now.jeita = self.apply_jeita(json);
        self.apply_jeita_params(json);
        now.caps = self.apply_caps(json);
        now.ladder = self.apply_ladder(json);
        self.apply_bypass(json);

        // Sections the previous profile owned and this one does not.
        if self.applied.jeita && !now.jeita {
            self.write(&self.jeita_table, "reset", "jeita_table");
        }
        if self.applied.caps && !now.caps {
            self.write(&self.chg_caps, "reset", "chg_caps");
        }
        if self.applied.ladder && !now.ladder {
            self.write(&self.thermal_table, "reset", "wired_thermal_table");
        }
        if self.applied.envelope && !now.envelope {
            self.write(&self.policy_envelope, "reset", "policy_envelope");
        }

        self.applied = now;
        self.seq = seq;
        self.active = true;
        self.verify_counter = 0;
        info!(
            "policy: profile seq={} applied (envelope={} jeita={} caps={} ladder={})",
            seq, now.envelope, now.jeita, now.caps, now.ladder
        );
    }

    fn restore_all(&mut self) {
        if self.applied.jeita {
            self.write(&self.jeita_table, "reset", "jeita_table");
        }
        if self.applied.caps {
            self.write(&self.chg_caps, "reset", "chg_caps");
        }
        if self.applied.ladder {
            self.write(&self.thermal_table, "reset", "wired_thermal_table");
        }
        if self.applied.envelope {
            self.write(&self.policy_envelope, "reset", "policy_envelope");
        }
        self.applied = AppliedSections::default();
        self.active = false;
        self.seq = -1;
    }

    // ---- sections ----

    /// Requested envelope, e.g. {"envelope":{"fcc_max":22000,"iin_max":6000}}.
    /// Pushed before every other section. Validated against the ABS silicon
    /// ceiling; the kernel re-validates against the same ABS bounds and logs
    /// every widening in dmesg.
    fn apply_envelope(&self, json: &str) -> bool {
        let Some(section) = find_section(json, "envelope") else { return false };
        let mut pushed = false;
        // (key, abs_min, abs_max)
        for (key, lo, hi) in [
            ("fcc_max", 0, config::HW_FCC_MAX_MA),
            ("iin_max", 0, config::HW_IIN_MAX_MA),
            ("fv_max", config::HW_FV_MIN_MV, config::HW_FV_MAX_MV),
            ("temp_min", config::HW_TEMP_MIN_C, config::HW_TEMP_MAX_C),
            ("temp_max", config::HW_TEMP_MIN_C, config::HW_TEMP_MAX_C),
            ("iterm_min", 0, config::HW_ITERM_MAX_MA),
            ("iterm_max", 0, config::HW_ITERM_MAX_MA),
        ] {
            let Some(v) = extract_field_i64(section, key) else { continue };
            if v < lo || v > hi {
                error!("policy: REJECTED envelope.{}={} (ABS {}..{})", key, v, lo, hi);
                continue;
            }
            self.write(&self.policy_envelope, &format!("{} {}", key, v), "policy_envelope");
            pushed = true;
        }
        if pushed {
            info!("policy: envelope widened per profile");
        }
        pushed
    }

    fn apply_jeita(&self, json: &str) -> bool {
        let Some(section) = find_section(json, "jeita") else { return false };
        let mut pushed = false;

        for set in ["normal", "ffc"] {
            let Some(body) = find_section(section, set) else { continue };
            let rows = parse_rows(body, JEITA_COLS, JEITA_MAX_ROWS);
            if rows.is_empty() {
                continue;
            }
            match validate_jeita(&rows) {
                Ok(()) => {}
                Err(why) => {
                    error!("policy: REJECTED jeita.{} - {}", set, why);
                    continue;
                }
            }
            let mut cmd = String::with_capacity(16 + rows.len() * JEITA_COLS * 7);
            cmd.push_str(set);
            cmd.push(' ');
            cmd.push_str(&rows.len().to_string());
            for row in &rows {
                for v in row {
                    cmd.push(' ');
                    cmd.push_str(&v.to_string());
                }
            }
            self.write(&self.jeita_table, &cmd, "jeita_table");
            info!("policy: jeita.{} <- {} bands", set, rows.len());
            pushed = true;
        }
        pushed
    }

    fn apply_jeita_params(&self, json: &str) {
        let Some(section) = find_section(json, "jeita_params") else { return };
        // hot_fv_gate decides whether a hot band *cuts charging* or merely
        // steps the float voltage down. Below the band's own vterm it cuts;
        // that is the 48 C latch. A profile raises it to keep charging.
        for key in [
            "hot_fv_gate",
            "hot_recharge_hys",
            "hot_term_hys",
            "vbat_high_hyst",
            "vbat_low_hyst",
            "vbat_low_cold_hyst",
        ] {
            let Some(v) = extract_field_i64(section, key) else { continue };
            if key == "hot_fv_gate" && !(config::HW_FV_MIN_MV..=config::HW_FV_MAX_MV).contains(&v) {
                error!(
                    "policy: REJECTED jeita_params.hot_fv_gate={} (envelope {}..{} mV)",
                    v,
                    config::HW_FV_MIN_MV,
                    config::HW_FV_MAX_MV
                );
                continue;
            }
            self.write(&self.jeita_params, &format!("{} {}", key, v), "jeita_params");
        }
    }

    fn apply_caps(&self, json: &str) -> bool {
        let Some(section) = find_section(json, "caps") else { return false };
        let mut pushed = false;

        for key in CAP_KEYS {
            let Some(v) = extract_field_i64(section, key) else { continue };
            let limit = if key.starts_with("in_") {
                config::HW_IIN_MAX_MA
            } else {
                config::HW_FCC_MAX_MA
            };
            if v < 0 || v > limit {
                error!("policy: REJECTED caps.{}={} mA (envelope 0..{})", key, v, limit);
                continue;
            }
            self.write(&self.chg_caps, &format!("{} {}", key, v), "chg_caps");
            pushed = true;
        }

        for key in ["ffc_temp_low", "ffc_temp_high"] {
            let Some(v) = extract_field_i64(section, key) else { continue };
            if !(config::HW_TEMP_MIN_C..=config::HW_TEMP_MAX_C).contains(&v) {
                error!("policy: REJECTED caps.{}={}°C (envelope {}..{})",
                    key, v, config::HW_TEMP_MIN_C, config::HW_TEMP_MAX_C);
                continue;
            }
            self.write(&self.chg_caps, &format!("{} {}", key, v), "chg_caps");
            pushed = true;
        }

        if pushed {
            info!("policy: per-adapter ceilings updated");
        }
        pushed
    }

    fn apply_ladder(&self, json: &str) -> bool {
        let Some(body) = find_section(json, "ladder") else { return false };
        let rows = parse_rows(body, LADDER_COLS, LADDER_MAX_ROWS);
        if rows.is_empty() {
            return false;
        }
        let mut pushed = false;
        for row in &rows {
            let level = row[0];
            if !(1..=LADDER_MAX_ROWS as i64).contains(&level) {
                error!("policy: REJECTED ladder level {}", level);
                continue;
            }
            // Columns 1..2 are the two VBUS input caps, 3..10 land on the pack.
            if let Some(bad) = row[1..].iter().enumerate().find(|(i, &v)| {
                let lim = if *i < 2 { config::HW_IIN_MAX_MA } else { config::HW_FCC_MAX_MA };
                v < 0 || v > lim
            }) {
                error!("policy: REJECTED ladder level {} - column {} = {} mA out of envelope",
                    level, bad.0 + 1, bad.1);
                continue;
            }
            let mut cmd = String::with_capacity(LADDER_COLS * 7);
            for (i, v) in row.iter().enumerate() {
                if i > 0 {
                    cmd.push(' ');
                }
                cmd.push_str(&v.to_string());
            }
            self.write(&self.thermal_table, &cmd, "wired_thermal_table");
            pushed = true;
        }
        if pushed {
            info!("policy: thermal ladder <- {} levels", rows.len());
        }
        pushed
    }

    fn apply_bypass(&self, json: &str) {
        let Some(section) = find_section(json, "bypass") else { return };

        // ZDFB v2 regulator tunables. The kernel sanitises these itself
        // (bypass_tune_sanitise); the ranges here only keep a typo from
        // silently becoming a 3 A step size.
        for (key, lo, hi) in BYPASS_KEYS {
            let Some(v) = extract_field_i64(section, key) else { continue };
            if v < *lo || v > *hi {
                error!("policy: REJECTED bypass.{}={} (range {}..{})", key, v, lo, hi);
                continue;
            }
            self.write(&self.bypass_params, &format!("{} {}", key, v), "bypass_params");
        }

        // 0 = off, 1 = force, 2 = auto (game scenes only).
        if let Some(mode) = extract_field_i64(section, "mode") {
            if (0..=2).contains(&mode) {
                self.write(&self.bypass_mode, &mode.to_string(), "bypass_charge");
                info!("policy: bypass mode -> {}", mode);
            } else {
                error!("policy: REJECTED bypass.mode={}", mode);
            }
        }
    }

    fn write(&self, fd: &Option<SysfsFd>, value: &str, what: &str) {
        match fd {
            Some(fd) => fd.write_str(value),
            None => warn!("policy: {} unavailable, dropped {:?}", what, value),
        }
    }
}

/// Per-adapter ceilings, mirroring strategy_buckchg_cap_table() in the driver.
const CAP_KEYS: &[&str] = &[
    "in_dcp", "in_pd", "in_pps", "in_hvdcp", "in_hvdcp3", "in_hvdcp3p5",
    "in_cdp", "in_sdp", "in_float",
    "chg_dcp", "chg_pd", "chg_pps", "chg_hvdcp", "chg_hvdcp3", "chg_hvdcp3p5",
    "chg_cdp", "chg_sdp", "chg_float", "chg_batt_auth_failed",
];

/// (key, min, max) for the bypass regulator. `target` is signed because a
/// profile may hold the pack slightly charge-positive rather than at true zero.
const BYPASS_KEYS: &[(&str, i64, i64)] = &[
    ("target", -500, 3000),
    ("fcc_max", 0, config::HW_FCC_MAX_MA),
    ("kp", 0, 1000),
    ("deadband", 0, 500),
    ("step", 1, 1000),
    ("in9v", 0, config::HW_IIN_MAX_MA),
    ("in5v", 0, config::HW_IIN_MAX_MA),
    ("cp_en", 0, 1),
    ("cp_mw", 6000, 25000),
    ("cp_vbat", 4200, 4480),
];

fn validate_jeita(rows: &[Vec<i64>]) -> Result<(), String> {
    for (i, r) in rows.iter().enumerate() {
        let (t_lo, t_hi, fcc, fv, iterm) = (r[0], r[1], r[4], r[5], r[6]);

        for (name, t) in [("t_lo", t_lo), ("t_hi", t_hi)] {
            if !config::is_temp_sentinel(t)
                && !(config::HW_TEMP_MIN_C..=config::HW_TEMP_MAX_C).contains(&t)
            {
                return Err(format!(
                    "band {} {}={}°C outside {}..{}",
                    i, name, t, config::HW_TEMP_MIN_C, config::HW_TEMP_MAX_C
                ));
            }
        }
        if t_hi <= t_lo {
            return Err(format!("band {} has t_hi {} <= t_lo {}", i, t_hi, t_lo));
        }
        if fcc < 0 || fcc > config::HW_FCC_MAX_MA {
            return Err(format!(
                "band {} fcc={} mA outside 0..{}",
                i, fcc, config::HW_FCC_MAX_MA
            ));
        }
        // fv == 0 is the legitimate "this band does not charge" encoding.
        if fv != 0 && !(config::HW_FV_MIN_MV..=config::HW_FV_MAX_MV).contains(&fv) {
            return Err(format!(
                "band {} fv={} mV outside {}..{} - lithium plating limit, refusing",
                i, fv, config::HW_FV_MIN_MV, config::HW_FV_MAX_MV
            ));
        }
        if iterm != 0 && !(config::HW_ITERM_MIN_MA..=config::HW_ITERM_MAX_MA).contains(&iterm) {
            return Err(format!(
                "band {} iterm={} mA outside {}..{}",
                i, iterm, config::HW_ITERM_MIN_MA, config::HW_ITERM_MAX_MA
            ));
        }
    }
    Ok(())
}

// ================================================================
// Minimal JSON shape readers
//
// Same rationale as the tier parser in sic.rs: serde would drag a derive
// dependency and an allocator-heavy DOM into a daemon whose whole point is a
// tiny steady-state footprint, for input we generate ourselves.
// ================================================================

/// Return the balanced `[...]` or `{...}` that `"key":` introduces.
fn find_section<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let pat = format!("\"{}\"", key);
    let idx = json.find(&pat)?;
    let rest = &json[idx + pat.len()..];
    let colon = rest.find(':')?;
    let body = &rest[colon + 1..];
    let start = body.find(['[', '{'])?;
    let open = body.as_bytes()[start];
    let close = if open == b'[' { b']' } else { b'}' };

    let mut depth = 0i32;
    for (i, &b) in body.as_bytes()[start..].iter().enumerate() {
        if b == open {
            depth += 1;
        } else if b == close {
            depth -= 1;
            if depth == 0 {
                return Some(&body[start..start + i + 1]);
            }
        }
    }
    None
}

/// Parse `[[a,b,...],[a,b,...]]` into fixed-width integer rows. Rows with the
/// wrong arity are dropped rather than padded - a short row means the writer
/// and this parser disagree, and guessing the missing column would be worse
/// than ignoring the row.
fn parse_rows(body: &str, cols: usize, max_rows: usize) -> Vec<Vec<i64>> {
    let mut out: Vec<Vec<i64>> = Vec::new();
    let bytes = body.as_bytes();
    let mut depth = 0i32;
    let mut start: Option<usize> = None;

    for (i, &b) in bytes.iter().enumerate() {
        match b {
            b'[' => {
                depth += 1;
                if depth == 2 {
                    start = Some(i);
                }
            }
            b']' => {
                if depth == 2 {
                    if let Some(s) = start.take() {
                        let row: Vec<i64> = body[s + 1..i]
                            .split(',')
                            .filter_map(|t| t.trim().parse::<i64>().ok())
                            .collect();
                        if row.len() == cols {
                            if out.len() < max_rows {
                                out.push(row);
                            }
                        } else {
                            warn!("policy: dropped row with {} of {} columns", row.len(), cols);
                        }
                    }
                }
                depth -= 1;
            }
            _ => {}
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"{"suspend":52000,"resume":48000,
        "tiers":[{"trig":40000,"max":20000000}],
        "jeita":{"normal":[[-65535,-9,0,2,0,4530,572],[48,58,1,0,20000,4510,572]],
                 "ffc":[[20,35,1,2,20000,4580,2667]]},
        "jeita_params":{"hot_fv_gate":4520,"hot_recharge_hys":80},
        "caps":{"in_pd":3000,"chg_pd":6000,"ffc_temp_high":58},
        "ladder":[[1,2000,2000,2000,3000,20000,20000,20000,20000,20000,20000]],
        "bypass":{"target":0,"kp":15,"mode":2}}"#;

    #[test]
    fn finds_nested_sections() {
        let jeita = find_section(SAMPLE, "jeita").unwrap();
        assert!(jeita.starts_with('{') && jeita.ends_with('}'));
        assert!(jeita.contains("ffc"));
        let normal = find_section(jeita, "normal").unwrap();
        assert!(normal.starts_with("[[") && normal.ends_with("]]"));
    }

    #[test]
    fn section_key_does_not_match_a_prefix() {
        // "jeita" must not swallow "jeita_params" or vice versa.
        let p = find_section(SAMPLE, "jeita_params").unwrap();
        assert!(p.contains("hot_fv_gate"));
        assert!(!p.contains("4530"));
    }

    #[test]
    fn parses_fixed_width_rows() {
        let normal = find_section(SAMPLE, "jeita").and_then(|s| find_section(s, "normal")).unwrap();
        let rows = parse_rows(normal, JEITA_COLS, JEITA_MAX_ROWS);
        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0][0], -65535);
        assert_eq!(rows[1], vec![48, 58, 1, 0, 20000, 4510, 572]);
    }

    #[test]
    fn drops_malformed_rows() {
        let rows = parse_rows("[[1,2,3],[1,2,3,4,5,6,7]]", JEITA_COLS, JEITA_MAX_ROWS);
        assert_eq!(rows.len(), 1);
    }

    #[test]
    fn sentinels_survive_the_envelope() {
        let rows = vec![vec![-65535, -9, 0, 2, 0, 4530, 572]];
        assert!(validate_jeita(&rows).is_ok());
    }

    #[test]
    fn full_speed_hot_band_is_allowed() {
        // The whole point: 48-58 C at 20 A / 4510 mV must pass. This is vendor
        // conservatism, not chemistry.
        let rows = vec![vec![48, 58, 1, 0, 20000, 4510, 572]];
        assert!(validate_jeita(&rows).is_ok());
    }

    #[test]
    fn stock_ffc_float_voltage_is_accepted() {
        // The shipped FFC bands run at 4580 mV. If the envelope rejects these,
        // the vendor's own table stops being expressible as a profile.
        let rows = vec![
            vec![20, 35, 1, 2, 20000, 4580, 2667],
            vec![40, 48, 1, 0, 20000, 4580, 3239],
        ];
        assert!(validate_jeita(&rows).is_ok());
    }

    #[test]
    fn rejects_float_voltage_above_chemistry_limit() {
        // HW_FV_MAX_MV is 4750 mV (MCA_POLICY_FV_ABS_MAX_MV).
        let rows = vec![vec![20, 35, 1, 1, 20000, 4800, 572]];
        assert!(validate_jeita(&rows).is_err());
    }

    #[test]
    fn bypass_keys_include_hybrid_cp_bypass_tunables() {
        let keys: Vec<&str> = BYPASS_KEYS.iter().map(|(k, _, _)| *k).collect();
        assert!(keys.contains(&"cp_en"));
        assert!(keys.contains(&"cp_mw"));
        assert!(keys.contains(&"cp_vbat"));
    }

    #[test]
    fn rejects_charging_above_cell_temp_ceiling() {
        let rows = vec![vec![48, 75, 1, 0, 20000, 4510, 572]];
        assert!(validate_jeita(&rows).is_err());
    }

    #[test]
    fn rejects_inverted_band() {
        let rows = vec![vec![55, 48, 1, 0, 20000, 4510, 572]];
        assert!(validate_jeita(&rows).is_err());
    }
}
