// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Zero-copy sysfs I/O layer for MiThermalEngine.
//!
//! Key optimizations:
//! - Pre-opened file descriptor pool (opened lazily only for active thermal zones)
//! - `pread()`/`pwrite()` at offset 0 (avoids seek + open/close per cycle)
//! - Stack-allocated read/write buffers (zero heap allocation on hot path)

use std::borrow::Cow;
use std::fs::{File, OpenOptions};

use log::{error, warn};

/// Maximum bytes we ever read from a sysfs node (temperature values, freq values).
const READ_BUF_SIZE: usize = 64;

/// A pre-opened file descriptor for a sysfs node.
/// Uses pread/pwrite to avoid seek + read/write overhead.
pub struct SysfsFd {
    file: File,
    path: Cow<'static, str>,
}

impl SysfsFd {
    /// Open a sysfs node for reading only.
    pub fn open_ro(path: &'static str) -> Option<Self> {
        Self::open_ro_cow(Cow::Borrowed(path))
    }

    /// Open a dynamically constructed sysfs path for reading only (no `Box::leak`).
    pub fn open_ro_owned(path: String) -> Option<Self> {
        Self::open_ro_cow(Cow::Owned(path))
    }

    fn open_ro_cow(path: Cow<'static, str>) -> Option<Self> {
        match File::open(path.as_ref()) {
            Ok(file) => Some(SysfsFd { file, path }),
            Err(e) => {
                warn!("Failed to open {}: {}", path, e);
                None
            }
        }
    }

    /// Open a sysfs node for read-write.
    pub fn open_rw(path: &'static str) -> Option<Self> {
        Self::open_rw_cow(Cow::Borrowed(path))
    }

    /// Open a dynamically constructed sysfs path for read-write (no `Box::leak`).
    pub fn open_rw_owned(path: String) -> Option<Self> {
        Self::open_rw_cow(Cow::Owned(path))
    }

    fn open_rw_cow(path: Cow<'static, str>) -> Option<Self> {
        match OpenOptions::new().read(true).write(true).open(path.as_ref()) {
            Ok(file) => Some(SysfsFd { file, path }),
            Err(e) => {
                warn!("Failed to open rw {}: {}", path, e);
                None
            }
        }
    }

    /// Read an integer value from the sysfs node.
    /// Uses pread at offset 0 — no seek needed, no allocation.
    pub fn read_int(&self) -> Option<i64> {
        let mut buf = [0u8; READ_BUF_SIZE];
        match nix::sys::uio::pread(&self.file, &mut buf, 0) {
            Ok(n) if n > 0 => parse_int_from_bytes(&buf[..n]),
            Ok(_) => None,
            Err(e) => {
                if e != nix::errno::Errno::EAGAIN {
                    error!("pread {}: {}", self.path, e);
                }
                None
            }
        }
    }

    /// Read raw string content from sysfs node.
    #[allow(dead_code)]
    pub fn read_str<'a>(&self, buf: &'a mut [u8]) -> Option<&'a str> {
        match nix::sys::uio::pread(&self.file, buf, 0) {
            Ok(n) if n > 0 => {
                let s = std::str::from_utf8(&buf[..n]).ok()?;
                Some(s.trim())
            }
            _ => None,
        }
    }

    /// Write an integer value to the sysfs node.
    /// Uses pwrite at offset 0 — no seek needed, zero heap allocation.
    pub fn write_int(&self, value: i64) {
        let mut buf = [0u8; 24];
        let s = int_to_bytes(value, &mut buf);
        if let Err(e) = nix::sys::uio::pwrite(&self.file, s, 0) {
            error!("pwrite {} = {}: {}", self.path, value, e);
        }
    }

    /// Write a string value to the sysfs node.
    pub fn write_str(&self, value: &str) {
        if let Err(e) = nix::sys::uio::pwrite(&self.file, value.as_bytes(), 0) {
            error!("pwrite {} = {}: {}", self.path, value, e);
        }
    }

    /// Write `"<prefix> <value>"` on the stack — no heap allocation.
    pub fn write_prefixed_int(&self, prefix: &str, value: i64) {
        let mut buf = [0u8; PREFIXED_BUF_SIZE];
        let bytes = match format_prefixed_int(prefix, value, &mut buf) {
            Some(b) => b,
            None => {
                error!("prefix {:?} too long for {}", prefix, self.path);
                return;
            }
        };
        if let Err(e) = nix::sys::uio::pwrite(&self.file, bytes, 0) {
            error!("pwrite {} = {:?} {}: {}", self.path, prefix, value, e);
        }
    }

    /// Write `"<prefix> <a_val>#<b_val>"` on the stack — no heap allocation.
    pub fn write_prefixed_pair(&self, prefix: &str, a: i64, b: i64) {
        let mut buf = [0u8; PREFIXED_BUF_SIZE];
        let plen = prefix.len();
        let mut na = [0u8; 24];
        let mut nb = [0u8; 24];
        let da = int_to_bytes(a, &mut na);
        let db = int_to_bytes(b, &mut nb);
        let end = plen + 1 + da.len() + 1 + db.len();
        if end > buf.len() {
            error!("prefix {:?} pair too long for {}", prefix, self.path);
            return;
        }
        buf[..plen].copy_from_slice(prefix.as_bytes());
        buf[plen] = b' ';
        let p1 = plen + 1;
        buf[p1..p1 + da.len()].copy_from_slice(da);
        buf[p1 + da.len()] = b'#';
        let p2 = p1 + da.len() + 1;
        buf[p2..end].copy_from_slice(db);
        if let Err(e) = nix::sys::uio::pwrite(&self.file, &buf[..end], 0) {
            error!("pwrite {} = {:?} {}#{}: {}", self.path, prefix, a, b, e);
        }
    }

    #[allow(dead_code)]
    pub fn path(&self) -> &str {
        self.path.as_ref()
    }
}

/// Parse integer from raw bytes without String allocation.
fn parse_int_from_bytes(bytes: &[u8]) -> Option<i64> {
    let mut i = 0;
    let len = bytes.len();

    while i < len && bytes[i].is_ascii_whitespace() {
        i += 1;
    }
    if i >= len {
        return None;
    }

    let negative = bytes[i] == b'-';
    if negative || bytes[i] == b'+' {
        i += 1;
    }

    let mut result: i64 = 0;
    let mut found_digit = false;
    while i < len && bytes[i].is_ascii_digit() {
        result = result.wrapping_mul(10).wrapping_add((bytes[i] - b'0') as i64);
        found_digit = true;
        i += 1;
    }

    if !found_digit {
        return None;
    }

    Some(if negative { -result } else { result })
}

const PREFIXED_BUF_SIZE: usize = 64;

fn format_prefixed_int<'a>(
    prefix: &str,
    value: i64,
    buf: &'a mut [u8; PREFIXED_BUF_SIZE],
) -> Option<&'a [u8]> {
    let plen = prefix.len();
    let mut num = [0u8; 24];
    let digits = int_to_bytes(value, &mut num);
    let end = plen + 1 + digits.len();
    if end > buf.len() {
        return None;
    }
    buf[..plen].copy_from_slice(prefix.as_bytes());
    buf[plen] = b' ';
    buf[plen + 1..end].copy_from_slice(digits);
    Some(&buf[..end])
}

fn int_to_bytes(mut value: i64, buf: &mut [u8; 24]) -> &[u8] {
    if value == 0 {
        buf[0] = b'0';
        return &buf[..1];
    }

    let negative = value < 0;
    if negative {
        value = -value;
    }

    let mut pos = 24;
    while value > 0 {
        pos -= 1;
        buf[pos] = b'0' + (value % 10) as u8;
        value /= 10;
    }

    if negative {
        pos -= 1;
        buf[pos] = b'-';
    }

    &buf[pos..]
}

/// Discover thermal zones (`type_name -> temp_path`) WITHOUT holding 70+ unused kernel FDs open.
/// Only zones actually bound to a sensor role open their `/temp` fd.
pub fn discover_thermal_zones() -> Vec<(String, String)> {
    let mut zones = Vec::new();
    for i in 0..128 {
        let type_path = format!("/sys/class/thermal/thermal_zone{}/type", i);
        let type_fd = match File::open(&type_path) {
            Ok(f) => f,
            Err(_) => break,
        };
        let mut type_buf = [0u8; 64];
        let type_name = match nix::sys::uio::pread(&type_fd, &mut type_buf, 0) {
            Ok(n) if n > 0 => String::from_utf8_lossy(&type_buf[..n]).trim().to_string(),
            _ => continue,
        };
        let temp_path = format!("/sys/class/thermal/thermal_zone{}/temp", i);
        zones.push((type_name, temp_path));
    }
    zones
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_int() {
        assert_eq!(parse_int_from_bytes(b"42000\n"), Some(42000));
        assert_eq!(parse_int_from_bytes(b"-1234\n"), Some(-1234));
        assert_eq!(parse_int_from_bytes(b"  38110 \n"), Some(38110));
        assert_eq!(parse_int_from_bytes(b"0\n"), Some(0));
        assert_eq!(parse_int_from_bytes(b"\n"), None);
    }

    #[test]
    fn test_int_to_bytes() {
        let mut buf = [0u8; 24];
        assert_eq!(int_to_bytes(42000, &mut buf), b"42000");
        assert_eq!(int_to_bytes(-1234, &mut buf), b"-1234");
        assert_eq!(int_to_bytes(0, &mut buf), b"0");
    }

    #[test]
    fn test_format_prefixed_int() {
        let mut buf = [0u8; PREFIXED_BUF_SIZE];
        assert_eq!(
            format_prefixed_int("mithermal buck", 1854, &mut buf),
            Some(&b"mithermal buck 1854"[..])
        );
        assert_eq!(
            format_prefixed_int("mithermal buck", 0, &mut buf),
            Some(&b"mithermal buck 0"[..])
        );
    }

    #[test]
    fn test_format_prefixed_int_rejects_overflow() {
        let mut buf = [0u8; PREFIXED_BUF_SIZE];
        let long = "x".repeat(PREFIXED_BUF_SIZE);
        assert_eq!(format_prefixed_int(&long, 1, &mut buf), None);
    }
}
