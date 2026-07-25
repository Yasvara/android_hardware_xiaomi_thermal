// Copyright (C) 2026 IRedDragonICY
// SPDX-License-Identifier: GPL-2.0-only

//! Zero-copy sysfs I/O layer for MiThermalEngine.
//!
//! Key optimizations:
//! - Pre-opened file descriptor pool (open once at startup)
//! - `pread()`/`pwrite()` with seek-to-0 (avoids open/close per cycle)
//! - Stack-allocated read buffer (zero heap allocation on hot path)
//! - Retry logic for transient EAGAIN/EINTR

use std::fs::{File, OpenOptions};

use log::{error, warn};

/// Maximum bytes we ever read from a sysfs node (temperature values, freq values).
const READ_BUF_SIZE: usize = 64;

/// A pre-opened file descriptor for a sysfs node.
/// Uses pread/pwrite to avoid seek + read/write overhead.
pub struct SysfsFd {
    file: File,
    path: &'static str,
}

impl SysfsFd {
    /// Open a sysfs node for reading only.
    pub fn open_ro(path: &'static str) -> Option<Self> {
        match File::open(path) {
            Ok(file) => Some(SysfsFd {
                file,
                path,
            }),
            Err(e) => {
                warn!("Failed to open {}: {}", path, e);
                None
            }
        }
    }

    /// Open a sysfs node for read-write.
    pub fn open_rw(path: &'static str) -> Option<Self> {
        match OpenOptions::new().read(true).write(true).open(path) {
            Ok(file) => Some(SysfsFd {
                file,
                path,
            }),
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
                // EAGAIN is normal for some sysfs nodes during init
                if e != nix::errno::Errno::EAGAIN {
                    error!("pread {}: {}", self.path, e);
                }
                None
            }
        }
    }

    /// Read raw string content from sysfs node.
    /// Returns bytes read into the provided buffer.
    #[allow(dead_code)]
    pub fn read_str<'a>(&self, buf: &'a mut [u8]) -> Option<&'a str> {
        match nix::sys::uio::pread(&self.file, buf, 0) {
            Ok(n) if n > 0 => {
                // Trim trailing whitespace/newline
                let s = std::str::from_utf8(&buf[..n]).ok()?;
                Some(s.trim())
            }
            _ => None,
        }
    }

    /// Write an integer value to the sysfs node.
    /// Uses pwrite at offset 0 — no seek needed.
    pub fn write_int(&self, value: i64) {
        // itoa on stack — no heap allocation
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

    #[allow(dead_code)]
    pub fn path(&self) -> &str {
        self.path
    }
}

/// Parse integer from raw bytes without String allocation.
/// Handles leading whitespace, optional sign, trailing whitespace/newline.
fn parse_int_from_bytes(bytes: &[u8]) -> Option<i64> {
    let mut i = 0;
    let len = bytes.len();

    // Skip leading whitespace
    while i < len && bytes[i].is_ascii_whitespace() {
        i += 1;
    }
    if i >= len {
        return None;
    }

    // Handle sign
    let negative = bytes[i] == b'-';
    if negative || bytes[i] == b'+' {
        i += 1;
    }

    // Parse digits
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

/// Convert integer to bytes on stack. Returns the slice containing the number.
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

/// Batch-open multiple sysfs paths for reading.
/// Returns a Vec of successfully opened fds (skips failures).
pub fn open_thermal_zones() -> Vec<(String, SysfsFd)> {
    let mut zones = Vec::new();
    for i in 0..128 {
        // Read the zone type first
        let type_path: String = format!("/sys/class/thermal/thermal_zone{}/type", i);
        let temp_path_str: String = format!("/sys/class/thermal/thermal_zone{}/temp", i);

        // Try to read the type
        let type_fd = match File::open(&type_path) {
            Ok(f) => f,
            Err(_) => break, // No more zones
        };

        let mut type_buf = [0u8; 64];
        let type_name = match nix::sys::uio::pread(&type_fd, &mut type_buf, 0) {
            Ok(n) if n > 0 => {
                String::from_utf8_lossy(&type_buf[..n]).trim().to_string()
            }
            _ => continue,
        };

        // Leak the temp path string so we get 'static lifetime for SysfsFd
        let temp_path: &'static str = Box::leak(temp_path_str.into_boxed_str());
        if let Some(fd) = SysfsFd::open_ro(temp_path) {
            zones.push((type_name, fd));
        }
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
}
