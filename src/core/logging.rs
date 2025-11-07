//! Logging abstraction
//!
//! Provides unified logging macros that work across different targets:
//! - Embedded (pico2_w): Uses defmt
//! - Host tests: Uses println!
//! - Host non-test: No-op

/// Log informational message
#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => {
        #[cfg(feature = "pico2_w")]
        ::defmt::info!($($arg)*);

        #[cfg(all(not(feature = "pico2_w"), test))]
        println!("[INFO] {}", format!($($arg)*));
    };
}

/// Log warning message
#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => {
        #[cfg(feature = "pico2_w")]
        ::defmt::warn!($($arg)*);

        #[cfg(all(not(feature = "pico2_w"), test))]
        println!("[WARN] {}", format!($($arg)*));
    };
}

/// Log error message
#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => {
        #[cfg(feature = "pico2_w")]
        ::defmt::error!($($arg)*);

        #[cfg(all(not(feature = "pico2_w"), test))]
        eprintln!("[ERROR] {}", format!($($arg)*));
    };
}

/// Log debug message
#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => {
        #[cfg(feature = "pico2_w")]
        ::defmt::debug!($($arg)*);

        #[cfg(all(not(feature = "pico2_w"), test))]
        println!("[DEBUG] {}", format!($($arg)*));
    };
}

/// Log trace message
#[macro_export]
macro_rules! log_trace {
    ($($arg:tt)*) => {
        #[cfg(feature = "pico2_w")]
        ::defmt::trace!($($arg)*);

        #[cfg(all(not(feature = "pico2_w"), test))]
        println!("[TRACE] {}", format!($($arg)*));
    };
}
