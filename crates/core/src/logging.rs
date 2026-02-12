//! Logging macros for core crate
//!
//! Uses `defmt` on embedded targets (when `defmt` feature is enabled),
//! no-op otherwise. Firmware crate provides its own richer logging.

/// Log informational message
#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => {{
        #[cfg(feature = "defmt")]
        ::defmt::info!($($arg)*);

        #[cfg(all(not(feature = "defmt"), test))]
        {
            // In host tests, use println for visibility
            std::println!("[INFO] {}", std::format!($($arg)*));
        }
    }};
}

/// Log warning message
#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => {{
        #[cfg(feature = "defmt")]
        ::defmt::warn!($($arg)*);

        #[cfg(all(not(feature = "defmt"), test))]
        {
            std::println!("[WARN] {}", std::format!($($arg)*));
        }
    }};
}

/// Log error message
#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => {{
        #[cfg(feature = "defmt")]
        ::defmt::error!($($arg)*);

        #[cfg(all(not(feature = "defmt"), test))]
        {
            std::println!("[ERROR] {}", std::format!($($arg)*));
        }
    }};
}

/// Log debug message
#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => {{
        #[cfg(feature = "defmt")]
        ::defmt::debug!($($arg)*);

        #[cfg(all(not(feature = "defmt"), test))]
        {
            let _ = std::format!($($arg)*);
        }
    }};
}

/// Log trace message
#[macro_export]
macro_rules! log_trace {
    ($($arg:tt)*) => {{
        #[cfg(feature = "defmt")]
        ::defmt::trace!($($arg)*);
    }};
}
