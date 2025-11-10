//! Logging abstraction
//!
//! Provides unified logging macros that work across different targets:
//! - Embedded (pico2_w + usb_serial feature): Uses USB Serial
//! - Embedded (pico2_w): Uses defmt
//! - Host tests: Uses println!
//! - Host non-test: No-op
//!
//! ## USB Serial Logging
//!
//! When the `usb_serial` feature is enabled, logs are sent to USB Serial instead of defmt.
//! You need to:
//! 1. Initialize USB Serial in your main function
//! 2. Spawn the logger task: `spawner.spawn(usb_logger_task(usb_class))`
//! 3. Use the log macros as usual: `log_info!("message")`

#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
use embassy_sync::channel::Channel;

#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
extern crate alloc;

/// Log message buffer size
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
const LOG_MSG_SIZE: usize = 256;

/// Log channel capacity
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
const LOG_CHANNEL_SIZE: usize = 16;

/// Log message type
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
pub struct LogMessage {
    pub level: LogLevel,
    pub message: heapless::String<LOG_MSG_SIZE>,
}

/// Log level
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
#[derive(Debug, Clone, Copy)]
pub enum LogLevel {
    Info,
    Warn,
    Error,
    Debug,
    Trace,
}

/// Global log channel
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
static LOG_CHANNEL: Channel<CriticalSectionRawMutex, LogMessage, LOG_CHANNEL_SIZE> = Channel::new();

// ============================================================================
// defmt transport implementation for USB Serial
// ============================================================================
//
// When usb_serial feature is enabled, we provide defmt symbols but redirect
// all logging through our own crate::log_*! macros instead.
// This allows the linker to succeed while using USB Serial for all logs.

#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
use core::sync::atomic::{AtomicBool, Ordering};

#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
static DEFMT_LOCKED: AtomicBool = AtomicBool::new(false);

/// defmt acquire - no-op implementation for USB Serial
///
/// We don't actually use defmt when usb_serial is enabled,
/// but we need to provide these symbols for the linker.
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
#[no_mangle]
unsafe extern "C" fn _defmt_acquire() {
    // Simple spinlock
    while DEFMT_LOCKED
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        core::hint::spin_loop();
    }
}

/// defmt release - no-op implementation for USB Serial
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
#[no_mangle]
unsafe extern "C" fn _defmt_release() {
    DEFMT_LOCKED.store(false, Ordering::Release);
}

/// defmt write - no-op implementation for USB Serial
///
/// We ignore defmt's binary data since we use crate::log_*! macros instead.
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
#[no_mangle]
unsafe extern "C" fn _defmt_write(_bytes: *const u8, _len: usize) {
    // No-op: we use crate::log_*! macros for all logging
}

/// Send log message to channel
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
pub fn send_log(level: LogLevel, msg: &str) {
    use heapless::String;

    let mut message = String::new();
    let _ = core::fmt::write(&mut message, format_args!("{}", msg));

    let log_msg = LogMessage { level, message };

    // Try to send, drop if channel is full (non-blocking)
    let _ = LOG_CHANNEL.try_send(log_msg);
}

/// USB Serial logger task
///
/// Spawn this task to enable USB Serial logging.
///
/// # Example
///
/// ```no_run
/// use embassy_usb::class::cdc_acm::CdcAcmClass;
/// use pico_trail::core::logging::usb_logger_task;
///
/// #[embassy_executor::main]
/// async fn main(spawner: Spawner) {
///     // ... USB initialization ...
///     let mut cdc_class = CdcAcmClass::new(&mut builder, &STATE, 64);
///
///     // Spawn logger task
///     spawner.spawn(usb_logger_task(cdc_class)).unwrap();
///
///     // ... rest of your code ...
/// }
/// ```
#[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
#[embassy_executor::task]
pub async fn usb_logger_task(
    mut usb_class: embassy_usb::class::cdc_acm::CdcAcmClass<
        'static,
        embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>,
    >,
) {
    use embassy_time::{Duration, Timer};

    // Wait for USB to be configured
    Timer::after(Duration::from_secs(1)).await;

    loop {
        let log_msg = LOG_CHANNEL.receive().await;

        let level_str = match log_msg.level {
            LogLevel::Info => "[INFO] ",
            LogLevel::Warn => "[WARN] ",
            LogLevel::Error => "[ERROR]",
            LogLevel::Debug => "[DEBUG]",
            LogLevel::Trace => "[TRACE]",
        };

        let mut buf = heapless::String::<{ LOG_MSG_SIZE + 32 }>::new();
        let _ = core::fmt::write(
            &mut buf,
            format_args!("{} {}\r\n", level_str, log_msg.message),
        );

        let _ = usb_class.write_packet(buf.as_bytes()).await;
    }
}

/// Log informational message
#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => {{
        #[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
        {
            extern crate alloc;
            use alloc::format;
            $crate::core::logging::send_log(
                $crate::core::logging::LogLevel::Info,
                &format!($($arg)*)
            );
        }

        #[cfg(all(feature = "pico2_w", not(feature = "usb_serial")))]
        ::defmt::info!($($arg)*);

        #[cfg(all(not(feature = "pico2_w"), test))]
        println!("[INFO] {}", format!($($arg)*));
    }};
}

/// Log warning message
#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => {{
        #[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
        {
            extern crate alloc;
            use alloc::format;
            $crate::core::logging::send_log(
                $crate::core::logging::LogLevel::Warn,
                &format!($($arg)*)
            );
        }

        #[cfg(all(feature = "pico2_w", not(feature = "usb_serial")))]
        ::defmt::warn!($($arg)*);

        #[cfg(all(not(feature = "pico2_w"), test))]
        println!("[WARN] {}", format!($($arg)*));
    }};
}

/// Log error message
#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => {{
        #[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
        {
            extern crate alloc;
            use alloc::format;
            $crate::core::logging::send_log(
                $crate::core::logging::LogLevel::Error,
                &format!($($arg)*)
            );
        }

        #[cfg(all(feature = "pico2_w", not(feature = "usb_serial")))]
        ::defmt::error!($($arg)*);

        #[cfg(all(not(feature = "pico2_w"), test))]
        eprintln!("[ERROR] {}", format!($($arg)*));
    }};
}

/// Log debug message
#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => {{
        #[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
        {
            extern crate alloc;
            use alloc::format;
            $crate::core::logging::send_log(
                $crate::core::logging::LogLevel::Debug,
                &format!($($arg)*)
            );
        }

        #[cfg(all(feature = "pico2_w", not(feature = "usb_serial")))]
        ::defmt::debug!($($arg)*);

        #[cfg(all(not(feature = "pico2_w"), test))]
        println!("[DEBUG] {}", format!($($arg)*));
    }};
}

/// Log trace message
#[macro_export]
macro_rules! log_trace {
    ($($arg:tt)*) => {{
        #[cfg(all(feature = "pico2_w", feature = "usb_serial"))]
        {
            extern crate alloc;
            use alloc::format;
            $crate::core::logging::send_log(
                $crate::core::logging::LogLevel::Trace,
                &format!($($arg)*)
            );
        }

        #[cfg(all(feature = "pico2_w", not(feature = "usb_serial")))]
        ::defmt::trace!($($arg)*);

        #[cfg(all(not(feature = "pico2_w"), test))]
        println!("[TRACE] {}", format!($($arg)*));
    }};
}
