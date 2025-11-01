//! MAVLink Transport Abstraction
//!
//! This module provides a trait-based abstraction for MAVLink communication transports,
//! enabling support for multiple transport types (UART, UDP, TCP) with a unified interface.
//!
//! # Architecture
//!
//! The transport layer follows a trait-based design inspired by ArduPilot's `AP_HAL::UARTDriver`:
//!
//! - **Trait-based abstraction**: `MavlinkTransport` trait defines the interface
//! - **Zero-cost**: Monomorphization eliminates runtime overhead
//! - **Async-first**: All operations use Embassy async primitives
//! - **Transport-agnostic**: Protocol layer doesn't know about specific transports
//!
//! # Design Pattern
//!
//! ```text
//! ┌──────────────────────────────────────┐
//! │     MAVLink Protocol Layer           │
//! │  (parser, writer, message handlers)  │
//! └──────────┬───────────────────────────┘
//!            │
//!            │ Uses MavlinkTransport trait
//!            │
//!            ▼
//! ┌──────────────────────────────────────┐
//! │      Transport Router                 │
//! │  (manages multiple transports)        │
//! └──────────┬───────────────────────────┘
//!            │
//!            │ Holds Vec<dyn MavlinkTransport>
//!            │
//!            ▼
//! ┌─────────────────┬────────────────┬───────────────┐
//! │  UartTransport  │  UdpTransport  │ TcpTransport  │
//! │   (UART I/O)    │  (UDP socket)  │ (TCP socket)  │
//! └─────────────────┴────────────────┴───────────────┘
//! ```
//!
//! # Example Usage
//!
//! ```ignore
//! use pico_trail::communication::mavlink::transport::{MavlinkTransport, uart::UartTransport};
//!
//! async fn send_heartbeat(transport: &mut impl MavlinkTransport) {
//!     let data = [/* MAVLink HEARTBEAT message */];
//!     transport.write(&data).await.unwrap();
//!     transport.flush().await.unwrap();
//! }
//! ```
//!
//! # Modules
//!
//! - `uart` - UART transport implementation (Phase 1)
//! - `udp` - UDP transport implementation (Phase 2)
//! - `tcp` - TCP transport implementation (future)

pub mod uart;
pub mod udp;

use core::fmt;

/// Transport abstraction for MAVLink communication
///
/// This trait defines the interface for all MAVLink transports (UART, UDP, TCP).
/// Implementations must provide async read/write operations compatible with
/// Embassy's async executor.
///
/// # Design Rationale
///
/// - **Async operations**: Non-blocking I/O essential for concurrent transports
/// - **Byte-oriented**: MAVLink is a byte-stream protocol
/// - **Error handling**: Transports can fail, timeout, or disconnect
/// - **Buffering**: Implementations may buffer internally (e.g., UART DMA)
///
/// # Trait Methods
///
/// - `available()`: Check if data is ready to read (non-blocking check)
/// - `read()`: Read bytes into buffer (blocking until data available)
/// - `write()`: Write bytes from buffer (may buffer internally)
/// - `flush()`: Ensure all buffered writes are transmitted
///
/// # Implementation Guidelines
///
/// - `read()` should block until at least 1 byte is available or error occurs
/// - `write()` may buffer data; call `flush()` to ensure transmission
/// - `available()` is optional optimization; return 0 if unsupported
/// - Errors should be specific: IoError (generic), Timeout, Disconnected
#[allow(async_fn_in_trait)]
pub trait MavlinkTransport {
    /// Returns number of bytes available to read without blocking
    ///
    /// This is an optional optimization hint. Implementations that don't track
    /// available bytes can return 0.
    ///
    /// # Returns
    ///
    /// Number of bytes available in receive buffer, or 0 if unknown/unsupported
    async fn available(&self) -> usize;

    /// Read bytes from transport into buffer
    ///
    /// Blocks until at least 1 byte is read or an error occurs.
    /// May read fewer bytes than buffer size if less data is available.
    ///
    /// # Arguments
    ///
    /// * `buf` - Buffer to read into
    ///
    /// # Returns
    ///
    /// - `Ok(n)` - Number of bytes read (1 ≤ n ≤ buf.len())
    /// - `Err(TransportError)` - I/O error, timeout, or disconnection
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let mut buf = [0u8; 256];
    /// let n = transport.read(&mut buf).await?;
    /// // Process buf[..n]
    /// ```
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;

    /// Write bytes from buffer to transport
    ///
    /// May buffer data internally. Call `flush()` to ensure transmission.
    ///
    /// # Arguments
    ///
    /// * `buf` - Data to write
    ///
    /// # Returns
    ///
    /// - `Ok(n)` - Number of bytes written (typically buf.len())
    /// - `Err(TransportError)` - I/O error or disconnection
    ///
    /// # Examples
    ///
    /// ```ignore
    /// let data = [0xFE, 0x09, 0x00, /* ... */];
    /// transport.write(&data).await?;
    /// transport.flush().await?;
    /// ```
    async fn write(&mut self, buf: &[u8]) -> Result<usize, TransportError>;

    /// Flush any buffered writes to ensure transmission
    ///
    /// Blocks until all buffered data has been transmitted or an error occurs.
    /// Some transports (e.g., UDP) may not need explicit flushing.
    ///
    /// # Returns
    ///
    /// - `Ok(())` - All data transmitted
    /// - `Err(TransportError)` - Flush failed
    async fn flush(&mut self) -> Result<(), TransportError>;
}

/// Transport error types
///
/// Categorizes transport failures for appropriate error handling.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TransportError {
    /// Generic I/O error
    ///
    /// Use this for unclassified errors that don't fit other categories.
    /// Examples: hardware failure, driver error, buffer overflow
    IoError,

    /// Operation timed out
    ///
    /// The operation didn't complete within the expected time.
    /// Examples: WiFi connection timeout, UART read timeout
    Timeout,

    /// Transport disconnected
    ///
    /// The transport is no longer available for communication.
    /// Examples: UART cable unplugged, WiFi disconnected, TCP connection closed
    Disconnected,
}

impl fmt::Display for TransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            TransportError::IoError => write!(f, "I/O error"),
            TransportError::Timeout => write!(f, "Operation timed out"),
            TransportError::Disconnected => write!(f, "Transport disconnected"),
        }
    }
}

// Note: TransportError implements Display, sufficient for embedded use
