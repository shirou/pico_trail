//! GPIO Abstractions for BNO086 INT and RST Pins
//!
//! This module provides trait abstractions for the BNO086's GPIO pins:
//! - INT (Interrupt): Active low, signals data ready
//! - RST (Reset): Active low, triggers hardware reset
//!
//! These abstractions allow the driver to work with Embassy GPIO (embedded)
//! and mock implementations (testing).

/// Trait for interrupt pin (INT) with async edge detection
///
/// The BNO086 INT pin is active low and signals data ready on falling edge.
/// Implementations should wait for the falling edge asynchronously.
#[allow(async_fn_in_trait)]
pub trait IntPin {
    /// Wait for falling edge (data ready signal)
    ///
    /// This method blocks asynchronously until the INT pin transitions
    /// from high to low, indicating data is ready to read.
    async fn wait_for_falling_edge(&mut self);

    /// Check if INT is currently low (data ready)
    ///
    /// Non-blocking check of current pin state.
    fn is_low(&self) -> bool;
}

/// Trait for reset pin (RST) with output control
///
/// The BNO086 RST pin is active low. Assert low to reset, high to run.
#[allow(async_fn_in_trait)]
pub trait RstPin {
    /// Set RST pin low (assert reset)
    fn set_low(&mut self);

    /// Set RST pin high (release reset)
    fn set_high(&mut self);
}

/// No-op INT pin for polling mode
///
/// Use this when INT pin is not connected. All operations are no-ops.
#[derive(Debug, Default)]
pub struct NoIntPin;

impl IntPin for NoIntPin {
    async fn wait_for_falling_edge(&mut self) {
        // No-op: polling mode doesn't wait for INT
    }

    fn is_low(&self) -> bool {
        // Always return true to allow reads
        true
    }
}

/// No-op RST pin when hardware reset is not available
///
/// Use this when RST pin is not connected. All operations are no-ops.
#[derive(Debug, Default)]
pub struct NoRstPin;

impl RstPin for NoRstPin {
    fn set_low(&mut self) {
        // No-op: no hardware reset capability
    }

    fn set_high(&mut self) {
        // No-op: no hardware reset capability
    }
}

// =============================================================================
// Embassy GPIO Implementations (RP2350)
// =============================================================================

/// INT pin implementation using Embassy GPIO
pub struct EmbassyIntPin<'d> {
    pin: embassy_rp::gpio::Input<'d>,
}

impl<'d> EmbassyIntPin<'d> {
    /// Create INT pin from Embassy GPIO input
    ///
    /// The pin should be configured with internal pull-up since BNO086
    /// INT is open-drain active low.
    pub fn new(pin: embassy_rp::gpio::Input<'d>) -> Self {
        Self { pin }
    }
}

impl IntPin for EmbassyIntPin<'_> {
    async fn wait_for_falling_edge(&mut self) {
        self.pin.wait_for_falling_edge().await;
    }

    fn is_low(&self) -> bool {
        self.pin.is_low()
    }
}

/// RST pin implementation using Embassy GPIO Flex
///
/// Uses Flex pin to properly handle BNO086 RST requirements:
/// - Assert reset: Drive pin LOW
/// - Release reset: Switch to high-Z (input mode with pull-up)
///
/// The BNO086 RST pin must NOT be actively driven HIGH - it must be
/// released to high-Z and allowed to float up via external pull-up.
pub struct EmbassyRstPin<'d> {
    pin: embassy_rp::gpio::Flex<'d>,
}

impl<'d> EmbassyRstPin<'d> {
    /// Create RST pin from Embassy GPIO Flex
    ///
    /// The pin starts in high-Z (input) mode with pull-up enabled.
    /// This allows the external pull-up to keep RST high.
    pub fn new(pin: embassy_rp::gpio::Flex<'d>) -> Self {
        let mut rst = Self { pin };
        // Start in high-Z mode (input with pull-up)
        rst.pin.set_as_input();
        rst.pin.set_pull(embassy_rp::gpio::Pull::Up);
        rst
    }
}

impl RstPin for EmbassyRstPin<'_> {
    fn set_low(&mut self) {
        // Switch to output and drive LOW
        self.pin.set_as_output();
        self.pin.set_low();
    }

    fn set_high(&mut self) {
        // Release to high-Z (input mode with pull-up)
        // External pull-up will bring the line HIGH
        self.pin.set_as_input();
        self.pin.set_pull(embassy_rp::gpio::Pull::Up);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_int_pin() {
        let pin = NoIntPin;
        assert!(pin.is_low()); // Always returns true for polling mode
    }

    #[test]
    fn test_no_rst_pin() {
        let mut pin = NoRstPin;
        // Should not panic
        pin.set_low();
        pin.set_high();
    }

    #[tokio::test]
    async fn test_no_int_pin_wait() {
        let mut pin = NoIntPin;
        // Should return immediately
        pin.wait_for_falling_edge().await;
    }
}
