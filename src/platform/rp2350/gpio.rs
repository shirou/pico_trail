//! RP2350 GPIO implementation
//!
//! This module provides GPIO support for RP2350 using the `rp235x-hal` crate.

use crate::platform::{
    Result,
    error::{GpioError, PlatformError},
    traits::{GpioInterface, GpioMode},
};
use rp235x_hal::gpio::{FunctionNull, Pin, PinId};

/// RP2350 GPIO implementation
///
/// Wraps the `rp235x-hal` GPIO pin to implement the `GpioInterface` trait.
pub struct Rp2350Gpio<I: PinId, M: PinMode> {
    pin: Pin<I, M>,
    mode: GpioMode,
}

impl<I: PinId, M: PinMode> Rp2350Gpio<I, M> {
    /// Create a new RP2350 GPIO instance
    ///
    /// # Arguments
    ///
    /// * `pin` - The HAL GPIO pin
    /// * `mode` - Initial GPIO mode
    pub fn new(pin: Pin<I, M>, mode: GpioMode) -> Self {
        Self { pin, mode }
    }
}

// Implementation for output pins
impl<I: PinId> GpioInterface for Rp2350Gpio<I, rp235x_hal::gpio::FunctionSioOutput> {
    fn set_high(&mut self) -> Result<()> {
        use rp235x_hal::gpio::PinState;
        self.pin.set_output_state(PinState::High);
        Ok(())
    }

    fn set_low(&mut self) -> Result<()> {
        use rp235x_hal::gpio::PinState;
        self.pin.set_output_state(PinState::Low);
        Ok(())
    }

    fn toggle(&mut self) -> Result<()> {
        self.pin.toggle_output_state();
        Ok(())
    }

    fn read(&self) -> bool {
        self.pin.is_high()
    }

    fn set_mode(&mut self, mode: GpioMode) -> Result<()> {
        // Runtime mode changes require pin reconfiguration
        // For now, we store the mode but don't change the hardware configuration
        // Full implementation would require Pin type conversions which is complex
        self.mode = mode;
        Ok(())
    }

    fn mode(&self) -> GpioMode {
        self.mode
    }
}

// Implementation for input pins
impl<I: PinId> GpioInterface for Rp2350Gpio<I, rp235x_hal::gpio::FunctionSioInput> {
    fn set_high(&mut self) -> Result<()> {
        Err(PlatformError::Gpio(GpioError::InvalidMode))
    }

    fn set_low(&mut self) -> Result<()> {
        Err(PlatformError::Gpio(GpioError::InvalidMode))
    }

    fn toggle(&mut self) -> Result<()> {
        Err(PlatformError::Gpio(GpioError::InvalidMode))
    }

    fn read(&self) -> bool {
        self.pin.is_high()
    }

    fn set_mode(&mut self, mode: GpioMode) -> Result<()> {
        // Runtime mode changes require pin reconfiguration
        // For now, we store the mode but don't change the hardware configuration
        self.mode = mode;
        Ok(())
    }

    fn mode(&self) -> GpioMode {
        self.mode
    }
}

// Helper functions for pin configuration
impl<I: PinId, M: PinMode> Rp2350Gpio<I, M> {
    /// Convert to output mode (helper for initialization)
    pub fn into_output(self) -> Rp2350Gpio<I, rp235x_hal::gpio::FunctionSioOutput>
    where
        M: rp235x_hal::gpio::ValidFunction<I>,
    {
        let pin = self
            .pin
            .into_function::<FunctionNull>()
            .into_push_pull_output();
        Rp2350Gpio {
            pin,
            mode: GpioMode::OutputPushPull,
        }
    }

    /// Convert to input mode (helper for initialization)
    pub fn into_input(self) -> Rp2350Gpio<I, rp235x_hal::gpio::FunctionSioInput>
    where
        M: rp235x_hal::gpio::ValidFunction<I>,
    {
        let pin = self
            .pin
            .into_function::<FunctionNull>()
            .into_floating_input();
        Rp2350Gpio {
            pin,
            mode: GpioMode::Input,
        }
    }

    /// Convert to input with pull-up (helper for initialization)
    pub fn into_pull_up_input(self) -> Rp2350Gpio<I, rp235x_hal::gpio::FunctionSioInput>
    where
        M: rp235x_hal::gpio::ValidFunction<I>,
    {
        let pin = self
            .pin
            .into_function::<FunctionNull>()
            .into_pull_up_input();
        Rp2350Gpio {
            pin,
            mode: GpioMode::InputPullUp,
        }
    }

    /// Convert to input with pull-down (helper for initialization)
    pub fn into_pull_down_input(self) -> Rp2350Gpio<I, rp235x_hal::gpio::FunctionSioInput>
    where
        M: rp235x_hal::gpio::ValidFunction<I>,
    {
        let pin = self
            .pin
            .into_function::<FunctionNull>()
            .into_pull_down_input();
        Rp2350Gpio {
            pin,
            mode: GpioMode::InputPullDown,
        }
    }
}
