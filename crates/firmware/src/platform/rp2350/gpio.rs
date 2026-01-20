//! RP2350 GPIO implementation
//!
//! This module provides GPIO support for RP2350 using the `rp235x-hal` crate.

use crate::platform::{
    error::{GpioError, PlatformError},
    traits::{GpioInterface, GpioMode},
    Result,
};
use rp235x_hal::gpio::{
    FunctionNull, FunctionSioInput, FunctionSioOutput, Pin, PinId, PullDown, PullNone, PullType,
    PullUp,
};

/// RP2350 GPIO implementation
///
/// Wraps the `rp235x-hal` GPIO pin to implement the `GpioInterface` trait.
pub struct Rp2350Gpio<I: PinId, F: rp235x_hal::gpio::Function, P: PullType> {
    pin: Pin<I, F, P>,
    mode: GpioMode,
}

impl<I: PinId, F: rp235x_hal::gpio::Function, P: PullType> Rp2350Gpio<I, F, P> {
    /// Create a new RP2350 GPIO instance
    ///
    /// # Arguments
    ///
    /// * `pin` - The HAL GPIO pin
    /// * `mode` - Initial GPIO mode
    pub fn new(pin: Pin<I, F, P>, mode: GpioMode) -> Self {
        Self { pin, mode }
    }
}

// Implementation for output pins
impl<I: PinId, P: PullType> GpioInterface for Rp2350Gpio<I, FunctionSioOutput, P> {
    fn set_high(&mut self) -> Result<()> {
        use embedded_hal::digital::v2::OutputPin;
        self.pin
            .set_high()
            .map_err(|_| PlatformError::Gpio(GpioError::HardwareError))
    }

    fn set_low(&mut self) -> Result<()> {
        use embedded_hal::digital::v2::OutputPin;
        self.pin
            .set_low()
            .map_err(|_| PlatformError::Gpio(GpioError::HardwareError))
    }

    fn toggle(&mut self) -> Result<()> {
        use embedded_hal::digital::v2::ToggleableOutputPin;
        self.pin
            .toggle()
            .map_err(|_| PlatformError::Gpio(GpioError::HardwareError))
    }

    fn read(&self) -> bool {
        use embedded_hal::digital::v2::InputPin;
        self.pin.is_high().unwrap_or(false)
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
impl<I: PinId, P: PullType> GpioInterface for Rp2350Gpio<I, FunctionSioInput, P> {
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
        use embedded_hal::digital::v2::InputPin;
        self.pin.is_high().unwrap_or(false)
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
impl<I: PinId, F: rp235x_hal::gpio::Function, P: PullType> Rp2350Gpio<I, F, P> {
    /// Convert to output mode (helper for initialization)
    pub fn into_output(self) -> Rp2350Gpio<I, FunctionSioOutput, PullNone>
    where
        I: rp235x_hal::gpio::ValidFunction<FunctionNull>,
        I: rp235x_hal::gpio::ValidFunction<FunctionSioOutput>,
    {
        let pin = self
            .pin
            .into_function::<FunctionNull>()
            .into_push_pull_output()
            .into_pull_type::<PullNone>();
        Rp2350Gpio {
            pin,
            mode: GpioMode::OutputPushPull,
        }
    }

    /// Convert to input mode (helper for initialization)
    pub fn into_input(self) -> Rp2350Gpio<I, FunctionSioInput, PullNone>
    where
        I: rp235x_hal::gpio::ValidFunction<FunctionNull>,
        I: rp235x_hal::gpio::ValidFunction<FunctionSioInput>,
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
    pub fn into_pull_up_input(self) -> Rp2350Gpio<I, FunctionSioInput, PullUp>
    where
        I: rp235x_hal::gpio::ValidFunction<FunctionNull>,
        I: rp235x_hal::gpio::ValidFunction<FunctionSioInput>,
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
    pub fn into_pull_down_input(self) -> Rp2350Gpio<I, FunctionSioInput, PullDown>
    where
        I: rp235x_hal::gpio::ValidFunction<FunctionNull>,
        I: rp235x_hal::gpio::ValidFunction<FunctionSioInput>,
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
