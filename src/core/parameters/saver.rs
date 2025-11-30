//! Debounced parameter save task
//!
//! This module provides an async task that debounces parameter save operations,
//! batching multiple parameter changes into a single Flash write to reduce wear.
//!
//! This module requires Embassy runtime and is only available on embedded targets.

#![cfg(feature = "embassy")]

use super::registry::ParameterRegistry;
use crate::platform::traits::flash::FlashInterface;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};

/// Save request message
#[derive(Debug, Clone, Copy)]
pub enum SaveRequest {
    /// Schedule a save (will be debounced)
    Schedule,
    /// Force immediate save (bypass debounce)
    Immediate,
}

/// Parameter save manager
///
/// Manages debounced saves to Flash storage. Multiple save requests
/// within the debounce window are batched into a single Flash write.
pub struct ParamSaver {
    /// Channel for receiving save requests
    channel: &'static Channel<CriticalSectionRawMutex, SaveRequest, 4>,
}

impl ParamSaver {
    /// Create new parameter saver with global channel
    pub fn new(channel: &'static Channel<CriticalSectionRawMutex, SaveRequest, 4>) -> Self {
        Self { channel }
    }

    /// Schedule a save (debounced)
    pub async fn schedule_save(&self) {
        self.channel.send(SaveRequest::Schedule).await;
    }

    /// Request immediate save (bypass debounce)
    pub async fn save_immediately(&self) {
        self.channel.send(SaveRequest::Immediate).await;
    }

    /// Run the save task (call from async executor)
    ///
    /// This task receives save requests via the channel and debounces them.
    /// Multiple Schedule requests within 5 seconds are batched into a single save.
    pub async fn run_task<F: FlashInterface>(
        &self,
        registry: &'static embassy_sync::mutex::Mutex<
            CriticalSectionRawMutex,
            ParameterRegistry<F>,
        >,
        debounce_ms: u64,
    ) {
        loop {
            // Wait for save request
            let request = self.channel.receive().await;

            match request {
                SaveRequest::Schedule => {
                    // Start debounce timer
                    let mut pending = true;

                    while pending {
                        // Wait for debounce period or new request
                        match embassy_futures::select::select(
                            Timer::after(Duration::from_millis(debounce_ms)),
                            self.channel.receive(),
                        )
                        .await
                        {
                            embassy_futures::select::Either::First(_) => {
                                // Timer expired, perform save
                                pending = false;
                            }
                            embassy_futures::select::Either::Second(new_request) => {
                                match new_request {
                                    SaveRequest::Schedule => {
                                        // Reset timer (continue loop)
                                    }
                                    SaveRequest::Immediate => {
                                        // Break out and save immediately
                                        pending = false;
                                    }
                                }
                            }
                        }
                    }

                    // Perform the save
                    self.execute_save(registry).await;
                }
                SaveRequest::Immediate => {
                    // Save immediately without debounce
                    self.execute_save(registry).await;
                }
            }
        }
    }

    /// Execute save operation
    async fn execute_save<F: FlashInterface>(
        &self,
        registry: &'static embassy_sync::mutex::Mutex<
            CriticalSectionRawMutex,
            ParameterRegistry<F>,
        >,
    ) {
        let mut reg = registry.lock().await;

        if !reg.has_modified() {
            crate::log_debug!("No modified parameters, skipping save");
            return;
        }

        crate::log_info!("Saving parameters to Flash...");

        match reg.save_to_flash() {
            Ok(_) => {
                crate::log_info!("Parameters saved successfully");
            }
            Err(_e) => {
                crate::log_error!("Failed to save parameters");
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::parameters::{ParamMetadata, ParamValue};
    use crate::platform::mock::flash::MockFlash;

    #[test]
    fn test_save_request_types() {
        // Just verify enum can be created
        let _schedule = SaveRequest::Schedule;
        let _immediate = SaveRequest::Immediate;
    }

    // Note: Full async task testing requires Embassy runtime,
    // which is not available in unit tests. Integration tests
    // should be run on hardware or in embassy-executor test harness.
}
