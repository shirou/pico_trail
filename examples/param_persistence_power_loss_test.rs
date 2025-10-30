//! Parameter persistence power-loss recovery test
//!
//! This example tests parameter recovery after power loss during Flash write.
//! The test continuously saves parameters and relies on manual power cycling
//! to verify corruption recovery.
//!
//! # Test Procedure
//!
//! 1. Flash this example to Pico 2 W
//! 2. Connect via probe-rs or USB-CDC to monitor logs
//! 3. Let it run for a few cycles (observe "Saving..." messages)
//! 4. Manually disconnect power during a save operation
//! 5. Reconnect power and observe recovery
//! 6. Repeat 100 times to verify < 0.1% corruption rate
//!
//! # Expected Behavior
//!
//! - On normal startup: Parameters load from Flash successfully
//! - After power loss during save: Either:
//!   a) Latest block is valid (save completed before power loss)
//!   b) Previous block is valid (falls back to backup)
//! - Corruption rate should be < 0.1% (< 1 failure per 100 power cycles)
//!
//! # Hardware
//!
//! Requires Raspberry Pi Pico 2 W (RP2350)
//!
//! # Usage
//!
//! ```bash
//! ./scripts/build-rp2350.sh --release param_persistence_power_loss_test
//! probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/param_persistence_power_loss_test
//! ```

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_time::{Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

use pico_trail::core::parameters::{ParamMetadata, ParamValue, ParameterRegistry};
use pico_trail::platform::rp2350::Rp2350Flash;

/// Save interval in seconds (frequent saves increase chance of power loss during write)
const SAVE_INTERVAL_SECS: u64 = 2;

/// Boot counter parameter name (used to track boot count)
const BOOT_COUNT_PARAM: &str = "BOOT_COUNT";

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize platform
    let _p = hal::init(Default::default());

    info!("╔════════════════════════════════════════════════════════╗");
    info!("║  Parameter Persistence Power-Loss Recovery Test        ║");
    info!("╚════════════════════════════════════════════════════════╝");
    info!("");

    // Initialize Flash and Registry
    info!("Initializing Flash storage...");
    let flash = Rp2350Flash::new();
    let mut registry = ParameterRegistry::with_flash(flash);

    // Register test parameters
    registry
        .register(ParamMetadata::new_uint32(BOOT_COUNT_PARAM, 0, 0, 1_000_000))
        .unwrap();
    registry
        .register(ParamMetadata::new_float("TEST_VALUE", 0.0, -100.0, 100.0))
        .unwrap();
    registry
        .register(ParamMetadata::new_uint32("SAVE_COUNT", 0, 0, 1_000_000))
        .unwrap();
    registry
        .register(ParamMetadata::new_uint32("RECOVERY_COUNT", 0, 0, 1_000_000))
        .unwrap();

    info!("✓ Registered {} parameters", registry.count());
    info!("");

    // Load parameters from Flash
    info!("Loading parameters from Flash...");
    let load_start = Instant::now();
    let load_result = registry.load_from_flash();

    match load_result {
        Ok(_) => {
            let load_time = load_start.elapsed().as_millis();
            info!("✓ Parameters loaded successfully in {} ms", load_time);
            info!("");

            // Display loaded values
            info!("Loaded parameters:");
            for i in 0..registry.count() {
                if let Some(param) = registry.get_by_index(i) {
                    match param.value {
                        ParamValue::Float(f) => info!("  {} = {}", param.name, f),
                        ParamValue::Uint32(u) => info!("  {} = {}", param.name, u),
                    }
                }
            }
            info!("");

            // Check if this is a recovery from power loss
            if let Some(stats) = registry.storage_stats() {
                if stats.total_saves > 0 {
                    info!("✓ RECOVERY SUCCESSFUL");
                    info!("  Parameters recovered from Flash");
                    info!("  Active block: {}", stats.active_block);
                    info!("");

                    // Increment recovery counter
                    if let Some(param) = registry.get_by_name("RECOVERY_COUNT") {
                        if let ParamValue::Uint32(count) = param.value {
                            registry
                                .set_by_name("RECOVERY_COUNT", ParamValue::Uint32(count + 1))
                                .ok();
                        }
                    }
                }
            }
        }
        Err(e) => {
            warn!("Load failed (using defaults): {:?}", e);
            warn!("This may indicate Flash corruption or first boot");
            info!("");
        }
    }

    // Increment boot counter
    let boot_count = if let Some(param) = registry.get_by_name(BOOT_COUNT_PARAM) {
        if let ParamValue::Uint32(count) = param.value {
            let new_count = count + 1;
            registry
                .set_by_name(BOOT_COUNT_PARAM, ParamValue::Uint32(new_count))
                .unwrap();
            new_count
        } else {
            1
        }
    } else {
        1
    };

    info!("═══════════════════════════════════════════════════════");
    info!("Boot Count: {}", boot_count);
    info!("═══════════════════════════════════════════════════════");
    info!("");

    // Save boot count immediately
    info!("Saving boot count to Flash...");
    match registry.save_to_flash() {
        Ok(_) => info!("✓ Boot count saved"),
        Err(e) => error!("✗ Failed to save boot count: {:?}", e),
    }
    info!("");

    // Display test instructions
    info!("Power-Loss Test Instructions:");
    info!("1. Let the device run and observe 'Saving...' messages");
    info!("2. Disconnect power DURING a save operation");
    info!("3. Reconnect power and check recovery");
    info!("4. Repeat 100 times to verify < 0.1% corruption rate");
    info!("");
    info!("Current Statistics:");
    if let Some(param) = registry.get_by_name("SAVE_COUNT") {
        if let ParamValue::Uint32(count) = param.value {
            info!("  Total saves: {}", count);
        }
    }
    if let Some(param) = registry.get_by_name("RECOVERY_COUNT") {
        if let ParamValue::Uint32(count) = param.value {
            info!("  Successful recoveries: {}", count);
            if boot_count > 1 {
                let recovery_rate = (count * 100) / (boot_count - 1);
                info!("  Recovery rate: {}%", recovery_rate);
            }
        }
    }
    info!("");

    // Main test loop
    info!("Starting continuous save loop...");
    info!("Save interval: {} seconds", SAVE_INTERVAL_SECS);
    info!("");

    let mut cycle: u32 = 0;

    loop {
        cycle += 1;

        // Update test value (varies over time for verification)
        // Use simple modulo pattern instead of sin (no_std doesn't have f32::sin)
        let test_value = ((cycle % 100) as f32) - 50.0;
        registry
            .set_by_name("TEST_VALUE", ParamValue::Float(test_value))
            .ok();

        // Increment save counter
        if let Some(param) = registry.get_by_name("SAVE_COUNT") {
            if let ParamValue::Uint32(count) = param.value {
                registry
                    .set_by_name("SAVE_COUNT", ParamValue::Uint32(count + 1))
                    .ok();
            }
        }

        // Perform save with timing
        info!("─────────────────────────────────────────────────────");
        info!("Cycle {}: Saving parameters...", cycle);
        info!("  TEST_VALUE = {}", test_value);

        let save_start = Instant::now();
        match registry.save_to_flash() {
            Ok(_) => {
                let save_time = save_start.elapsed().as_millis();
                info!("  ✓ Saved in {} ms", save_time);

                // Display storage stats
                if let Some(stats) = registry.storage_stats() {
                    info!("  Active block: {}", stats.active_block);
                    info!("  Total saves: {}", stats.total_saves);
                    info!("  Erase counts: {:?}", stats.erase_counts);
                }
            }
            Err(e) => {
                error!("  ✗ Save failed: {:?}", e);
            }
        }

        info!("");
        info!("⚠ DISCONNECT POWER NOW to test recovery ⚠");
        info!("");

        // Wait before next save
        Timer::after(Duration::from_secs(SAVE_INTERVAL_SECS)).await;
    }
}
