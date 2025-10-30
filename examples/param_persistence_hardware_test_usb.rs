//! Parameter persistence hardware validation test
//!
//! This example performs comprehensive hardware testing of the parameter
//! persistence system, including:
//! - Load time measurement (target < 100ms)
//! - Save latency measurement
//! - Endurance testing (100-1000 save cycles)
//! - Block rotation verification
//! - Storage statistics validation
//!
//! # Hardware
//!
//! This example requires Raspberry Pi Pico 2 W (RP2350) with Flash storage.
//!
//! # Usage
//!
//! ```bash
//! ./scripts/build-rp2350.sh --release param_persistence_hardware_test
//! probe-rs run --chip RP2350 target/thumbv8m.main-none-eabihf/release/examples/param_persistence_hardware_test
//! ```
//!
//! # Test Modes
//!
//! Set TEST_MODE to select test:
//! - 0: Quick test (10 saves, verify basics)
//! - 1: Performance test (measure load/save times)
//! - 2: Endurance test (100 saves, verify wear leveling)
//! - 3: Stress test (1000 saves, full endurance validation)

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp as hal;
use embassy_time::{Duration, Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

use pico_trail::core::parameters::{ParamMetadata, ParamValue, ParameterRegistry};
use pico_trail::platform::rp2350::Rp2350Flash;

/// Test mode selection (0=quick, 1=performance, 2=endurance, 3=stress)
const TEST_MODE: u8 = 0;

/// Number of test cycles per mode
const TEST_CYCLES: [u32; 4] = [
    10,   // Quick test
    50,   // Performance test
    100,  // Endurance test
    1000, // Stress test
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Initialize platform
    let _p = hal::init(Default::default());

    info!("╔════════════════════════════════════════════════════════╗");
    info!("║  Parameter Persistence Hardware Validation Test        ║");
    info!("╚════════════════════════════════════════════════════════╝");
    info!("");

    let test_name = match TEST_MODE {
        0 => "Quick Test (10 cycles)",
        1 => "Performance Test (50 cycles)",
        2 => "Endurance Test (100 cycles)",
        3 => "Stress Test (1000 cycles)",
        _ => "Unknown Test",
    };
    info!("Test Mode: {}", test_name);
    info!("");

    // Phase 1: Initialize Flash and Registry
    info!("═══ Phase 1: Initialization ═══");
    let flash = Rp2350Flash::new();
    let mut registry = ParameterRegistry::with_flash(flash);

    // Register test parameters
    info!("Registering test parameters...");
    registry
        .register(ParamMetadata::new_float("RATE_ROLL_P", 0.15, 0.0, 1.0))
        .unwrap();
    registry
        .register(ParamMetadata::new_float("RATE_ROLL_I", 0.1, 0.0, 1.0))
        .unwrap();
    registry
        .register(ParamMetadata::new_float("RATE_ROLL_D", 0.004, 0.0, 0.1))
        .unwrap();
    registry
        .register(ParamMetadata::new_float("RATE_PITCH_P", 0.15, 0.0, 1.0))
        .unwrap();
    registry
        .register(ParamMetadata::new_float("RATE_PITCH_I", 0.1, 0.0, 1.0))
        .unwrap();
    registry
        .register(ParamMetadata::new_float("RATE_PITCH_D", 0.004, 0.0, 0.1))
        .unwrap();
    registry
        .register(ParamMetadata::new_uint32("SYSID_THISMAV", 1, 1, 255))
        .unwrap();
    registry
        .register(ParamMetadata::new_uint32("SR_EXTRA1", 10, 0, 50))
        .unwrap();
    registry
        .register(ParamMetadata::new_uint32("SR_POSITION", 5, 0, 50))
        .unwrap();
    registry
        .register(ParamMetadata::new_uint32("SR_RC_CHAN", 5, 0, 50))
        .unwrap();
    info!("✓ Registered {} parameters", registry.count());
    info!("");

    // Phase 2: Load Performance Test
    info!("═══ Phase 2: Load Performance ═══");
    let load_start = Instant::now();
    match registry.load_from_flash() {
        Ok(_) => {
            let load_time_ms = load_start.elapsed().as_millis();
            info!("✓ Parameters loaded in {} ms", load_time_ms);

            if load_time_ms < 100 {
                info!("  ✓ PASS: Load time < 100ms target");
            } else {
                warn!("  ✗ FAIL: Load time exceeds 100ms target");
            }
        }
        Err(e) => {
            warn!("Load failed (using defaults): {:?}", e);
        }
    }
    info!("");

    // Display loaded values
    info!("Current parameter values:");
    for i in 0..registry.count() {
        if let Some(param) = registry.get_by_index(i) {
            match param.value {
                ParamValue::Float(f) => info!("  {} = {}", param.name, f),
                ParamValue::Uint32(u) => info!("  {} = {}", param.name, u),
            }
        }
    }
    info!("");

    // Phase 3: Save Performance Test
    info!("═══ Phase 3: Save Performance ═══");

    // Modify a parameter
    registry
        .set_by_name("RATE_ROLL_P", ParamValue::Float(0.20))
        .unwrap();
    info!("Modified RATE_ROLL_P to 0.20");

    // Measure save time
    let save_start = Instant::now();
    match registry.save_to_flash() {
        Ok(_) => {
            let save_time_ms = save_start.elapsed().as_millis();
            info!("✓ Parameters saved in {} ms", save_time_ms);

            // Check if save is reasonably fast (should be < 200ms for Flash erase + write)
            if save_time_ms < 200 {
                info!("  ✓ PASS: Save time acceptable");
            } else {
                warn!("  ⚠ WARNING: Save time higher than expected");
            }
        }
        Err(e) => {
            error!("✗ Save failed: {:?}", e);
        }
    }
    info!("");

    // Display storage stats
    if let Some(stats) = registry.storage_stats() {
        info!("Storage Statistics:");
        info!("  Total saves: {}", stats.total_saves);
        info!("  Active block: {}", stats.active_block);
        info!("  Block erase counts:");
        for (i, count) in stats.erase_counts.iter().enumerate() {
            info!("    Block {}: {} erases", i, count);
        }
        info!("");
    }

    // Phase 4: Endurance Test
    info!("═══ Phase 4: Endurance Test ═══");
    let test_cycles = TEST_CYCLES[TEST_MODE as usize];
    info!("Running {} save cycles...", test_cycles);
    info!("");

    let mut save_times_sum: u64 = 0;
    let mut save_times_min: u64 = u64::MAX;
    let mut save_times_max: u64 = 0;
    let mut failed_saves = 0;

    let endurance_start = Instant::now();

    for cycle in 0..test_cycles {
        // Modify parameter value (cycle through range)
        let new_value = 0.15 + ((cycle % 20) as f32 * 0.01);
        registry
            .set_by_name("RATE_ROLL_P", ParamValue::Float(new_value))
            .unwrap();

        // Save and measure time
        let save_start = Instant::now();
        match registry.save_to_flash() {
            Ok(_) => {
                let save_time = save_start.elapsed().as_millis();
                save_times_sum += save_time;
                save_times_min = save_times_min.min(save_time);
                save_times_max = save_times_max.max(save_time);
            }
            Err(e) => {
                error!("Save failed at cycle {}: {:?}", cycle, e);
                failed_saves += 1;
            }
        }

        // Progress indicator (every 10% or every 10 cycles for quick test)
        let report_interval = if test_cycles >= 100 {
            test_cycles / 10
        } else {
            10
        };

        if (cycle + 1) % report_interval == 0 {
            let progress = ((cycle + 1) * 100) / test_cycles;
            info!("  Progress: {}% ({}/{})", progress, cycle + 1, test_cycles);

            // Display current stats
            if let Some(stats) = registry.storage_stats() {
                info!(
                    "    Active block: {}, Total saves: {}",
                    stats.active_block, stats.total_saves
                );
            }
        }

        // Small delay between cycles (simulate real usage)
        Timer::after(Duration::from_millis(10)).await;
    }

    let total_duration = endurance_start.elapsed();
    info!("");
    info!(
        "✓ Endurance test completed in {} ms",
        total_duration.as_millis()
    );
    info!("");

    // Phase 5: Results Analysis
    info!("═══ Phase 5: Results Summary ═══");
    info!("");

    // Save performance stats
    if test_cycles > 0 && failed_saves < test_cycles {
        let avg_save_time = save_times_sum / (test_cycles as u64 - failed_saves as u64);
        info!("Save Performance:");
        info!("  Average: {} ms", avg_save_time);
        info!("  Minimum: {} ms", save_times_min);
        info!("  Maximum: {} ms", save_times_max);
        info!("  Failed saves: {}/{}", failed_saves, test_cycles);
        info!("");
    }

    // Storage statistics
    if let Some(stats) = registry.storage_stats() {
        info!("Final Storage Statistics:");
        info!("  Total saves: {}", stats.total_saves);
        info!("  Active block: {}", stats.active_block);
        info!("");
        info!("  Block erase counts:");
        for (i, count) in stats.erase_counts.iter().enumerate() {
            info!("    Block {}: {} erases", i, count);
        }
        info!("");

        // Verify wear leveling
        let max_erase = stats.erase_counts.iter().max().unwrap_or(&0);
        let min_erase = stats.erase_counts.iter().min().unwrap_or(&0);
        let erase_diff = max_erase - min_erase;

        info!("Wear Leveling Analysis:");
        info!("  Max erase count: {}", max_erase);
        info!("  Min erase count: {}", min_erase);
        info!("  Difference: {}", erase_diff);

        // For good wear leveling, difference should be small
        // With round-robin, difference should be ≤ 1
        if erase_diff <= 1 {
            info!("  ✓ PASS: Wear leveling working correctly");
        } else {
            warn!("  ⚠ WARNING: Wear leveling may not be optimal");
        }
        info!("");

        // Verify all blocks are used
        let blocks_used = stats.erase_counts.iter().filter(|&&c| c > 0).count();
        info!("Block Usage:");
        info!("  Blocks used: {}/4", blocks_used);
        if blocks_used == 4 || (test_cycles < 4 && blocks_used >= test_cycles as usize) {
            info!("  ✓ PASS: Block rotation working");
        } else {
            warn!("  ⚠ WARNING: Not all blocks used");
        }
        info!("");

        // Endurance projection
        let erase_limit = 10_000; // Conservative estimate for Flash endurance
        let remaining_cycles = if *max_erase > 0 {
            (erase_limit - max_erase) * 4 // 4 blocks for rotation
        } else {
            erase_limit * 4
        };

        info!("Endurance Projection:");
        info!("  Erase limit per block: {}", erase_limit);
        info!("  Remaining save cycles: ~{}", remaining_cycles);
        info!(
            "  Estimated lifespan at 10 saves/day: ~{} years",
            remaining_cycles / (10 * 365)
        );
        info!("");
    }

    // Phase 6: Verification Load Test
    info!("═══ Phase 6: Verification Load ═══");
    info!("Performing verification load...");

    let verify_start = Instant::now();
    match registry.load_from_flash() {
        Ok(_) => {
            let load_time = verify_start.elapsed().as_millis();
            info!("✓ Verification load successful in {} ms", load_time);

            // Verify the last saved value
            if let Some(param) = registry.get_by_name("RATE_ROLL_P") {
                match param.value {
                    ParamValue::Float(f) => {
                        info!("  RATE_ROLL_P = {}", f);
                        let expected = 0.15 + (((test_cycles - 1) % 20) as f32 * 0.01);
                        let diff = (f - expected).abs();
                        if diff < 0.001 {
                            info!("  ✓ PASS: Value matches expected ({})", expected);
                        } else {
                            warn!("  ✗ FAIL: Value mismatch (expected {})", expected);
                        }
                    }
                    _ => warn!("  ✗ FAIL: Wrong parameter type"),
                }
            }
        }
        Err(e) => {
            error!("✗ Verification load failed: {:?}", e);
        }
    }
    info!("");

    // Final summary
    info!("╔════════════════════════════════════════════════════════╗");
    info!("║  Test Complete                                         ║");
    info!("╚════════════════════════════════════════════════════════╝");
    info!("");
    info!("Test Summary:");
    info!("  Test mode: {}", test_name);
    info!("  Total cycles: {}", test_cycles);
    info!("  Failed saves: {}", failed_saves);
    info!("");

    if failed_saves == 0 {
        info!("✓ ALL TESTS PASSED");
    } else {
        warn!("⚠ SOME TESTS FAILED");
    }
    info!("");
    info!("Hardware validation test finished.");
    info!("");
    info!("To run different test modes, change TEST_MODE constant:");
    info!("  0 = Quick test (10 cycles)");
    info!("  1 = Performance test (50 cycles)");
    info!("  2 = Endurance test (100 cycles)");
    info!("  3 = Stress test (1000 cycles)");
    info!("");

    // Keep running to allow log inspection
    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
