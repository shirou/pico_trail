#![cfg_attr(feature = "pico2_w", no_std)]
#![cfg_attr(feature = "pico2_w", no_main)]
#![cfg(feature = "pico2_w")] // Only compile for embedded targets

use pico_trail as _; // memory layout + panic handler

// See https://crates.io/crates/defmt-test/0.3.0 for more documentation (e.g. about the 'state'
// feature)
#[defmt_test::tests]
mod tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
