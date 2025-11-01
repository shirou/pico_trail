use std::env;

fn main() {
    // Read WiFi configuration from environment variables (optional)
    // These are used as default values when parameter storage is empty

    // WiFi SSID (network name)
    if let Ok(ssid) = env::var("WIFI_SSID") {
        println!("cargo:rustc-env=WIFI_SSID={}", ssid);
        println!("cargo:warning=Using WIFI_SSID from environment: {}", ssid);
    } else {
        println!("cargo:rustc-env=WIFI_SSID=");
    }

    // WiFi password
    if let Ok(password) = env::var("WIFI_PASSWORD") {
        println!("cargo:rustc-env=WIFI_PASSWORD={}", password);
        println!("cargo:warning=Using WIFI_PASSWORD from environment (hidden)");
    } else {
        println!("cargo:rustc-env=WIFI_PASSWORD=");
    }

    // DHCP enabled (default: true)
    if let Ok(dhcp) = env::var("WIFI_DHCP") {
        println!("cargo:rustc-env=WIFI_DHCP={}", dhcp);
        println!("cargo:warning=Using WIFI_DHCP from environment: {}", dhcp);
    } else {
        println!("cargo:rustc-env=WIFI_DHCP=true");
    }

    // Static IP (used if DHCP=false)
    if let Ok(ip) = env::var("WIFI_IP") {
        println!("cargo:rustc-env=WIFI_IP={}", ip);
        println!("cargo:warning=Using WIFI_IP from environment: {}", ip);
    } else {
        println!("cargo:rustc-env=WIFI_IP=0.0.0.0");
    }

    // Network mask
    if let Ok(netmask) = env::var("WIFI_NETMASK") {
        println!("cargo:rustc-env=WIFI_NETMASK={}", netmask);
        println!(
            "cargo:warning=Using WIFI_NETMASK from environment: {}",
            netmask
        );
    } else {
        println!("cargo:rustc-env=WIFI_NETMASK=255.255.255.0");
    }

    // Gateway
    if let Ok(gateway) = env::var("WIFI_GATEWAY") {
        println!("cargo:rustc-env=WIFI_GATEWAY={}", gateway);
        println!(
            "cargo:warning=Using WIFI_GATEWAY from environment: {}",
            gateway
        );
    } else {
        println!("cargo:rustc-env=WIFI_GATEWAY=0.0.0.0");
    }

    // Rerun if environment variables change
    println!("cargo:rerun-if-env-changed=WIFI_SSID");
    println!("cargo:rerun-if-env-changed=WIFI_PASSWORD");
    println!("cargo:rerun-if-env-changed=WIFI_DHCP");
    println!("cargo:rerun-if-env-changed=WIFI_IP");
    println!("cargo:rerun-if-env-changed=WIFI_NETMASK");
    println!("cargo:rerun-if-env-changed=WIFI_GATEWAY");
}
