use std::env;

fn main() {
    // Read WiFi configuration from environment variables (optional)
    // These are used as default values when parameter storage is empty

    // WiFi SSID (network name)
    if let Ok(ssid) = env::var("NET_SSID") {
        println!("cargo:rustc-env=NET_SSID={}", ssid);
        println!("cargo:warning=Using NET_SSID from environment: {}", ssid);
    } else {
        println!("cargo:rustc-env=NET_SSID=");
    }

    // WiFi password
    if let Ok(password) = env::var("NET_PASS") {
        println!("cargo:rustc-env=NET_PASS={}", password);
        println!("cargo:warning=Using NET_PASS from environment (hidden)");
    } else {
        println!("cargo:rustc-env=NET_PASS=");
    }

    // DHCP enabled (default: true)
    if let Ok(dhcp) = env::var("NET_DHCP") {
        println!("cargo:rustc-env=NET_DHCP={}", dhcp);
        println!("cargo:warning=Using NET_DHCP from environment: {}", dhcp);
    } else {
        println!("cargo:rustc-env=NET_DHCP=true");
    }

    // Static IP (used if DHCP=false)
    if let Ok(ip) = env::var("NET_IP") {
        println!("cargo:rustc-env=NET_IP={}", ip);
        println!("cargo:warning=Using NET_IP from environment: {}", ip);
    } else {
        println!("cargo:rustc-env=NET_IP=0.0.0.0");
    }

    // Network mask
    if let Ok(netmask) = env::var("NET_NETMASK") {
        println!("cargo:rustc-env=NET_NETMASK={}", netmask);
        println!(
            "cargo:warning=Using NET_NETMASK from environment: {}",
            netmask
        );
    } else {
        println!("cargo:rustc-env=NET_NETMASK=255.255.255.0");
    }

    // Gateway
    if let Ok(gateway) = env::var("NET_GATEWAY") {
        println!("cargo:rustc-env=NET_GATEWAY={}", gateway);
        println!(
            "cargo:warning=Using NET_GATEWAY from environment: {}",
            gateway
        );
    } else {
        println!("cargo:rustc-env=NET_GATEWAY=0.0.0.0");
    }

    // Rerun if environment variables change
    println!("cargo:rerun-if-env-changed=NET_SSID");
    println!("cargo:rerun-if-env-changed=NET_PASS");
    println!("cargo:rerun-if-env-changed=NET_DHCP");
    println!("cargo:rerun-if-env-changed=NET_IP");
    println!("cargo:rerun-if-env-changed=NET_NETMASK");
    println!("cargo:rerun-if-env-changed=NET_GATEWAY");
}
