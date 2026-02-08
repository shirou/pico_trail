//! Minimal UDP test — send servo packet to Gazebo, receive FDM response.
//! Tests both std (blocking) and tokio (async) sockets.
//!
//! Usage: cargo run -p pico_trail_sitl --bin test_udp

use std::time::Duration;

const MAGIC_16: u16 = 0x481A;
const FRAME_RATE: u16 = 400;

fn build_servo_packet(frame_count: u32) -> [u8; 40] {
    let mut buf = [0u8; 40];
    buf[0..2].copy_from_slice(&MAGIC_16.to_le_bytes());
    buf[2..4].copy_from_slice(&FRAME_RATE.to_le_bytes());
    buf[4..8].copy_from_slice(&frame_count.to_le_bytes());
    // PWM channels: all 1500 (neutral)
    for i in 0..16 {
        let offset = 8 + i * 2;
        buf[offset..offset + 2].copy_from_slice(&1500u16.to_le_bytes());
    }
    buf
}

fn test_std_socket(port: u16) {
    println!("--- Test 1: std::net::UdpSocket (blocking) ---");
    let sock = std::net::UdpSocket::bind("0.0.0.0:0").unwrap();
    sock.set_read_timeout(Some(Duration::from_millis(500)))
        .unwrap();
    let target = format!("127.0.0.1:{port}");

    for i in 0..5 {
        let pkt = build_servo_packet(i);
        sock.send_to(&pkt, &target).unwrap();

        match sock.recv_from(&mut [0u8; 2048]) {
            Ok((len, addr)) => {
                println!("  frame {i}: RX {len} bytes from {addr}");
                if len > 0 {
                    println!("  SUCCESS with std socket!");
                    return;
                }
            }
            Err(e) => println!("  frame {i}: {e}"),
        }
    }
    println!("  No data received with std socket.\n");
}

#[tokio::main(flavor = "current_thread")]
async fn main() {
    let port: u16 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(9002);

    println!("=== UDP Socket Test (port {port}) ===\n");

    // Test 1: std blocking socket
    test_std_socket(port);

    // Test 2: tokio async socket (unconnected, recv_from)
    println!("--- Test 2: tokio UdpSocket (async, recv_from) ---");
    {
        let sock = tokio::net::UdpSocket::bind("0.0.0.0:0").await.unwrap();
        let target: std::net::SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();

        for i in 0u32..5 {
            let pkt = build_servo_packet(100 + i);
            sock.send_to(&pkt, target).await.unwrap();

            match tokio::time::timeout(Duration::from_millis(500), sock.recv_from(&mut [0u8; 2048]))
                .await
            {
                Ok(Ok((len, addr))) => {
                    println!("  frame {i}: RX {len} bytes from {addr}");
                    if len > 0 {
                        println!("  SUCCESS with tokio recv_from!");
                        return;
                    }
                }
                Ok(Err(e)) => println!("  frame {i}: error {e}"),
                Err(_) => println!("  frame {i}: timeout"),
            }
        }
        println!("  No data received with tokio recv_from.\n");
    }

    // Test 3: tokio async socket (connected, recv)
    println!("--- Test 3: tokio UdpSocket (async, connected + recv) ---");
    {
        let sock = tokio::net::UdpSocket::bind("0.0.0.0:0").await.unwrap();
        let target: std::net::SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
        sock.connect(target).await.unwrap();

        for i in 0u32..5 {
            let pkt = build_servo_packet(200 + i);
            sock.send(&pkt).await.unwrap();

            match tokio::time::timeout(Duration::from_millis(500), sock.recv(&mut [0u8; 2048]))
                .await
            {
                Ok(Ok(len)) => {
                    println!("  frame {i}: RX {len} bytes");
                    if len > 0 {
                        println!("  SUCCESS with tokio connected recv!");
                        return;
                    }
                }
                Ok(Err(e)) => println!("  frame {i}: error {e}"),
                Err(_) => println!("  frame {i}: timeout"),
            }
        }
        println!("  No data received with tokio connected recv.\n");
    }

    println!("\nAll tests complete.");
}
