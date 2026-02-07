//! GCS (Ground Control Station) communication over MAVLink UDP.
//!
//! Provides a per-vehicle UDP link that sends telemetry to and receives
//! commands from Mission Planner or other MAVLink-compatible GCS software.

mod telemetry;

use std::io::{self, Cursor};
use std::net::{SocketAddr, UdpSocket};

use mavlink::common::MavMessage;
use mavlink::peek_reader::PeekReader;
use mavlink::MavHeader;

use crate::types::SensorData;
use telemetry::{build_heartbeat, build_sys_status, build_telemetry};

/// Rate intervals in microseconds for each telemetry message type.
const HEARTBEAT_INTERVAL_US: u64 = 1_000_000; // 1 Hz
const ATTITUDE_INTERVAL_US: u64 = 250_000; // 4 Hz
const POSITION_INTERVAL_US: u64 = 500_000; // 2 Hz
const SYS_STATUS_INTERVAL_US: u64 = 1_000_000; // 1 Hz

/// Per-vehicle MAVLink UDP connection to a ground control station.
pub struct GcsLink {
    socket: UdpSocket,
    system_id: u8,
    component_id: u8,
    sequence: u8,
    gcs_addr: Option<SocketAddr>,
    last_heartbeat_us: u64,
    last_attitude_us: u64,
    last_position_us: u64,
    last_sys_status_us: u64,
    recv_buf: Vec<u8>,
}

impl GcsLink {
    /// Create a new GCS link bound to `0.0.0.0:{port}` in non-blocking mode.
    pub fn new(system_id: u8, port: u16) -> io::Result<Self> {
        let socket = UdpSocket::bind(format!("0.0.0.0:{port}"))?;
        socket.set_nonblocking(true)?;

        Ok(Self {
            socket,
            system_id,
            component_id: 1, // MAV_COMP_ID_AUTOPILOT1
            sequence: 0,
            gcs_addr: None,
            last_heartbeat_us: 0,
            last_attitude_us: 0,
            last_position_us: 0,
            last_sys_status_us: 0,
            recv_buf: vec![0u8; 280],
        })
    }

    /// Try to receive and parse incoming MAVLink messages.
    ///
    /// On the first message received, the sender address is stored as the GCS
    /// endpoint for outgoing telemetry.
    pub fn poll_incoming(&mut self) -> Vec<(MavHeader, MavMessage)> {
        let mut messages = Vec::new();
        loop {
            match self.socket.recv_from(&mut self.recv_buf) {
                Ok((len, addr)) => {
                    if self.gcs_addr.is_none() {
                        self.gcs_addr = Some(addr);
                    }
                    if let Some(msg) = self.parse_datagram(&self.recv_buf[..len]) {
                        messages.push(msg);
                    }
                }
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
        messages
    }

    /// Send a MAVLink v2 message to the discovered GCS address.
    ///
    /// Does nothing if no GCS has been discovered yet.
    pub fn send_message(&mut self, msg: &MavMessage) -> io::Result<()> {
        let Some(addr) = self.gcs_addr else {
            return Ok(());
        };

        let header = MavHeader {
            system_id: self.system_id,
            component_id: self.component_id,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);

        let mut buf = Cursor::new(Vec::with_capacity(280));
        mavlink::write_v2_msg(&mut buf, header, msg)
            .map_err(|e| io::Error::other(format!("{e:?}")))?;

        self.socket.send_to(&buf.into_inner(), addr)?;
        Ok(())
    }

    /// Poll for incoming messages and send rate-limited telemetry.
    ///
    /// Call this once per simulation step.
    pub fn update(&mut self, sensors: &SensorData, sim_time_us: u64) {
        // Drain incoming messages (auto-discovers GCS address)
        let _incoming = self.poll_incoming();

        // HEARTBEAT at 1 Hz
        if sim_time_us - self.last_heartbeat_us >= HEARTBEAT_INTERVAL_US {
            self.last_heartbeat_us = sim_time_us;
            let _ = self.send_message(&build_heartbeat());
        }

        let telem = build_telemetry(sensors);

        // ATTITUDE at 4 Hz
        if sim_time_us - self.last_attitude_us >= ATTITUDE_INTERVAL_US {
            self.last_attitude_us = sim_time_us;
            if let Some(msg) = &telem.attitude {
                let _ = self.send_message(msg);
            }
        }

        // GPS_RAW_INT + GLOBAL_POSITION_INT at 2 Hz
        if sim_time_us - self.last_position_us >= POSITION_INTERVAL_US {
            self.last_position_us = sim_time_us;
            if let Some(msg) = &telem.gps_raw {
                let _ = self.send_message(msg);
            }
            if let Some(msg) = &telem.global_pos {
                let _ = self.send_message(msg);
            }
        }

        // SYS_STATUS at 1 Hz
        if sim_time_us - self.last_sys_status_us >= SYS_STATUS_INTERVAL_US {
            self.last_sys_status_us = sim_time_us;
            let _ = self.send_message(&build_sys_status(12600)); // ~12.6V default
        }
    }

    /// Whether a GCS endpoint has been discovered.
    pub fn is_connected(&self) -> bool {
        self.gcs_addr.is_some()
    }

    fn parse_datagram(&self, data: &[u8]) -> Option<(MavHeader, MavMessage)> {
        let cursor = Cursor::new(data);
        let mut reader = PeekReader::new(cursor);
        mavlink::read_v2_msg::<MavMessage, _>(&mut reader).ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gcs_link_creation() {
        // Bind to port 0 to let OS assign a free port
        let link = GcsLink::new(1, 0);
        assert!(link.is_ok());
        let link = link.unwrap();
        assert!(!link.is_connected());
        assert_eq!(link.system_id, 1);
    }

    #[test]
    fn test_send_without_gcs_is_noop() {
        let mut link = GcsLink::new(1, 0).unwrap();
        let msg = build_heartbeat();
        // Should succeed (noop) even without GCS
        assert!(link.send_message(&msg).is_ok());
    }

    #[test]
    fn test_poll_incoming_empty() {
        let mut link = GcsLink::new(1, 0).unwrap();
        let messages = link.poll_incoming();
        assert!(messages.is_empty());
    }

    #[test]
    fn test_gcs_link_loopback() {
        // Create a GcsLink and a "GCS" socket that will talk to it
        let link = GcsLink::new(1, 0).unwrap();
        let link_addr = link.socket.local_addr().unwrap();

        let gcs = UdpSocket::bind("127.0.0.1:0").unwrap();
        gcs.set_nonblocking(true).unwrap();

        // GCS sends a heartbeat to the link
        let header = MavHeader {
            system_id: 255,
            component_id: 0,
            sequence: 0,
        };
        let msg = build_heartbeat();
        let mut buf = Cursor::new(Vec::with_capacity(280));
        mavlink::write_v2_msg(&mut buf, header, &msg).unwrap();
        gcs.send_to(&buf.into_inner(), link_addr).unwrap();

        // Give the OS a moment to deliver the datagram
        std::thread::sleep(std::time::Duration::from_millis(10));

        let mut link = link;
        let received = link.poll_incoming();
        assert_eq!(received.len(), 1);
        assert!(link.is_connected());

        // Now the link should be able to send back
        let result = link.send_message(&build_heartbeat());
        assert!(result.is_ok());
    }
}
