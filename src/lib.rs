//! This module contains code shared by firmware and PC software for
//! communication between devices and PC, using a simple protocol.

#![no_std]

// use serialport::{self, SerialPort, SerialPortType};

use num_enum::TryFromPrimitive; // Enum from integer

pub const MSG_START: u8 = 69;

// Includes start byte, message type, and device-specific code.
pub const PAYLOAD_START_I: usize = 3;

pub const CRC_LEN: usize = 1;

pub const CRC_POLY: u8 = 0xab;
pub const CRC_LUT: [u8; 256] = crc_init(CRC_POLY);

// todo: No.
pub const SENSOR_IFACE_READINGS_SIZE: usize = 4 * 4 + 1;

// todo: enum etc for these?

pub const DEVICE_CODE_PC: u8 = 0;
pub const DEVICE_CODE_SENSOR_INTERFACE: u8 = 1;
pub const DEVICE_CODE_GNSS: u8 = 2;
pub const DEVICE_CODE_RECEIVER: u8 = 3;
pub const DEVICE_CODE_CORVUS: u8 = 4;

pub const CONFIG_SIZE_COMMON: usize = 4;

pub const CONFIG_SIZE_GNSS: usize = CONFIG_SIZE_COMMON + 8;
pub const CONFIG_SIZE_RECEIVER: usize = CONFIG_SIZE_COMMON + 2;

pub const SENSOR_INTERFACE_READINGS_SIZE: usize = 4 * 4 + 1;

pub const PAYLOAD_SIZE_CONFIG_GNSS: usize = CONFIG_SIZE_GNSS + PAYLOAD_START_I + CRC_LEN;
pub const PAYLOAD_SIZE_CONFIG_RX: usize = CONFIG_SIZE_RECEIVER + PAYLOAD_START_I + CRC_LEN;

pub trait MessageType {
    fn val(&self) -> u8;
    fn payload_size(&self) -> usize;
}

#[derive(Copy, Clone, Eq, PartialEq, TryFromPrimitive)]
#[repr(u8)]
pub enum MsgType {
    ReqConfig = 0,
    ConfigGnss = 1,
    ConfigRx = 2,
    SaveConfigGnss = 3,
    SaveConfigRx = 4,
}

impl MessageType for MsgType {
    fn val(&self) -> u8 {
        *self as u8
    }

    fn payload_size(&self) -> usize {
        match self {
            Self::ReqConfig => 0,
            Self::ConfigGnss => CONFIG_SIZE_GNSS,
            Self::ConfigRx => CONFIG_SIZE_RECEIVER,
            Self::SaveConfigGnss => CONFIG_SIZE_GNSS,
            Self::SaveConfigRx => CONFIG_SIZE_RECEIVER,
        }
    }
}

// impl MsgType {
//     pub fn payload_size(&self) -> usize {
//         match self {
//             Self::ReqConfig => 0,
//             Self::ConfigGnss => CONFIG_SIZE_GNSS,
//             Self::ConfigRx => CONFIG_SIZE_RECEIVER,
//             Self::SaveConfigGnss => CONFIG_SIZE_GNSS,
//             Self::SaveConfigRx => CONFIG_SIZE_RECEIVER,
//         }
//     }
// }

/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
const fn crc_init(poly: u8) -> [u8; 256] {
    let mut lut = [0; 256];

    let mut i = 0;
    while i < 256 {
        // Can't use for loops in const fns
        let mut crc = i as u8;

        let mut j = 0;
        while j < 8 {
            crc = (crc << 1) ^ (if (crc & 0x80) > 0 { poly } else { 0 });
            j += 1;
        }
        lut[i] = crc;

        i += 1;
    }

    lut
}

/// CRC8 using a specific poly, includes all bytes, except for
/// the CRC byte itself.
/// https://github.com/chris1seto/OzarkRiver/blob/4channel/FlightComputerFirmware/Src/Crsf.c
pub fn calc_crc(lut: &[u8; 256], data: &[u8], mut size: u8) -> u8 {
    let mut crc = 0;
    let mut i = 0;

    while size > 0 {
        size -= 1;
        crc = lut[(crc ^ data[i]) as usize];
        i += 1;
    }
    crc
}

/// Convert bytes to a float
pub fn bytes_to_float(bytes: &[u8]) -> f32 {
    let bytes: [u8; 4] = bytes.try_into().unwrap();
    f32::from_bits(u32::from_be_bytes(bytes))
}

/// Returns true if the CRC passed; false if failed.
pub fn check_crc(buf: &[u8], payload_size: usize) -> bool {
    let crc_i = payload_size + PAYLOAD_START_I;

    let crc_received = buf[crc_i];
    let expected_crc_rx = calc_crc(&CRC_LUT, &buf[..crc_i], crc_i as u8);

    crc_received == expected_crc_rx
}
