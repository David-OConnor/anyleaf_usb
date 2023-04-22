//! This module contains code shared by firmware and PC software for
//! communication between devices and PC, using a simple protocol.

#![no_std]

// use serialport::{self, SerialPort, SerialPortType};


pub const MSG_START: u8 = 69;

pub const CRC_POLY: u8 = 0xab;
pub const CRC_LUT: [u8; 256] = crc_init(CRC_POLY);

pub const READINGS_SIZE: usize = 4 * 4 + 1;

// todo: enum etc for these?

pub const DEVICE_CODE_SENSOR_INTERFACE: u8 = 1;
pub const DEVICE_CODE_GNSS: u8 = 2;
pub const DEVICE_CODE_RECEIVER: u8 = 3;

pub const CONFIG_SIZE_COMMON: usize = 4;

pub const CONFIG_SIZE_GNSS: usize = CONFIG_SIZE_COMMON + 3;
pub const CONFIG_SIZE_RECEIVER: usize = CONFIG_SIZE_COMMON + 1;

pub const SENSOR_INTERFACE_READINGS_SIZE: usize = 4 * 4 + 1;


pub trait MessageType {
    fn val(&self) -> u8;
    fn payload_size(&self) -> usize;
}

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

/// CRC8 using a specific poly, includes all bytes buffer[1] to end of payload, except for
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

