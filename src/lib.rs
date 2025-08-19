//! A platform agnostic driver to interface with the
//! [LIS3MDL](https://www.st.com/en/mems-and-sensors/lis3mdl.html) (3-axis magnetic sensor).
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/1.0

#![no_std]

use embedded_hal::i2c::I2c;
use bitflags::bitflags;

/// The full scale for measurement from 4 Gauss to 16 Gauss; Sensitivity of the sensor
#[derive(Debug)]
pub enum FullScale {
    Fs4g,
    Fs8g,
    Fs12g,
    Fs16g
}

/// The mode to operate in, impacts noise, power consumption, and speed
#[derive(Debug)]
pub enum OperatingMode {
    LowPower,
    MediumPerformance,
    HighPerformance,
    UltraHighPerformance
}

/// State of the LIS3MDL
#[derive(Debug)]
pub enum MeasurementMode {
    Idle,
    SingleMeasurement,
    Continuous
}

/// Possible data rates at which the xyz data can be provided
#[allow(non_camel_case_types)]
#[derive(Debug)]
pub enum DataRate {
    ODR_0_625Hz,
    ODR_1_25Hz,
    ODR_2_5Hz,
    ODR_5Hz,
    ODR_10Hz,
    ODR_20Hz,
    ODR_40Hz,
    ODR_80Hz,
    /// Fastest obtainable data rate for the given operating mode
    ODR_Fast
}

/// Driver Errors
#[derive(Debug)]
pub enum Error<E> {
    /// Any issue with the I<sup>2</sup>C connection
    CommunicationError(E),
    /// An invalid value was found, should not happen
    InvalidValue,
    /// The wrong device was present when queried, checked once on driver setup
    IncorrectDeviceIdFound,
}

/// XYZ triple for raw values (int16)
#[derive(Debug)]
pub struct I16xyz {
    /// X component
    pub x: i16,
    /// Y component
    pub y: i16,
    /// Z component
    pub z: i16,
}

/// XYZ triple 32-bit float
#[derive(Debug)]
pub struct I32xyz {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

pub enum Address {
    Addr1E = 0x1E,
    Addr1C = 0x1C,
}

const LIS3MDL_DEVICE_ID: u8 = 0x3D;

/// LIS3MDL driver
pub struct Lis3mdl {
    address: u8,
}

impl Lis3mdl {
    /// Create a new driver instance with the specified I2C address and configures default settings:
    ///
    /// * Full Scale Range: 12G
    /// * Measurement Mode: Continuous Measurement
    /// * Operating Mode: Ultra High Performance
    /// * Enable the temperature sensor
    /// * Set data rate to fast
    /// * Enables Block Data Update
    ///
    /// These defaults may be changed after initialization with `set_full_scale`,
    /// `set_measurement_mode`, `set_operating_mode` `set_temperature_sensor`,
    /// `set_data_rate`, and `set_block_data_update`, respectively.
    pub fn new<I2C>(i2c: &mut I2C, addr: Address) -> Result<Self, Error<I2C::Error>> 
    where 
        I2C: I2c
    {
        let mut lis3mdl = Lis3mdl {
            address: addr as u8,
        };

        if lis3mdl.who_am_i(i2c)? != LIS3MDL_DEVICE_ID {
            return Err(Error::IncorrectDeviceIdFound)
        }

        lis3mdl.set_full_scale(i2c, FullScale::Fs12g)?;
        lis3mdl.set_operating_mode(i2c, OperatingMode::UltraHighPerformance)?;
        lis3mdl.set_measurement_mode(i2c, MeasurementMode::Continuous)?;
        lis3mdl.set_temperature_sensor_enable(i2c, true)?;
        lis3mdl.set_data_rate(i2c, DataRate::ODR_Fast)?;
        lis3mdl.set_block_data_update(i2c, true)?;

        Ok(lis3mdl)
    }

    /// Reads the WHO_AM_I register; should return `0x3D`
    pub fn who_am_i<I2C>(&mut self, i2c: &mut I2C) -> Result<u8, Error<I2C::Error>> 
    where 
        I2C: I2c
    {
        self.read_register(i2c, Register::WHO_AM_I)
    }

    /// Reads the XYZ components values of the magnetic field and returns the raw signed 16-bit
    /// integer value of each axis. This will return whatever is present in the data registers for
    /// each axis, so it is recommend to ensure that you are in the proper measurement mode and
    /// either synchronizing the read with interrupt/`xyz_data_available` or using
    /// `set_block_data_update`.
    ///
    /// To obtain the value in Gauss or milliGauss either use `get_mag_axes_mgauss` or divide by
    /// column 2 in the table below (obtained from Table 2 in AN4602 Rev 1):
    ///
    /// | Full-scale (G) | Gain@16-bit (LSB/Gauss) |
    /// |----------------|-------------------------|
    /// |        4       |           6842          |
    /// |        8       |           3421          |
    /// |        12      |           2281          |
    /// |        16      |           1711          |
    pub fn get_raw_mag_axes<I2C>(&mut self, i2c: &mut I2C) -> Result<I16xyz, Error<I2C::Error>> 
    where 
        I2C: I2c
    {
        let x = self.read_x_raw(i2c)?;
        let y = self.read_y_raw(i2c)?;
        let z = self.read_z_raw(i2c)?;

        Ok(I16xyz {
            x,
            y,
            z
        })
    }

    /// True if the XYZ data is available to be read
    pub fn xyz_data_available<I2C>(&mut self, i2c: &mut I2C) -> Result<bool, Error<I2C::Error>> 
    where 
        I2C: I2c
    {
        Ok(self.read_device_status(i2c)?.contains(StatusRegisterBits::ZYXDA))
    }

    /// Provide the magnetic field strength in each axis in milliGauss. Uses `get_raw_mag_axes` to
    /// obtain the value.
    pub fn get_mag_axes_mgauss<I2C>(&mut self, i2c: &mut I2C) -> Result<I32xyz, Error<I2C::Error>> 
    where 
        I2C: I2c
    {
        let mag_data = self.get_raw_mag_axes(i2c)?;

        let fullscale = FullScaleBits::from_bits(self.read_register(i2c, Register::CTRL_REG2)?).unwrap();

        // Gain values from Table 2 in AN4602 Rev 1
        let sensitivity: f64 = match fullscale.bits() {
            x if x == FullScaleBits::FS4G.bits() => 1000_f64/6842_f64,
            x if x == FullScaleBits::FS8G.bits() => 1000_f64/3421_f64,
            x if x == FullScaleBits::FS12G.bits() => 1000_f64/2281_f64,
            x if x == FullScaleBits::FS16G.bits() => 1000_f64/1711_f64,
            _ => return Err(Error::InvalidValue),
        };

        Ok(I32xyz {
            x: (mag_data.x as f64 * sensitivity) as i32,
            y: (mag_data.y as f64 * sensitivity) as i32,
            z: (mag_data.z as f64 * sensitivity) as i32
        })
    }

    /// Set the Full Scale from between 4 Gauss and 16 Gauss to adjust the input dynamic range,
    /// based on the magnetic field to be measured. This will affect the output of
    /// `get_raw_mag_axes` so use `get_mag_axes_mgauss` unless you intend to adjust the values
    /// yourself.
    pub fn set_full_scale<I2C>(&mut self, i2c: &mut I2C, scale: FullScale) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        // Mask for just the full scale bits.
        let fs_mask = !(ControlRegister2Bits::FS1 | ControlRegister2Bits::FS0);

        let fs = match scale {
            FullScale::Fs4g => FullScaleBits::FS4G,
            FullScale::Fs8g => FullScaleBits::FS8G,
            FullScale::Fs12g => FullScaleBits::FS12G,
            FullScale::Fs16g => FullScaleBits::FS16G,
        };

        // Zero out the full scale bits, we will replace them with the bitwise OR for the new setting
        let existing_settings = self.read_control_register_2(i2c)? & fs_mask;

        let reg2bits = ControlRegister2Bits::from_bits_truncate(fs.bits());

        // Update the full scale setting, preserving the other values
        self.set_control_register_2(i2c, reg2bits | existing_settings)
    }

    /// Adjust the operating mode. This will have an impact on current consumption, the max data
    /// rate, and the output noise with Ultra High Performance (UHP) being the slowest and highest
    /// current consumption with the lowest noise, and Low Power (LP) being the highest level of
    /// noise, but offering up to 1000 Hz data rate and lowest current consumption. See AN4602 for
    /// more details.
    pub fn set_operating_mode<I2C>(&mut self, i2c: &mut I2C, mode: OperatingMode) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        // Masks for just the operating mode bits for X, Y, and Z axes
        let reg1_mask = !(ControlRegister1Bits::OM1 | ControlRegister1Bits::OM0);
        let reg4_mask = !(ControlRegister4Bits::OMZ1 | ControlRegister4Bits::OMZ0);

        let om = match mode {
            OperatingMode::LowPower => (ControlRegister1Bits::empty(), ControlRegister4Bits::empty()),
            OperatingMode::MediumPerformance => (ControlRegister1Bits::OM0, ControlRegister4Bits::OMZ0),
            OperatingMode::HighPerformance => (ControlRegister1Bits::OM1, ControlRegister4Bits::OMZ1),
            OperatingMode::UltraHighPerformance => (ControlRegister1Bits::OM1
                                                        | ControlRegister1Bits::OM0,
                                                    ControlRegister4Bits::OMZ1
                                                        | ControlRegister4Bits::OMZ0),
        };

        // zero out the entries for OM1/OM0 and OMZ1/OMZ0
        let existing_reg_1_settings = self.read_control_register_1(i2c)? & reg1_mask;
        let existing_reg_4_settings = self.read_control_register_4(i2c)? & reg4_mask;

        // Update the operating mode settings, preserving the other values
        self.set_control_register_1(i2c, existing_reg_1_settings | om.0)?;
        self.set_control_register_4(i2c, existing_reg_4_settings | om.1)?;
        Ok(())
    }

    /// Select between 3 measurement modes: Idle, Single Measurement, and Continuous. Configure to
    /// Idle if not being used, Single Measurement if only one measurement is desired, and Continuous
    /// if a constant stream of data is needed.
    pub fn set_measurement_mode<I2C>(&mut self, i2c: &mut I2C, mode: MeasurementMode) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        // Mask for the measurement mode setting
        let mm_mask = !(ControlRegister3Bits::MD1 | ControlRegister3Bits::MD0);

        let mm = match mode {
            MeasurementMode::Idle => ControlRegister3Bits::MD1,
            MeasurementMode::SingleMeasurement => ControlRegister3Bits::MD0,
            MeasurementMode::Continuous => ControlRegister3Bits::empty(),
        };

        // zero out the entries for MD1 and MD0
        let existing_reg_3_settings = self.read_control_register_3(i2c)? & mm_mask;

        // Update the measurement mode settings, preserving the other values
        self.set_control_register_3(i2c, existing_reg_3_settings | mm)
    }

    /// Set the output data rate. Specific data rates from 0.625 Hz to 80 Hz can be configured for
    /// any given operating mode, and Fast will be the highest achievable data rate for the given
    /// operating mode at 1000 Hz for Low Power and 155 Hz for Ultra High Performance. See AN4602
    /// for more details.
    pub fn set_data_rate<I2C>(&mut self, i2c: &mut I2C, rate: DataRate) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        // Mask for the data rate setting
        let odr_mask = !(ControlRegister1Bits::DO2 | ControlRegister1Bits::DO1
            | ControlRegister1Bits::DO0 | ControlRegister1Bits::FAST_ODR);

        let odr = match rate {
            DataRate::ODR_0_625Hz => ControlRegister1Bits::empty(),
            DataRate::ODR_1_25Hz => ControlRegister1Bits::DO0,
            DataRate::ODR_2_5Hz => ControlRegister1Bits::DO1,
            DataRate::ODR_5Hz => ControlRegister1Bits::DO1 | ControlRegister1Bits::DO0,
            DataRate::ODR_10Hz => ControlRegister1Bits::DO2,
            DataRate::ODR_20Hz => ControlRegister1Bits::DO2 | ControlRegister1Bits::DO0,
            DataRate::ODR_40Hz => ControlRegister1Bits::DO2 | ControlRegister1Bits::DO1,
            DataRate::ODR_80Hz => ControlRegister1Bits::DO2 | ControlRegister1Bits::DO1
                                  | ControlRegister1Bits::DO0,
            DataRate::ODR_Fast => ControlRegister1Bits::FAST_ODR,
        };

        // zero out the entries for DO2, DO1, DO0, and FAST_ODR
        let existing_reg_1_settings = self.read_control_register_1(i2c)? & odr_mask;

        // Update the measurement mode settings, preserving other values
        self.set_control_register_1(i2c, existing_reg_1_settings | odr)
    }

    /// Blocks the refresh of data for a given axis until the initiated read for that axis
    /// completes. Strongly recommended if the reading of the magnetic data cannot be synchronized
    /// with the XYZDA in the status register. Ensures that data registers for each channel always
    /// contain the most recent magnetic data.
    pub fn set_block_data_update<I2C>(&mut self, i2c: &mut I2C, block: bool) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        let bdu_mask = !ControlRegister5Bits::BDU;

        let existing_reg_5_settings = self.read_control_register_5(i2c)? & bdu_mask;

        if block {
            self.set_control_register_5(i2c, existing_reg_5_settings | ControlRegister5Bits::BDU)
        }
        else {
            self.set_control_register_5(i2c, existing_reg_5_settings)
        }
    }

    /// Enables the temperature sensor
    pub fn set_temperature_sensor_enable<I2C>(&mut self, i2c: &mut I2C, enabled: bool) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        let temp_mask = !ControlRegister1Bits::TEMP_EN;

        let existing_reg_1_settings = self.read_control_register_1(i2c)? & temp_mask;

        if enabled {
            self.set_control_register_1(i2c, existing_reg_1_settings | ControlRegister1Bits::TEMP_EN)
        }
        else {
            self.set_control_register_1(i2c, existing_reg_1_settings)
        }
    }

    fn read_x_raw<I2C>(&mut self, i2c: &mut I2C) -> Result<i16, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        self.read_register_i16(i2c, Register::OUT_X_H, Register::OUT_X_L)
    }

    fn read_y_raw<I2C>(&mut self, i2c: &mut I2C) -> Result<i16, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        self.read_register_i16(i2c, Register::OUT_Y_H, Register::OUT_Y_L)
    }

    fn read_z_raw<I2C>(&mut self, i2c: &mut I2C) -> Result<i16, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        self.read_register_i16(i2c, Register::OUT_Z_H, Register::OUT_Z_L)
    }

    fn read_device_status<I2C>(&mut self, i2c: &mut I2C) -> Result<StatusRegisterBits, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        Ok(StatusRegisterBits::from_bits_truncate(self.read_register(i2c, Register::STATUS_REG)?))
    }

    fn set_control_register_1<I2C>(&mut self, i2c: &mut I2C, bits: ControlRegister1Bits) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        self.write_register(i2c, Register::CTRL_REG1, bits.bits())
    }

    fn set_control_register_2<I2C>(&mut self, i2c: &mut I2C, bits: ControlRegister2Bits) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        self.write_register(i2c, Register::CTRL_REG2, bits.bits())
    }

    fn set_control_register_3<I2C>(&mut self, i2c: &mut I2C, bits: ControlRegister3Bits) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        self.write_register(i2c, Register::CTRL_REG3, bits.bits())
    }

    fn set_control_register_4<I2C>(&mut self, i2c: &mut I2C, bits: ControlRegister4Bits) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        self.write_register(i2c, Register::CTRL_REG4, bits.bits())
    }

    fn set_control_register_5<I2C>(&mut self, i2c: &mut I2C, bits: ControlRegister5Bits) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        self.write_register(i2c, Register::CTRL_REG5, bits.bits())
    }

    fn read_control_register_1<I2C>(&mut self, i2c: &mut I2C) -> Result<ControlRegister1Bits, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        Ok(ControlRegister1Bits::from_bits_truncate(self.read_register(i2c, Register::CTRL_REG1)?))
    }

    fn read_control_register_2<I2C>(&mut self, i2c: &mut I2C) -> Result<ControlRegister2Bits, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        Ok(ControlRegister2Bits::from_bits_truncate(self.read_register(i2c, Register::CTRL_REG2)?))
    }

    fn read_control_register_3<I2C>(&mut self, i2c: &mut I2C) -> Result<ControlRegister3Bits, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        Ok(ControlRegister3Bits::from_bits_truncate(self.read_register(i2c, Register::CTRL_REG3)?))
    }

    fn read_control_register_4<I2C>(&mut self, i2c: &mut I2C) -> Result<ControlRegister4Bits, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        Ok(ControlRegister4Bits::from_bits_truncate(self.read_register(i2c, Register::CTRL_REG4)?))
    }

    fn read_control_register_5<I2C>(&mut self, i2c: &mut I2C) -> Result<ControlRegister5Bits, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        Ok(ControlRegister5Bits::from_bits_truncate(self.read_register(i2c, Register::CTRL_REG5)?))
    }

    fn read_register_i16<I2C>(&mut self, i2c: &mut I2C, reg_low: Register, reg_high: Register) -> Result<i16, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        let low = self.read_register(i2c, reg_low)?;
        let high = self.read_register(i2c, reg_high)?;

        // Convert the low and high bytes to signed 16-bit integer
        let signed = i16::from_le_bytes([high, low]);
        Ok(signed)
    }

    fn read_register<I2C>(&mut self, i2c: &mut I2C, reg: Register) -> Result<u8, Error<I2C::Error>>
    where 
        I2C: I2c
    {
        let mut buffer: [u8; 1] = [0];
        i2c.write_read(self.address, &[reg.addr()], &mut buffer)
            .map_err(Error::CommunicationError)?;
        Ok(buffer[0])
    }

    fn write_register<I2C>(&mut self, i2c: &mut I2C, reg: Register, byte: u8) -> Result<(), Error<I2C::Error>>
    where 
        I2C: I2c
    {
        i2c.write(self.address, &[reg.addr(), byte])
            .map_err(Error::CommunicationError)
    }
}

#[allow(non_camel_case_types, dead_code)]
#[derive(Debug, Copy, Clone)]
enum Register {
    OFFSET_X_REG_L_M = 0x05,
    OFFSET_X_REG_H_M = 0x06,
    OFFSET_Y_REG_L_M = 0x07,
    OFFSET_Y_REG_H_M = 0x08,
    OFFSET_Z_REG_L_M = 0x09,
    OFFSET_Z_REG_H_M = 0x0A,

    WHO_AM_I = 0x0F,

    CTRL_REG1 = 0x20,
    CTRL_REG2 = 0x21,
    CTRL_REG3 = 0x22,
    CTRL_REG4 = 0x23,
    CTRL_REG5 = 0x24,

    STATUS_REG = 0x27,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D,
    TEMP_OUT_L = 0x2E,
    TEMP_OUT_H = 0x2F,
    INT_CFG = 0x30,
    INT_SRC = 0x31,
    INT_THS_L = 0x32,
    INT_THS_H = 0x33
}

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}

bitflags! {
#[allow(non_camel_case_types, dead_code)]
struct ControlRegister1Bits: u8 {
    const TEMP_EN = 0b1000_0000;
    const OM1 = 0b0100_0000;
    const OM0 = 0b0010_0000;
    const DO2 = 0b0001_0000;
    const DO1 = 0b0000_1000;
    const DO0 = 0b0000_0100;
    const FAST_ODR = 0b0000_0010;
    const ST = 0b0000_0001;
}
}

bitflags! {
#[allow(non_camel_case_types, dead_code)]
struct ControlRegister2Bits: u8 {
    const FS1 = 0b0100_0000;
    const FS0 = 0b0010_0000;
    const REBOOT = 0b0000_1000;
    const SOFT_RST = 0b0000_0100;
}
}

bitflags! {
#[allow(non_camel_case_types, dead_code)]
struct ControlRegister3Bits: u8 {
    const LP = 0b0010_0000;
    const SIM = 0b0000_0100;
    const MD1 = 0b0000_0010;
    const MD0 = 0b0000_0001;
}
}

bitflags! {
struct ControlRegister4Bits: u8 {
    const OMZ1 = 0b0000_1000;
    const OMZ0 = 0b0000_0100;
    const BLE = 0b0000_0010;
    const UHP = 0b0000_1100;
}
}

bitflags! {
struct ControlRegister5Bits: u8 {
    const FAST_READ = 0b1000_0000;
    const BDU = 0b0100_0000;
}
}

bitflags! {
struct StatusRegisterBits: u8 {
    const ZYXOR = 0b1000_0000;
    const ZOR = 0b0100_0000;
    const YOR = 0b0010_0000;
    const XOR = 0b0001_0000;
    const ZYXDA = 0b0000_1000;
    const ZDA = 0b0000_0100;
    const YDA = 0b0000_0010;
    const XDA = 0b0000_0001;
}
}

bitflags! {
#[allow(non_camel_case_types, dead_code)]
struct FullScaleBits: u8 {
    const FS4G = 0b0000_0000;
    const FS8G = 0b0010_0000;
    const FS12G = 0b0100_0000;
    const FS16G = 0b0110_0000;
}
}