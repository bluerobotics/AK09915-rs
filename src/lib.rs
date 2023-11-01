use embedded_hal::blocking::i2c::{Write, WriteRead};

const AK09915_ADDRESS: u8 = 0x0C;
///Magnetic sensor sensitivity (BSE) for Ta = 25 ˚C [µT/LSB],  Typical 0.15 +- 0.0075
const AK09915_FLUX_CONSTANT: f32 = 0.15;

// Register Addresses
#[repr(u16)]
pub enum Register {
    WIA1 = 0x00,   // Device ID Register 1
    WIA2 = 0x01,   // Device ID Register 2
    ST1 = 0x10,    // Data Status Register 1
    HXL = 0x11,    // Data Register - X-axis Magnetic Data Low Byte
    HXH = 0x12,    // Data Register - X-axis Magnetic Data High Byte
    HYL = 0x13,    // Data Register - Y-axis Magnetic Data Low Byte
    HYH = 0x14,    // Data Register - Y-axis Magnetic Data High Byte
    HZL = 0x15,    // Data Register - Z-axis Magnetic Data Low Byte
    HZH = 0x16,    // Data Register - Z-axis Magnetic Data High Byte
    TMPS = 0x17,   // Temperature Sensor Data Register
    ST2 = 0x18,    // Data Status Register 2
    CNTL2 = 0x31,  // Control Register 2
    CNTL3 = 0x32,  // Control Register 3
    TS1 = 0x33,    // Self Test Register 1
    TS2 = 0x34,    // Self Test Register 2
    I2CDIS = 0x3A, // I2C Disable Register
}

// AK09915 Mode Settings - Corresponding to Control Register 2
#[repr(u16)]
pub enum Mode {
    PowerDown = 0x00,
    Single = 0x01,
    Cont10Hz = 0x02,
    Cont20Hz = 0x04,
    Cont50Hz = 0x06,
    Cont100Hz = 0x08,
    Cont200Hz = 0x0A,
    Cont1Hz = 0x0C,
    SelfTest = 0x10,
}

#[derive(Debug, PartialEq)]
pub enum Error<E> {
    I2C(E),
    SensorOverflow,
    DataNotReady,
}

impl From<Mode> for u8 {
    fn from(mode: Mode) -> Self {
        mode as u8
    }
}

impl From<Register> for u8 {
    fn from(register: Register) -> Self {
        register as u8
    }
}

pub struct Ak09915<I2C> {
    pub i2c: I2C,
    pub address: u8,
}

impl<I2C, E> Ak09915<I2C>
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: AK09915_ADDRESS,
        }
    }

    pub fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error<E>> {
        self.i2c
            .write(self.address, &[register.into(), value])
            .map_err(Error::I2C)
    }

    pub fn read_register(&mut self, register: Register) -> Result<u8, Error<E>> {
        let mut buffer = [0u8];
        self.i2c
            .write_read(self.address, &[register.into()], &mut buffer)
            .map_err(Error::I2C)
            .and(Ok(buffer[0]))
    }

    pub fn init(&mut self) -> Result<(), Error<E>> {
        // Soft reset device and put on continuous measurement mode, with 200 Hz
        self.reset()?;
        self.set_mode(Mode::Cont200Hz)?;
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Error<E>> {
        // Soft reset device
        self.write_register(Register::CNTL3, 0x01)?;
        Ok(())
    }

    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E>> {
        self.write_register(Register::CNTL2, Mode::PowerDown.into())?;
        std::thread::sleep(std::time::Duration::from_micros(100));
        self.write_register(Register::CNTL2, mode.into())?;
        Ok(())
    }

    // 9.4.4.1. Self-test Sequence:
    //   1. Set Power-down mode (MODE[4:0] bits = "00000").
    //   2. Set Self-test mode (MODE[4:0] bits = "10000").
    //   3. Check Data Ready by:
    //      - Polling DRDY bit of ST1 register.
    //      - Monitoring DRDY pin.
    //      When Data Ready, proceed to the next step.
    //   4. Read measurement data (HXL to HZH).
    // 9.4.4.2. Self-test Judgment:
    //   If measurement data read by the above sequence is within the following ranges,
    //   AK09915 is working normally:
    //     - HX[15:0] bits: -200 ≤ HX ≤ +200
    //     - HY[15:0] bits: -200 ≤ HY ≤ +200
    //     - HZ[15:0] bits: -800 ≤ HZ ≤ -200

    pub fn self_test(&mut self) -> Result<bool, Error<E>> {
        self.set_mode(Mode::SelfTest)?;
        std::thread::sleep(std::time::Duration::from_micros(4000));
        let (hx, hy, hz) = self.read_raw()?;
        // Self-test judgment
        if (-200..=200).contains(&hx) && (-200..=200).contains(&hy) && (-800..=-200).contains(&hz) {
            return Ok(true);
        }
        Ok(false)
    }

    pub fn check_data_ready(&mut self) -> Result<(), Error<E>> {
        let mut retries = 10;
        while retries > 0 {
            let status = self.read_register(Register::ST1)?;
            if (status & 0x01) != 0 {
                return Ok(()); // Data ready
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
            retries -= 1;
        }
        Err(Error::DataNotReady)
    }
    // 9.4.3.2. Normal Read Sequence:
    //   1. Check Data Ready or not by any of the following method:
    //      |ST2| -> | 0 0 0 0 HOFL INV 0 0 |
    pub fn check_overflow(&mut self) -> Result<(), Error<E>> {
        let status = self.read_register(Register::ST2)?;
        if (status & 0x08) != 0 {
            return Err(Error::SensorOverflow);
        }
        Ok(())
    }

    // Return the readings in magnetic flux density [uT]
    pub fn read(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let raw_data = self.read_raw()?;
        let scaled_data = (
            raw_data.0 as f32 * AK09915_FLUX_CONSTANT,
            raw_data.1 as f32 * AK09915_FLUX_CONSTANT,
            raw_data.2 as f32 * AK09915_FLUX_CONSTANT,
        );
        Ok(scaled_data)
    }

    // 9.4.3.2. Normal Read Sequence:
    //   1. Check Data Ready or not by any of the following method:
    //      - Polling DRDY bit of ST1 register
    //      - Monitor DRDY pin
    //      When Data Ready, proceed to the next step.
    //   2. Read ST1 register (not needed when polling ST1)
    //      - DRDY: Shows Data Ready or not. Not when "0", Data Ready when "1".
    //      - DOR: Shows if any data has been skipped before the current data or not.
    //      There are no skipped data when "0", there are skipped data when "1".
    //   3. Read measurement data:
    //      - When any of measurement data registers (HXL to TMPS) or ST2 register is read,
    //      AK09915 judges that data reading is started. When data reading is started,
    //      DRDY bit and DOR bit turns to "0".
    //   4. Read ST2 register (required):
    //      - HOFL: Shows if magnetic sensor is overflowed or not. "0" means not overflowed,
    //      "1" means overflowed.
    //      - When ST2 register is read, AK09915 judges that data reading is finished.
    //      Stored measurement data is protected during data reading and data is not updated.
    //      By reading ST2 register, this protection is released. It is required to read
    //      ST2 register after data reading.

    pub fn read_raw(&mut self) -> Result<(i16, i16, i16), Error<E>> {
        self.check_data_ready()?;

        let mut buffer: [u8; 8] = [0u8; 8];
        self.i2c
            .write_read(self.address, &[Register::HXL.into()], &mut buffer)
            .map_err(Error::I2C)?;

        // Check the HOFL bit from ST2 register
        if (buffer[7] & 0x08) != 0 {
            return Err(Error::SensorOverflow);
        }

        let x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let z = i16::from_le_bytes([buffer[4], buffer[5]]);
        Ok((x, y, z))
    }

    pub fn read_unchecked(&mut self) -> Result<(i16, i16, i16), Error<E>> {
        let mut buffer: [u8; 6] = [0u8; 6];
        self.i2c
            .write_read(self.address, &[Register::HXL.into()], &mut buffer)
            .map_err(Error::I2C)?;
        let x = i16::from_le_bytes([buffer[0], buffer[1]]);
        let y = i16::from_le_bytes([buffer[2], buffer[3]]);
        let z = i16::from_le_bytes([buffer[4], buffer[5]]);
        Ok((x, y, z))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTrans};
    #[test]
    fn read_sensor() {
        let expected_trans = [
            I2cTrans::write_read(AK09915_ADDRESS, vec![Register::ST1.into()], vec![0x01]),
            I2cTrans::write_read(
                AK09915_ADDRESS,
                vec![Register::HXL.into()],
                vec![0x04, 0x23, 0x05, 0x24, 0x06, 0x25, 0x00, 0x01],
            ),
        ];

        // Create a mock I2C device and queue the expected transactions
        let i2c_mock = I2cMock::new(&expected_trans);

        // Create an AK09915 instance using the mock I2C device
        let mut sensor = Ak09915::new(i2c_mock);

        let (x, y, z) = sensor.read_raw().expect("Error reading magnetometer");

        // Verify that the magnetometer data matches the expected values
        assert_eq!(x, 0x2304);
        assert_eq!(y, 0x2405);
        assert_eq!(z, 0x2506);
    }
}
