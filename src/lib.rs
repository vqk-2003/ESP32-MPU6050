#![no_std]

use esp_hal::{Async, i2c::master::I2c};

pub struct Mpu6050<'d> {
    i2c: I2c<'d, Async>,
    dev_addr: u8,
    accel_x_offset: i16,
    accel_y_offset: i16,
    accel_z_offset: i16,
    gyro_x_offset: i16,
    gyro_y_offset: i16,
    gyro_z_offset: i16,
}

pub struct RawMeasurements {
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub temp: i16,
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
}

pub struct Measurements {
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub temp: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
}

impl<'d> Mpu6050<'d> {
    pub fn new(i2c: I2c<'d, Async>, dev_addr: u8) -> Self {
        Self {
            i2c,
            dev_addr,
            accel_x_offset: 0,
            accel_y_offset: 0,
            accel_z_offset: 0,
            gyro_x_offset: 0,
            gyro_y_offset: 0,
            gyro_z_offset: 0,
        }
    }

    pub async fn init(&mut self) {
        // Initialization process is based on:
        // https://docs.espressif.com/projects/espressif-esp-drone/en/latest/drivers.html

        // Set X axis gyroscope clock reference
        const PWR_MGMT_1_REG: u8 = 107;
        const CLKSEL: u8 = 0;
        self.write(PWR_MGMT_1_REG, 1 << CLKSEL).await;

        // Set range of gyroscope (250 degree/s)
        const GYRO_CONFIG_REG: u8 = 27;
        const FS_SEL: u8 = 3;
        self.write(GYRO_CONFIG_REG, 0 << FS_SEL).await;

        // Set range of accelerometer (2g)
        // and DHPF to 5Hz
        const ACCEL_CONFIG: u8 = 28;
        const AFS_SEL: u8 = 3;
        const ACCEL_HPF: u8 = 0;
        self.write(ACCEL_CONFIG, (0 << AFS_SEL) | (1 << ACCEL_HPF))
            .await;

        // Set sample rate (125 Hz)
        const SMPRT_DIV_REG: u8 = 25;
        self.write(SMPRT_DIV_REG, 7).await;

        // Set Digital Low-Pass Filter (5 Hz)
        const CONFIG_REG: u8 = 26;
        const DLPF_CFG: u8 = 0;
        self.write(CONFIG_REG, 6 << DLPF_CFG).await;
    }

    pub fn set_offsets(
        &mut self,
        accel_x_offset: i16,
        accel_y_offset: i16,
        accel_z_offset: i16,
        gyro_x_offset: i16,
        gyro_y_offset: i16,
        gyro_z_offset: i16,
    ) {
        self.accel_x_offset = accel_x_offset;
        self.accel_y_offset = accel_y_offset;
        self.accel_z_offset = accel_z_offset;
        self.gyro_x_offset = gyro_x_offset;
        self.gyro_y_offset = gyro_y_offset;
        self.gyro_z_offset = gyro_z_offset;
    }

    pub async fn get_raw_measurements(&mut self) -> RawMeasurements {
        const START_REG_ADDR: u8 = 59;
        let mut buffer = [0; 14];
        self.i2c
            .write_read_async(self.dev_addr, &[START_REG_ADDR], &mut buffer)
            .await
            .unwrap();

        // Raw values
        let accel_x = i16::from_be_bytes([buffer[0], buffer[1]]);
        let accel_y = i16::from_be_bytes([buffer[2], buffer[3]]);
        let accel_z = i16::from_be_bytes([buffer[4], buffer[5]]);
        let temp = i16::from_be_bytes([buffer[6], buffer[7]]);
        let gyro_x = i16::from_be_bytes([buffer[8], buffer[9]]);
        let gyro_y = i16::from_be_bytes([buffer[10], buffer[11]]);
        let gyro_z = i16::from_be_bytes([buffer[12], buffer[13]]);

        RawMeasurements {
            accel_x,
            accel_y,
            accel_z,
            temp,
            gyro_x,
            gyro_y,
            gyro_z,
        }
    }

    pub async fn get_measurements(&mut self) -> Measurements {
        let raw_meas = self.get_raw_measurements().await;
        const G: f32 = 9.8067;
        const ACCEL_LSB_SENSITIVITY: f32 = 16384.0;
        const GYRO_LSB_SENSITIVITY: f32 = 131.0;
        let accel_x = (raw_meas.accel_x - self.accel_x_offset) as f32 * G / ACCEL_LSB_SENSITIVITY;
        let accel_y = (raw_meas.accel_y - self.accel_y_offset) as f32 * G / ACCEL_LSB_SENSITIVITY;
        let accel_z = (raw_meas.accel_z - self.accel_z_offset) as f32 * G / ACCEL_LSB_SENSITIVITY;
        let temp = raw_meas.temp as f32 / 340.0 + 36.53;
        let gyro_x = (raw_meas.gyro_x - self.gyro_x_offset) as f32 / GYRO_LSB_SENSITIVITY;
        let gyro_y = (raw_meas.gyro_y - self.gyro_y_offset) as f32 / GYRO_LSB_SENSITIVITY;
        let gyro_z = (raw_meas.gyro_z - self.gyro_z_offset) as f32 / GYRO_LSB_SENSITIVITY;

        Measurements {
            accel_x,
            accel_y,
            accel_z,
            temp,
            gyro_x,
            gyro_y,
            gyro_z,
        }
    }

    async fn write(&mut self, reg_addr: u8, val: u8) {
        self.i2c
            .write_async(self.dev_addr, &[reg_addr, val])
            .await
            .unwrap();
    }
}
