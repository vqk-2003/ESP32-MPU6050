#![no_std]

use esp_hal::{Async, i2c::master::I2c};

pub struct Mpu6050<'d> {
    i2c: I2c<'d, Async>,
    dev_addr: u8,
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
        Self { i2c, dev_addr }
    }

    pub async fn init(&mut self) {
        // Initialization process is based on:
        // https://docs.espressif.com/projects/espressif-esp-drone/en/latest/drivers.html        const PWR_MGMT_1_REG: u8 = 107;

        // Set X axis gyroscope clock reference
        const PWR_MGMT_1_REG: u8 = 107;
        const CLKSEL: u8 = 0;
        self.write(PWR_MGMT_1_REG, 1 << CLKSEL).await;

        // Set range of gyroscope (2000 degree/s)
        const GYRO_CONFIG_REG: u8 = 27;
        const FS_SEL: u8 = 3;
        self.write(GYRO_CONFIG_REG, 3 << FS_SEL).await;

        // Set range of accelerometer (2g)
        const ACCEL_CONFIG: u8 = 28;
        const AFS_SEL: u8 = 3;
        self.write(ACCEL_CONFIG, 0 << AFS_SEL).await;

        // Set sample rate (125 Hz)
        const SMPRT_DIV_REG: u8 = 25;
        self.write(SMPRT_DIV_REG, 7).await;

        // Set Digital Low-Pass Filter (5 Hz)
        const CONFIG_REG: u8 = 26;
        const DLPF_CFG: u8 = 0;
        self.write(CONFIG_REG, 6 << DLPF_CFG).await;
    }

    pub async fn get_measurements(&mut self) -> Measurements {
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

        // Real values
        const G: f32 = 9.8067;
        const ACCEL_LSB_SENSITIVITY: f32 = 16384.0;
        const GYRO_LSB_SENSITIVITY: f32 = 16.4;
        let accel_x = accel_x as f32 * G / ACCEL_LSB_SENSITIVITY;
        let accel_y = accel_y as f32 * G / ACCEL_LSB_SENSITIVITY;
        let accel_z = accel_z as f32 * G / ACCEL_LSB_SENSITIVITY;
        let temp = temp as f32 / 340.0 + 36.53;
        let gyro_x = gyro_x as f32 / GYRO_LSB_SENSITIVITY;
        let gyro_y = gyro_y as f32 / GYRO_LSB_SENSITIVITY;
        let gyro_z = gyro_z as f32 / GYRO_LSB_SENSITIVITY;

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
