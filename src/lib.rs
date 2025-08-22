//! # VL53L4CD Ultra-Low-Power Time-of-Flight Distance Sensor Driver
//!
//! This crate provides an async, `no_std` driver for ST-Microelectronics' VL53L4CD ultra-low-power
//! time-of-flight distance sensor.
//!
//! ## Basic Usage
//!
//! ```rust,no_run
//! use vl53l4cd_ulp::VL53L4cd;
//!
//! let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
//! let delay = embedded_hal_mock::eh1::delay::NoopDelay;
//! let mut sensor = VL53L4cd::new(i2c, delay);
//!
//! sensor.sensor_init().unwrap();
//! sensor.start_ranging().unwrap();
//!
//! // Wait for data ready (you can connect a GPIO to the interrupt pin to detect when new data is available)
//! if sensor.check_for_data_ready().unwrap() {
//!     let measurement = sensor.get_estimated_measurement().unwrap();
//!     println!("Distance: {} mm", measurement.estimated_distance_mm);
//!     sensor.clear_interrupt().unwrap();
//! }
//! ```
#![no_std]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]

mod fmt; // <-- must be first module!

#[cfg(not(feature = "async"))]
use embedded_hal::{delay::DelayNs, i2c::I2c};
#[cfg(feature = "async")]
use embedded_hal_async::{delay::DelayNs, i2c::I2c};

// This is the initialization sequence for the VL53L4CD from the Ultra Low Power Driver
const VL53L4CD_DEFAULT_CONFIGURATION: [u8; 91] = [
    0x00, /* 0x2d */
    0x00, /* 0x2e */
    0x00, /* 0x2f */
    0x11, /* 0x30 */
    0x02, /* 0x31 */
    0x00, /* 0x32 */
    0x02, /* 0x33 */
    0x08, /* 0x34 */
    0x00, /* 0x35 */
    0x08, /* 0x36 */
    0x10, /* 0x37 */
    0x01, /* 0x38 */
    0x01, /* 0x39 */
    0x00, /* 0x3a */
    0x00, /* 0x3b */
    0x00, /* 0x3c */
    0x00, /* 0x3d */
    0xff, /* 0x3e */
    0x00, /* 0x3f */
    0x0F, /* 0x40 */
    0x00, /* 0x41 */
    0x00, /* 0x42 */
    0x00, /* 0x43 */
    0x00, /* 0x44 */
    0x00, /* 0x45 */
    0x20, /* 0x46 */
    0x0b, /* 0x47 */
    0x00, /* 0x48 */
    0x00, /* 0x49 */
    0x02, /* 0x4a */
    0x14, /* 0x4b */
    0x21, /* 0x4c */
    0x00, /* 0x4d */
    0x00, /* 0x4e */
    0x05, /* 0x4f */
    0x00, /* 0x50 */
    0x00, /* 0x51 */
    0x00, /* 0x52 */
    0x00, /* 0x53 */
    0xc8, /* 0x54 */
    0x00, /* 0x55 */
    0x00, /* 0x56 */
    0x38, /* 0x57 */
    0xff, /* 0x58 */
    0x01, /* 0x59 */
    0x00, /* 0x5a */
    0x08, /* 0x5b */
    0x00, /* 0x5c */
    0x00, /* 0x5d */
    0x00, /* 0x5e */
    0x01, /* 0x5f */
    0x07, /* 0x60 */
    0x00, /* 0x61 */
    0x02, /* 0x62 */
    0x05, /* 0x63 */
    0x00, /* 0x64 */
    0xb4, /* 0x65 */
    0x00, /* 0x66 */
    0xbb, /* 0x67 */
    0x08, /* 0x68 */
    0x38, /* 0x69 */
    0x00, /* 0x6a */
    0x00, /* 0x6b */
    0x00, /* 0x6c */
    0x00, /* 0x6d */
    0x0f, /* 0x6e */
    0x89, /* 0x6f */
    0x00, /* 0x70 */
    0x00, /* 0x71 */
    0x00, /* 0x72 */
    0x00, /* 0x73 */
    0x00, /* 0x74 */
    0x00, /* 0x75 */
    0x00, /* 0x76 */
    0x01, /* 0x77 */
    0x07, /* 0x78 */
    0x05, /* 0x79 */
    0x06, /* 0x7a */
    0x06, /* 0x7b */
    0x00, /* 0x7c */
    0x00, /* 0x7d */
    0x02, /* 0x7e */
    0xc7, /* 0x7f */
    0xff, /* 0x80 */
    0x9B, /* 0x81 */
    0x00, /* 0x82 */
    0x00, /* 0x83 */
    0x00, /* 0x84 */
    0x01, /* 0x85 */
    0x00, /* 0x86 */
    0x00, /* 0x87 */
];

/// Register addresses for the VL53L4CD sensor.
#[repr(u16)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Register {
    /// I2C slave device address register (0x0001)
    I2cSlaveDeviceAddress = 0x0001,
    /// VHV configuration timeout macro loop bound register (0x0008)
    VhvConfigTimeoutMacropLoopBound = 0x0008,
    /// GPIO HV mux control register (0x0030)
    GpioHvMuxCtrl = 0x0030,
    /// GPIO TIO HV status register (0x0031)
    GpioTioHvStatus = 0x0031,
    /// System interrupt configuration register (0x0046)
    SystemInterrupt = 0x0046,
    /// Range configuration A register (0x005E)
    RangeConfigA = 0x005E,
    /// Range configuration B register (0x0061)
    RangeConfigB = 0x0061,
    /// Range configuration sigma threshold register (0x0064)
    RangeConfigSigmaThresh = 0x0064,
    /// Minimum count rate return limit MCPS register (0x0066)
    MinCountRateRtnLimitMcps = 0x0066,
    /// Inter-measurement period in milliseconds register (0x006C)
    IntermeasurementMs = 0x006C,
    /// High threshold register (0x0072)
    ThreshHigh = 0x0072,
    /// Low threshold register (0x0074)
    ThreshLow = 0x0074,
    /// Power GO1 register (0x0083)
    PowerGo1 = 0x0083,
    /// Firmware enable register (0x0085)
    FirmwareEnable = 0x0085,
    /// System interrupt clear register (0x0086)
    SystemInterruptClear = 0x0086,
    /// System start register (0x0087)
    SystemStart = 0x0087,
    /// Result range status register (0x0089)
    ResultRangeStatus = 0x0089,
    /// Result SPAD number register (0x008C)
    ResultSpadNb = 0x008C,
    /// Result signal rate register (0x008E)
    ResultSignalRate = 0x008E,
    /// Result ambient rate register (0x0090)
    ResultAmbientRate = 0x0090,
    /// Result sigma register (0x0092)
    ResultSigma = 0x0092,
    /// Result distance register (0x0096)
    ResultDistance = 0x0096,
    /// Result oscillator calibration value register (0x00DE)
    ResultOscCalibrateVal = 0x00DE,
    /// Firmware system status register (0x00E5)
    FirmwareSystemStatus = 0x00E5,
    /// Identification model ID register (0x010F)
    IdentificationModelId = 0x010F,
}

impl From<Register> for u16 {
    fn from(r: Register) -> Self {
        r as u16
    }
}

/// Interrupt configuration options for the VL53L4CD sensor.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptOn {
    /// Interrupt triggered on level low detection
    LevelLow,
    /// Interrupt triggered on level high detection
    LevelHigh,
    /// Interrupt triggered when distance is out of threshold window
    OutOfWindow,
    /// Interrupt triggered when distance is out of threshold window or no target detected
    OutOfWindowOrNoTarget,
    /// Interrupt triggered when distance is within threshold window
    InWindow,
    /// Interrupt triggered when new ranging data is available
    NewSampleReady,
    /// Custom interrupt configuration value
    Unknown(u8),
}

impl From<InterruptOn> for u8 {
    fn from(interrupt_on: InterruptOn) -> Self {
        match interrupt_on {
            InterruptOn::LevelLow => 0,
            InterruptOn::LevelHigh => 1,
            InterruptOn::OutOfWindow => 2,
            InterruptOn::OutOfWindowOrNoTarget => 0x42,
            InterruptOn::InWindow => 3,
            InterruptOn::NewSampleReady => 0x20,
            InterruptOn::Unknown(value) => value,
        }
    }
}

impl From<u8> for InterruptOn {
    fn from(value: u8) -> Self {
        match value {
            0 => InterruptOn::LevelLow,
            1 => InterruptOn::LevelHigh,
            2 => InterruptOn::OutOfWindow,
            0x42 => InterruptOn::OutOfWindowOrNoTarget,
            3 => InterruptOn::InWindow,
            0x20 => InterruptOn::NewSampleReady,
            _ => {
                warn!("Unknown InterruptOn value: {}", value);
                InterruptOn::Unknown(value)
            }
        }
    }
}

/// Distance measurement result from the VL53L4CD sensor.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EstimatedMeasurement {
    /// Measurement status code indicating validity and quality
    pub measurement_status: u8,
    /// Estimated distance in millimeters (0-4000 mm)
    pub estimated_distance_mm: u16,
    /// Measurement precision (1σ) in millimeters
    pub sigma_mm: u16,
    /// Signal rate in kilocounts per second (kcps)
    pub signal_kcps: u16,
    /// Ambient light rate in kilocounts per second (kcps)
    pub ambient_kcps: u16,
}

/// VL53L4CD ultra-low-power time-of-flight distance sensor driver.
///
/// This struct provides an async interface to control and read data from the VL53L4CD
/// sensor. It manages the I2C communication, sensor configuration, and ranging operations.
///
/// The driver is generic over the I2C and delay implementations, allowing it to work
/// with any embedded-hal-async compatible hardware.
pub struct VL53L4cd<I2C, D> {
    /// I2C interface for communication with the sensor
    i2c: I2C,
    /// Current I2C slave address of the sensor
    address: u8,
    /// Delay implementation for timing operations
    delay: D,
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), keep_self),
    async(feature = "async", keep_self)
)]
impl<I2C, E, D> VL53L4cd<I2C, D>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
    D: DelayNs,
{
    /// Creates a new VL53L4CD sensor driver instance.
    ///
    /// This function initializes a new sensor driver with the default I2C address (0x29)
    /// and the provided I2C and delay implementations. The sensor is not yet initialized
    /// and must be configured using [`sensor_init`](Self::sensor_init) before use.
    ///
    /// # Arguments
    ///
    /// * `i2c` - I2C interface implementation for sensor communication
    /// * `delay` - Delay implementation for timing operations
    ///
    /// # Returns
    ///
    /// A new `VL53L4cd` instance with default configuration
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    /// use embedded_hal::{i2c::I2c, delay::DelayNs};
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    ///
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    /// ```
    ///
    /// # Default Configuration
    ///
    /// - **I2C Address**: 0x29 (default sensor address)
    /// - **Sensor State**: Uninitialized (must call `sensor_init()`)
    /// - **Ranging Mode**: Stopped
    pub fn new(i2c: I2C, delay: D) -> Self {
        Self {
            i2c,
            address: 0x29,
            delay,
        }
    }

    /// Sets the I2C address of the sensor.
    ///
    /// This function writes a new I2C slave address to the sensor's internal register.
    /// The new address will take effect after the sensor is reset or reinitialized.
    ///
    /// **Note**: The address change only takes effect when the sensor is in a reset state.
    /// According to the VL53L4CD Application Note, to change the I2C address, the host must:
    /// 1. Put the device in HW standby by setting the XSHUT pin low
    /// 2. Raise the XSHUT pin
    /// 3. Call `set_i2c_address(new_address)` to program the new address,
    /// 4. call `sensor_init()` to initialize the sensor on the new address
    ///
    /// The current driver instance will continue to use the old address until reinitialization.
    ///
    /// # Arguments
    ///
    /// * `address` - The new 7-bit I2C address
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the address was set successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// // Change sensor address to 0x30
    /// sensor.set_i2c_address(0x30).unwrap();
    ///
    /// // Reinitialize to use new address
    /// sensor.sensor_init().unwrap();
    /// ```
    pub async fn set_i2c_address(&mut self, address: u8) -> Result<(), Error<E>> {
        self.write_byte(Register::I2cSlaveDeviceAddress, address)
            .await?;
        self.address = address;
        Ok(())
    }

    /// Retrieves the sensor identification model ID.
    ///
    /// This function reads the sensor's model identification register to verify
    /// that the correct sensor is connected and responding. The VL53L4CD should
    /// return a specific model ID value.
    ///
    /// # Returns
    ///
    /// * `Ok(u16)` - The sensor model ID (expected value for VL53L4CD)
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// let sensor_id = sensor.get_sensor_id().unwrap();
    /// println!("Sensor ID: 0x{:04X}", sensor_id);
    ///
    /// // Verify it's the correct sensor
    /// if sensor_id == 0xEACC { // Expected VL53L4CD model ID
    ///     println!("VL53L4CD sensor detected");
    /// } else {
    ///     println!("Unexpected sensor ID: 0x{:04X}", sensor_id);
    /// }
    /// ```
    pub async fn get_sensor_id(&mut self) -> Result<u16, Error<E>> {
        let id = self.read_word(Register::IdentificationModelId).await?;
        Ok(id)
    }

    /// Initializes the VL53L4CD sensor for operation.
    ///
    /// This function performs the complete sensor initialization sequence.
    ///
    /// **Important**: This function must be called before any ranging operations.
    /// The sensor will not function correctly without proper initialization.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the sensor was initialized successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::Timeout)` - If the sensor did not boot within 1 second
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// // Initialize the sensor
    /// sensor.sensor_init().unwrap();
    /// println!("Sensor initialized successfully");
    ///
    /// // Now the sensor is ready for ranging operations
    /// sensor.start_ranging().unwrap();
    /// ```
    pub async fn sensor_init(&mut self) -> Result<(), Error<E>> {
        const BOOT_STATUS: u8 = 0x3;
        let mut attempts = 0u16;

        info!("Waiting for sensor to boot");
        // Wait for sensor to boot
        loop {
            let status = self.read_byte(Register::FirmwareSystemStatus).await?;

            if status == BOOT_STATUS {
                break Ok(());
            }

            attempts += 1;
            if attempts >= 1000 {
                break Err(Error::Timeout);
            }

            self.delay.delay_ms(1).await;
        }?;

        // Load default configuration
        info!("Loading default configuration");
        for (i, &value) in VL53L4CD_DEFAULT_CONFIGURATION.iter().enumerate() {
            #[allow(clippy::cast_possible_truncation)]
            self.write_byte(i as u16 + 0x2D, value).await?;
        }

        // Start VHV
        info!("Starting VHV");
        self.write_byte(Register::SystemStart, 0x40).await?;

        // Wait for data ready
        info!("Waiting for data ready");
        let mut attempts = 0u16;
        loop {
            let status = self.check_for_data_ready().await?;
            if status {
                break Ok(());
            }

            attempts += 1;
            if attempts >= 1000 {
                break Err(Error::Timeout);
            }

            self.delay.delay_ms(1).await;
        }?;

        self.clear_interrupt().await?;
        self.stop_ranging().await?;
        self.write_byte(Register::VhvConfigTimeoutMacropLoopBound, 0x09)
            .await?;
        self.write_byte(0x0Bu16, 0x00).await?;
        self.write_word(0x0024u16, 0x500).await?;
        self.write_byte(0x81u16, 0b1000_1010).await?;
        self.write_byte(0x004Bu16, 0x03).await?;
        self.set_inter_measurement_in_ms(1000).await?;
        Ok(())
    }

    /// Checks if new ranging data is ready for retrieval.
    ///
    /// This function determines whether the sensor has completed a ranging measurement
    /// and new data is available. It checks the interrupt status by first determining
    /// the interrupt polarity configuration, then reading the actual interrupt status.
    ///
    /// **Note**: This function only checks the status - it does not clear the interrupt.
    /// Use [`clear_interrupt`](Self::clear_interrupt) after reading the data to reset
    /// the interrupt condition.
    ///
    /// # Returns
    ///
    /// * `Ok(true)` - New ranging data is available
    /// * `Ok(false)` - No new data available yet
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Start ranging
    /// sensor.start_ranging().unwrap();
    ///
    /// // Poll for data ready
    /// loop {
    ///     if sensor.check_for_data_ready().unwrap() {
    ///         let measurement = sensor.get_estimated_measurement().unwrap();
    ///         println!("Distance: {} mm", measurement.estimated_distance_mm);
    ///         sensor.clear_interrupt().unwrap();
    ///         break;
    ///     }
    /// }
    /// ```
    pub async fn check_for_data_ready(&mut self) -> Result<bool, Error<E>> {
        // first check the interrupt polarity
        let interrupt_polarity = self.read_byte(Register::GpioHvMuxCtrl).await?;
        let interrupt_polarity = u8::from(interrupt_polarity & 0x10 == 0);

        // then check the interrupt status
        let interrupt_status = self.read_byte(Register::GpioTioHvStatus).await?;
        if interrupt_status & 1 == interrupt_polarity {
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Clears the sensor interrupt flag.
    ///
    /// This function clears the interrupt condition by writing to the interrupt clear
    /// register. It should be called after reading measurement data to reset the
    /// interrupt state and prepare for the next measurement.
    ///
    /// **Important**: Always call this function after reading data when using
    /// interrupt-driven operation. Failure to clear the interrupt will prevent
    /// new interrupts from being generated.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the interrupt was cleared successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Check if data is ready
    /// if sensor.check_for_data_ready().unwrap() {
    ///     // Read the measurement data
    ///     let measurement = sensor.get_estimated_measurement().unwrap();
    ///     println!("Distance: {} mm", measurement.estimated_distance_mm);
    ///     
    ///     // Clear the interrupt to prepare for next measurement
    ///     sensor.clear_interrupt().unwrap();
    /// }
    /// ```
    pub async fn clear_interrupt(&mut self) -> Result<(), Error<E>> {
        self.write_byte(Register::SystemInterruptClear, 0x01).await
    }

    /// Starts a single-shot ranging measurement.
    ///
    /// This function initiates a single ranging measurement. The sensor will perform
    /// one complete ranging cycle and then automatically stop. This mode is useful
    /// for applications that need occasional distance measurements without continuous
    /// operation.
    ///
    /// **Note**: The sensor will automatically stop ranging after completing the measurement.
    /// No need to call [`stop_ranging`](Self::stop_ranging) for single-shot mode.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If ranging was started successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Start single-shot ranging
    /// sensor.start_ranging_single_shot().unwrap();
    ///
    /// // Wait for data to be ready
    /// loop {
    ///     if sensor.check_for_data_ready().unwrap() {
    ///         let measurement = sensor.get_estimated_measurement().unwrap();
    ///         println!("Single-shot distance: {} mm", measurement.estimated_distance_mm);
    ///         sensor.clear_interrupt().unwrap();
    ///         break;
    ///     }
    /// }
    /// ```
    pub async fn start_ranging_single_shot(&mut self) -> Result<(), Error<E>> {
        self.write_byte(Register::SystemStart, 0x10).await
    }

    /// Starts continuous ranging measurements.
    ///
    /// This function initiates continuous ranging mode where the sensor performs
    /// measurements continuously at the configured inter-measurement interval.
    /// The sensor will continue ranging until explicitly stopped with
    /// [`stop_ranging`](Self::stop_ranging).
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If ranging was started successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Start continuous ranging
    /// sensor.start_ranging().unwrap();
    ///
    /// // Continuous measurement loop
    /// for _ in 0..10 {
    ///     if sensor.check_for_data_ready().unwrap() {
    ///         let measurement = sensor.get_estimated_measurement().unwrap();
    ///         println!("Distance: {} mm", measurement.estimated_distance_mm);
    ///         sensor.clear_interrupt().unwrap();
    ///     }
    /// }
    ///
    /// // Stop ranging when done
    /// sensor.stop_ranging().unwrap();
    /// ```
    pub async fn start_ranging(&mut self) -> Result<(), Error<E>> {
        self.write_byte(Register::SystemStart, 0x40).await
    }

    /// Stops ranging measurements.
    ///
    /// This function stops the sensor from performing ranging measurements.
    /// It should be called when ranging is no longer needed to conserve power
    /// and prepare the sensor for low-power modes.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If ranging was stopped successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Start ranging
    /// sensor.start_ranging().unwrap();
    ///
    /// // Perform some measurements
    /// for _ in 0..5 {
    ///     if sensor.check_for_data_ready().unwrap() {
    ///         let measurement = sensor.get_estimated_measurement().unwrap();
    ///         println!("Distance: {} mm", measurement.estimated_distance_mm);
    ///         sensor.clear_interrupt().unwrap();
    ///     }
    /// }
    ///
    /// // Stop ranging when done
    /// sensor.stop_ranging().unwrap();
    /// ```
    pub async fn stop_ranging(&mut self) -> Result<(), Error<E>> {
        self.write_byte(Register::SystemStart, 0x00).await
    }

    /// Get the estimated measurement from the sensor.
    ///
    /// This function reads all the measurement data from the sensor's result registers
    /// and returns a comprehensive measurement result including distance, quality
    /// indicators, and environmental data.
    ///
    /// **Note**: This function should only be called after confirming that new data
    /// is available using [`check_for_data_ready`](Self::check_for_data_ready) or
    /// using an interrupt pin to detect when new data is available.
    ///
    /// # Returns
    ///
    /// * `Ok(EstimatedMeasurement)` - The complete measurement data
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// if sensor.check_for_data_ready().unwrap() {
    ///     let measurement = sensor.get_estimated_measurement().unwrap();
    ///     
    ///     if measurement.measurement_status == 0 {
    ///         println!("Distance: {} mm", measurement.estimated_distance_mm);
    ///         println!("Signal strength: {} kcps", measurement.signal_kcps);
    ///         println!("Ambient light: {} kcps", measurement.ambient_kcps);
    ///         println!("Measurement precision: ±{} mm", measurement.sigma_mm);
    ///     } else {
    ///         println!("Measurement failed with status: {}", measurement.measurement_status);
    ///     }
    ///     
    ///     sensor.clear_interrupt().unwrap();
    /// }
    /// ```
    pub async fn get_estimated_measurement(&mut self) -> Result<EstimatedMeasurement, Error<E>> {
        const STATUS_RTN: [u8; 24] = [
            255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255,
            255, 11, 12,
        ];

        let mut measurement_status = self.read_byte(Register::ResultRangeStatus).await? & 0x1f;
        if measurement_status < 24 {
            measurement_status = STATUS_RTN[measurement_status as usize];
        }
        let estimated_distance_mm = self.read_word(Register::ResultDistance).await?;
        let sigma_mm = self.read_word(Register::ResultSigma).await? / 4;
        let signal_kcps = self.read_word(Register::ResultSignalRate).await? * 8;
        let ambient_kcps = self.read_word(Register::ResultAmbientRate).await? * 8;
        Ok(EstimatedMeasurement {
            measurement_status,
            estimated_distance_mm,
            sigma_mm,
            signal_kcps,
            ambient_kcps,
        })
    }

    /// Sets the macro timing for ranging measurements.
    ///
    /// This function configures the timing parameters that control the duration
    /// and precision of ranging measurements. Higher values provide better
    /// precision but increase measurement time and power consumption.
    ///
    /// # Arguments
    ///
    /// * `macro_timing` - Macro timing value (1-255)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the timing was set successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::InvalidArgument)` - If the value is outside valid range
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Set macro timing for high precision
    /// sensor.set_macro_timing(200).unwrap();
    ///
    /// // Start ranging with new timing
    /// sensor.start_ranging().unwrap();
    /// ```
    pub async fn set_macro_timing(&mut self, macro_timing: u16) -> Result<(), Error<E>> {
        if !(1..=255).contains(&macro_timing) {
            error!("Invalid macro timing: {}", macro_timing);
            return Err(Error::InvalidArgument);
        }
        self.write_word(Register::RangeConfigA, macro_timing)
            .await?;
        self.write_word(Register::RangeConfigB, macro_timing + 1)
            .await?;
        Ok(())
    }

    /// Gets the current macro timing configuration.
    ///
    /// This function reads the current macro timing value from the sensor.
    /// The macro timing controls the duration and precision of ranging
    /// measurements, with higher values providing better precision but
    /// longer measurement times.
    ///
    /// # Returns
    ///
    /// * `Ok(u16)` - Current macro timing value (1-255)
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Get current macro timing
    /// let current_timing = sensor.get_macro_timing().unwrap();
    /// println!("Current macro timing: {}", current_timing);
    ///
    /// // Adjust timing if needed
    /// if current_timing < 100 {
    ///     println!("Timing is low, consider increasing for better precision");
    ///     sensor.set_macro_timing(150).unwrap();
    /// }
    /// ```
    pub async fn get_macro_timing(&mut self) -> Result<u16, Error<E>> {
        self.read_word(Register::RangeConfigA).await
    }

    /// Sets the inter-measurement period in milliseconds.
    ///
    /// This function configures the time interval between consecutive ranging
    /// measurements in continuous mode. The sensor will wait this duration
    /// after completing one measurement before starting the next.
    ///
    /// **Note**: This setting only affects continuous ranging mode. Single-shot
    /// mode ignores this setting and performs one measurement immediately.
    ///
    /// # Arguments
    ///
    /// * `inter_measurement_ms` - Time between measurements in milliseconds (1-65535)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the interval was set successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::InvalidArgument)` - If the value is outside valid range
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Set measurement interval to 100ms for high-frequency updates
    /// sensor.set_inter_measurement_in_ms(100).unwrap();
    ///
    /// // Start continuous ranging with 100ms interval
    /// sensor.start_ranging().unwrap();
    ///
    /// // Now measurements will occur every 100ms
    /// ```
    pub async fn set_inter_measurement_in_ms(
        &mut self,
        inter_measurement_ms: u32,
    ) -> Result<(), Error<E>> {
        if !(10..=60000).contains(&inter_measurement_ms) {
            error!("Invalid inter measurement in ms: {}", inter_measurement_ms);
            return Err(Error::InvalidArgument);
        }
        let inter_measurement_factor = 1.055f32;
        let clock_pll = self.read_word(Register::ResultOscCalibrateVal).await?;
        let clock_pll = clock_pll & 0x3FF;
        #[allow(
            clippy::cast_sign_loss,
            clippy::cast_possible_truncation,
            clippy::cast_precision_loss
        )]
        let inter_measurement_factor =
            inter_measurement_factor * inter_measurement_ms as f32 * f32::from(clock_pll);
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        self.write_dword(
            Register::IntermeasurementMs,
            inter_measurement_factor as u32,
        )
        .await?;
        Ok(())
    }

    /// Gets the current inter-measurement period.
    ///
    /// This function reads the current inter-measurement interval from the sensor.
    /// The inter-measurement period defines the time between consecutive ranging
    /// measurements in continuous mode, measured in milliseconds.
    ///
    /// # Returns
    ///
    /// * `Ok(u32)` - Current inter-measurement period in milliseconds
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Get current measurement interval
    /// let current_interval = sensor.get_inter_measurement_in_ms().unwrap();
    /// println!("Current measurement interval: {} ms", current_interval);
    ///
    /// // Adjust interval if needed for power optimization
    /// if current_interval < 500 {
    ///     println!("Interval is quite short, consider increasing for battery life");
    ///     sensor.set_inter_measurement_in_ms(1000).unwrap();
    /// }
    /// ```
    pub async fn get_inter_measurement_in_ms(&mut self) -> Result<u32, Error<E>> {
        let clock_pll_factor = 1.055f32;
        let inter_measurement_ms = self.read_dword(Register::IntermeasurementMs).await?;
        let clock_pll = self.read_word(Register::ResultOscCalibrateVal).await?;
        let clock_pll = clock_pll & 0x3FF;
        let clock_pll_factor = clock_pll_factor * f32::from(clock_pll);
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let clock_pll = clock_pll_factor as u32;
        let inter_measurement_ms = inter_measurement_ms / clock_pll;
        Ok(inter_measurement_ms)
    }

    /// Sets the Region of Interest (ROI) for the sensor.
    ///
    /// This function configures the number of SPADs (Single Photon Avalanche Diodes)
    /// used for ranging measurements. The ROI function allows some SPADs to be disabled,
    /// which affects both power consumption and maximum ranging distance.
    ///
    /// **Important Notes**:
    /// - Changing the SPAD number has **no effect** on the field of view.
    /// - The sensor defaults to 16x16 mode (maximum SPADs)
    /// - ST recommends changing SPAD number only if current consumption below 75 μA is desired
    /// - Using minimum ROI (4x4) typically reduces current consumption by -10 μA during ranging
    /// - Maximum ranging distance impact depends on reflectance, ambient light, and macroperiod
    /// - In some conditions, minimum ROI can reduce maximum ranging distance by up to -50%
    ///
    /// # Arguments
    ///
    /// * `roi_width` - The ROI width in pixels (4-16, where 4x4 = minimum, 16x16 = maximum)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the ROI was set successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error or invalid argument
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Set ROI to 4x4 for minimum power consumption (may reduce max range by up to 50%)
    /// sensor.set_roi(4).unwrap();
    ///
    /// // Or set to 8x8 for balanced power and range performance
    /// sensor.set_roi(8).unwrap();
    /// ```
    pub async fn set_roi(&mut self, roi_width: u8) -> Result<(), Error<E>> {
        if !(4..=16).contains(&roi_width) {
            error!("Invalid ROI width: {}", roi_width);
            return Err(Error::InvalidArgument);
        }
        let mut tmp = self.read_byte(0x013Eu16).await?;
        if roi_width > 10 {
            tmp = 199;
        }
        self.write_byte(0x007Fu16, tmp).await?;
        self.write_byte(0x0080u16, (roi_width - 1) << 4 | (roi_width - 1))
            .await?;
        Ok(())
    }

    /// Gets the current Region of Interest (ROI) width setting.
    ///
    /// This function reads the current ROI width configuration from the sensor.
    /// The returned value represents the ROI width in pixels, which affects the
    /// sensor's measurement area and field of view.
    ///
    /// # Returns
    ///
    /// * `Ok(u8)` - The current ROI width in pixels (4-16)
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Get current ROI width
    /// let roi_width = sensor.get_roi().unwrap();
    /// println!("Current ROI width: {} pixels", roi_width);
    /// ```
    pub async fn get_roi(&mut self) -> Result<u8, Error<E>> {
        let tmp = self.read_byte(0x0080u16).await?;
        Ok((tmp & 0x0F) + 1)
    }

    /// Sets the interrupt configuration and distance thresholds.
    ///
    /// This function configures the sensor's interrupt behavior and sets distance
    /// thresholds that determine when interrupts are generated. The interrupt can
    /// be configured to trigger on new sample ready, when distance is within a
    /// threshold window, or when distance is outside the threshold window.
    ///
    /// **Note**: The low threshold must be less than or equal to the high threshold
    /// for proper operation. Invalid threshold combinations may result in unexpected
    /// interrupt behavior.
    ///
    /// # Arguments
    ///
    /// * `low_threshold_mm` - Lower distance threshold in millimeters (0-4000)
    /// * `high_threshold_mm` - Upper distance threshold in millimeters (0-4000)
    /// * `interrupt_on` - Interrupt trigger condition (`InterruptOn`)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the interrupt configuration was set successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::{VL53L4cd, InterruptOn};
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Configure interrupt to trigger when distance is between 100-500mm
    /// sensor.set_interrupt_configuration(100, 500, InterruptOn::InWindow).unwrap();
    /// ```
    pub async fn set_interrupt_configuration(
        &mut self,
        low_threshold_mm: u16,
        high_threshold_mm: u16,
        interrupt_on: InterruptOn,
    ) -> Result<(), Error<E>> {
        self.write_byte(Register::SystemInterrupt, interrupt_on.into())
            .await?;
        self.write_word(Register::ThreshHigh, high_threshold_mm)
            .await?;
        self.write_word(Register::ThreshLow, low_threshold_mm)
            .await?;
        Ok(())
    }

    /// Gets the current interrupt configuration and threshold settings.
    ///
    /// This function reads the current interrupt configuration and high distance
    /// threshold from the sensor. It returns both the threshold value and the
    /// interrupt mode in a single call.
    ///
    /// # Returns
    ///
    /// * `Ok((u16, InterruptOn))` - Tuple of (`high_threshold_mm`, `interrupt_mode`)
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::{VL53L4cd, InterruptOn};
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Get current interrupt configuration and threshold
    /// let (threshold, mode) = sensor.get_interrupt_configuration().unwrap();
    ///
    /// println!("High threshold: {} mm", threshold);
    /// match mode {
    ///     InterruptOn::NewSampleReady => println!("Interrupt on new sample ready"),
    ///     InterruptOn::InWindow => println!("Interrupt when distance in threshold window"),
    ///     InterruptOn::OutOfWindow => println!("Interrupt when distance outside threshold window"),
    ///     _ => println!("Other interrupt mode: {:?}", mode),
    /// }
    /// ```
    pub async fn get_interrupt_configuration(&mut self) -> Result<(u16, InterruptOn), Error<E>> {
        let distance_threshold_mm = self.read_word(Register::ThreshHigh).await?;
        let interrupt_on = InterruptOn::from(self.read_byte(Register::SystemInterrupt).await?);
        Ok((distance_threshold_mm, interrupt_on))
    }

    /// Sets the signal rate threshold for ranging measurements.
    ///
    /// This function configures the minimum signal rate required for a valid
    /// ranging measurement. Measurements with signal rates below this threshold
    /// will be considered unreliable or invalid.
    ///
    /// **Note**: The signal threshold helps filter out weak or noisy measurements,
    /// improving overall ranging reliability at the cost of potentially missing
    /// some distant or low-reflectivity targets.
    ///
    /// # Arguments
    ///
    /// * `signal_kcps` - Minimum signal rate in kilocounts per second (0-65535)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the threshold was set successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Set signal threshold for reliable measurements
    /// sensor.set_signal_threshold(100).unwrap();  // 100 kcps minimum
    ///
    /// // Start ranging with signal threshold
    /// sensor.start_ranging().unwrap();
    /// ```
    pub async fn set_signal_threshold(&mut self, signal_kcps: u16) -> Result<(), Error<E>> {
        if !(1..=16384).contains(&signal_kcps) {
            error!("Invalid signal threshold: {}", signal_kcps);
            return Err(Error::InvalidArgument);
        }
        self.write_word(Register::MinCountRateRtnLimitMcps, signal_kcps >> 3)
            .await?;
        Ok(())
    }

    /// Gets the current signal rate threshold setting.
    ///
    /// This function reads the current signal rate threshold from the sensor.
    /// The signal threshold defines the minimum signal rate required for valid
    /// ranging measurements, helping filter out weak or noisy readings.
    ///
    /// # Returns
    ///
    /// * `Ok(u16)` - Current signal threshold in kilocounts per second
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Get current signal threshold
    /// let current_threshold = sensor.get_signal_threshold().unwrap();
    /// println!("Current signal threshold: {} kcps", current_threshold);
    ///
    /// // Adjust threshold if needed
    /// if current_threshold < 50 {
    ///     println!("Threshold is quite low, consider increasing for reliability");
    ///     sensor.set_signal_threshold(100).unwrap();
    /// }
    /// ```
    pub async fn get_signal_threshold(&mut self) -> Result<u16, Error<E>> {
        let signal_kcps = self.read_word(Register::MinCountRateRtnLimitMcps).await?;
        Ok(signal_kcps << 3)
    }

    /// Sets the sigma threshold for ranging measurements.
    ///
    /// This function configures the maximum acceptable measurement uncertainty
    /// (sigma) for valid ranging results. Measurements with sigma values above
    /// this threshold will be considered unreliable due to poor precision.
    ///
    /// **Note**: The sigma threshold helps filter out imprecise measurements,
    /// improving overall ranging accuracy at the cost of potentially rejecting
    /// some valid but noisy measurements.
    ///
    /// # Arguments
    ///
    /// * `sigma_mm` - Maximum acceptable sigma value in millimeters (0-65535)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the threshold was set successfully
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Set sigma threshold for high-precision measurements
    /// sensor.set_sigma_threshold(50).unwrap();  // 50mm maximum uncertainty
    ///
    /// // Start ranging with precision threshold
    /// sensor.start_ranging().unwrap();
    /// ```
    pub async fn set_sigma_threshold(&mut self, sigma_mm: u16) -> Result<(), Error<E>> {
        if sigma_mm > 0xFFFF >> 2 {
            error!("Invalid sigma threshold: {}", sigma_mm);
            return Err(Error::InvalidArgument);
        }
        self.write_word(Register::RangeConfigSigmaThresh, sigma_mm << 2)
            .await?;
        Ok(())
    }

    /// Gets the current sigma threshold setting.
    ///
    /// This function reads the current sigma threshold from the sensor.
    /// The sigma threshold defines the maximum acceptable measurement uncertainty
    /// for valid ranging results, helping filter out imprecise measurements.
    ///
    /// # Returns
    ///
    /// * `Ok(u16)` - Current sigma threshold in millimeters
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::VL53L4cd;
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Get current sigma threshold
    /// let current_threshold = sensor.get_sigma_threshold().unwrap();
    /// println!("Current sigma threshold: {} mm", current_threshold);
    ///
    /// // Adjust threshold if needed
    /// if current_threshold > 100 {
    ///     println!("Threshold is quite high, consider decreasing for precision");
    ///     sensor.set_sigma_threshold(50).unwrap();
    /// }
    /// ```
    pub async fn get_sigma_threshold(&mut self) -> Result<u16, Error<E>> {
        let sigma_mm = self.read_word(Register::RangeConfigSigmaThresh).await?;
        Ok(sigma_mm >> 2)
    }

    /// Writes a single byte to a sensor register.
    ///
    /// This is a low-level function that writes an 8-bit value to a specific
    /// register address on the sensor. The function is generic over the register
    /// address type, accepting any type that can be converted to `u16`.
    ///
    /// **Note**: This function performs direct I2C communication with the sensor.
    /// Most applications should use the higher-level configuration functions
    /// instead of calling this directly.
    ///
    /// # Arguments
    ///
    /// * `register_address` - The register address to write to (implements `Into<u16>`)
    /// * `value` - The 8-bit value to write to the register
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the write operation was successful
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::{VL53L4cd, Register};
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Write to a specific register using the Register enum
    /// sensor.write_byte(Register::SystemInterrupt, 0x01).unwrap();
    ///
    /// // Or write to a hardcoded address
    /// sensor.write_byte(0x0046u16, 0x01).unwrap();
    /// ```
    pub async fn write_byte<R>(&mut self, register_address: R, value: u8) -> Result<(), Error<E>>
    where
        R: Into<u16>,
    {
        let reg: u16 = register_address.into();
        let mut buffer = [0u8; 3];
        buffer[0] = (reg >> 8) as u8;
        buffer[1] = (reg & 0xff) as u8;
        buffer[2] = value;
        self.i2c.write(self.address, &buffer).await?;
        Ok(())
    }

    /// Reads a single byte from a sensor register.
    ///
    /// This is a low-level function that reads an 8-bit value from a specific
    /// register address on the sensor. The function is generic over the register
    /// address type, accepting any type that can be converted to `u16`.
    ///
    /// **Note**: This function performs direct I2C communication with the sensor.
    /// Most applications should use the higher-level data reading functions
    /// instead of calling this directly.
    ///
    /// # Arguments
    ///
    /// * `register_address` - The register address to read from (implements `Into<u16>`)
    ///
    /// # Returns
    ///
    /// * `Ok(u8)` - The 8-bit value read from the register
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::{VL53L4cd, Register};
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Read from a specific register using the Register enum
    /// let status = sensor.read_byte(Register::SystemInterrupt).unwrap();
    /// println!("System interrupt status: 0x{:02X}", status);
    ///
    /// // Or read from a hardcoded address
    /// let value = sensor.read_byte(0x0046u16).unwrap();
    /// ```
    pub async fn read_byte<R>(&mut self, register_address: R) -> Result<u8, Error<E>>
    where
        R: Into<u16>,
    {
        let reg: u16 = register_address.into();
        let write_buffer = reg.to_be_bytes();
        let mut read_buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &write_buffer, &mut read_buffer)
            .await?;
        Ok(read_buffer[0])
    }

    /// Writes a 16-bit word to a sensor register.
    ///
    /// This is a low-level function that writes a 16-bit value to a specific
    /// register address on the sensor. The function is generic over the register
    /// address type, accepting any type that can be converted to `u16`.
    ///
    /// **Note**: This function performs direct I2C communication with the sensor.
    /// Most applications should use the higher-level configuration functions
    /// instead of calling this directly.
    ///
    /// # Arguments
    ///
    /// * `register_address` - The register address to write to (implements `Into<u16>`)
    /// * `value` - The 16-bit value to write to the register
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the write operation was successful
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::{VL53L4cd, Register};
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Write a 16-bit value to a specific register using the Register enum
    /// sensor.write_word(Register::RangeConfigA, 0x0123).unwrap();
    ///
    /// // Or write to a hardcoded address
    /// sensor.write_word(0x005Eu16, 0x0123).unwrap();
    /// ```
    pub async fn write_word<R>(&mut self, register_address: R, value: u16) -> Result<(), Error<E>>
    where
        R: Into<u16>,
    {
        let reg: u16 = register_address.into();
        let mut buffer = [0u8; 4];
        buffer[0..2].copy_from_slice(&reg.to_be_bytes());
        buffer[2..4].copy_from_slice(&value.to_be_bytes());
        self.i2c.write(self.address, &buffer).await?;
        Ok(())
    }

    /// Reads a 16-bit word from a sensor register.
    ///
    /// This is a low-level function that reads a 16-bit value from a specific
    /// register address on the sensor. The function is generic over the register
    /// address type, accepting any type that can be converted to `u16`.
    ///
    /// **Note**: This function performs direct I2C communication with the sensor.
    /// Most applications should use the higher-level data reading functions
    /// instead of calling this directly.
    ///
    /// # Arguments
    ///
    /// * `register_address` - The register address to read from (implements `Into<u16>`)
    ///
    /// # Returns
    ///
    /// * `Ok(u16)` - The 16-bit value read from the register
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::{VL53L4cd, Register};
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Read a 16-bit value from a specific register using the Register enum
    /// let config = sensor.read_word(Register::RangeConfigA).unwrap();
    /// println!("Range config A: 0x{:04X}", config);
    ///
    /// // Or read from a hardcoded address
    /// let value = sensor.read_word(0x005Eu16).unwrap();
    /// ```
    pub async fn read_word<R>(&mut self, register_address: R) -> Result<u16, Error<E>>
    where
        R: Into<u16>,
    {
        let reg: u16 = register_address.into();
        let write_buffer = reg.to_be_bytes();
        let mut read_buffer = [0u8; 2];
        self.i2c
            .write_read(self.address, &write_buffer, &mut read_buffer)
            .await?;
        Ok(u16::from_be_bytes(read_buffer))
    }

    /// Writes a 32-bit double word to a sensor register.
    ///
    /// This is a low-level function that writes a 32-bit value to a specific
    /// register address on the sensor. The function is generic over the register
    /// address type, accepting any type that can be converted to `u16`.
    ///
    /// **Note**: This function performs direct I2C communication with the sensor.
    /// Most applications should use the higher-level configuration functions
    /// instead of calling this directly.
    ///
    /// # Arguments
    ///
    /// * `register_address` - The register address to write to (implements `Into<u16>`)
    /// * `value` - The 32-bit value to write to the register
    ///
    /// # Returns
    ///
    /// * `Ok(())` - If the write operation was successful
    ///
    /// # Errors
    ///
    /// * `Err(Error::I2cError(E))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::{VL53L4cd, Register};
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Write a 32-bit value to a specific register using the Register enum
    /// sensor.write_dword(Register::IntermeasurementMs, 1000).unwrap();
    ///
    /// // Or write to a hardcoded address
    /// sensor.write_dword(0x006Cu16, 1000).unwrap();
    /// ```
    pub async fn write_dword<R>(&mut self, register_address: R, value: u32) -> Result<(), Error<E>>
    where
        R: Into<u16>,
    {
        let reg: u16 = register_address.into();
        let mut buffer = [0u8; 6];
        buffer[0..2].copy_from_slice(&reg.to_be_bytes());
        buffer[2..6].copy_from_slice(&value.to_be_bytes());
        self.i2c.write(self.address, &buffer).await?;
        Ok(())
    }

    /// Reads a 32-bit double word from a sensor register.
    ///
    /// This is a low-level function that reads a 32-bit value from a specific
    /// register address on the sensor. The function is generic over the register
    /// address type, accepting any type that can be converted to `u16`.
    ///
    /// **Note**: This function performs direct I2C communication with the sensor.
    /// Most applications should use the higher-level data reading functions
    /// instead of calling this directly.
    ///
    /// # Arguments
    ///
    /// * `register_address` - The register address to read from (implements `Into<u16>`)
    ///
    /// # Returns
    ///
    /// * `Ok(u32)` - The 32-bit value read from the register
    ///
    /// # Errors
    ///
    /// * `Err(Error<E>::I2cError(e))` - If there was an I2C communication error
    ///
    /// # Examples
    ///
    /// ```rust,no_run
    /// use vl53l4cd_ulp::{VL53L4cd, Register};
    ///
    /// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
    /// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
    /// let mut sensor = VL53L4cd::new(i2c, delay);
    ///
    /// sensor.sensor_init().unwrap();
    /// // Read a 32-bit value from a specific register using the Register enum
    /// let interval = sensor.read_dword(Register::IntermeasurementMs).unwrap();
    /// println!("Inter-measurement interval: {} ms", interval);
    ///
    /// // Or read from a hardcoded address
    /// let value = sensor.read_dword(0x006Cu16).unwrap();
    /// ```
    pub async fn read_dword<R>(&mut self, register_address: R) -> Result<u32, Error<E>>
    where
        R: Into<u16>,
    {
        let reg: u16 = register_address.into();
        let write_buffer = reg.to_be_bytes();
        let mut read_buffer = [0u8; 4];
        self.i2c
            .write_read(self.address, &write_buffer, &mut read_buffer)
            .await?;
        Ok(u32::from_be_bytes(read_buffer))
    }
}

/// Error type for VL53L4CD sensor operations.
///
/// This enum represents all possible error conditions that can occur during
/// sensor initialization, configuration, and ranging operations.
///
/// # Examples
///
/// ```rust,no_run
/// use vl53l4cd_ulp::Error;
///
/// let i2c = embedded_hal_mock::eh1::i2c::Mock::new(&[]);
/// let delay = embedded_hal_mock::eh1::delay::NoopDelay;
/// let mut sensor = vl53l4cd_ulp::VL53L4cd::new(i2c, delay);
///
/// match sensor.sensor_init() {
///     Ok(()) => println!("Sensor initialized successfully"),
///     Err(Error::Timeout) => println!("Sensor initialization timed out"),
///     Err(Error::InvalidArgument) => println!("Invalid parameter provided"),
///     Err(Error::I2cError(e)) => println!("I2C communication error: {:?}", e),
/// }
/// ```
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E: core::fmt::Debug> {
    /// I2C communication error from the underlying hardware
    I2cError(E),
    /// Sensor operation timed out
    Timeout,
    /// Invalid parameter value provided
    InvalidArgument,
}

impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl<E: core::fmt::Debug> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::I2cError(error)
    }
}
