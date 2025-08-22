# vl53l4cd-ulp

[![Crates.io](https://img.shields.io/crates/v/vl53l4cd-ulp.svg)](https://crates.io/crates/vl53l4cd-ulp)
[![Documentation](https://docs.rs/vl53l4cd-ulp/badge.svg)](https://docs.rs/vl53l4cd-ulp)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](README.md)

An ultra-low-power Rust driver for the VL53L4CD time-of-flight distance sensor. This crate provides an async, `no_std` driver that integrates with the `embedded-hal-async` ecosystem.

## Getting Started

Add the dependency to your `Cargo.toml`:

```toml
[dependencies]
vl53l4cd-ulp = "0.0.0"
```

### Basic Usage

```rust
use vl53l4cd_ulp::VL53L4cd;

let i2c = /* your I2C implementation */;
let delay = /* your delay implementation */;

let mut sensor = VL53L4cd::new(i2c, delay);
sensor.sensor_init().unwrap();
sensor.start_ranging().unwrap();

if sensor.check_for_data_ready().unwrap() {
    let measurement = sensor.get_estimated_measurement().unwrap();
    println!("Distance: {} mm", measurement.estimated_distance_mm);
    sensor.clear_interrupt().unwrap();
}
```

### Power Optimization

```rust
// Use minimum ROI for lowest power consumption
sensor.set_roi(4).unwrap();

// Set longer measurement intervals
sensor.set_inter_measurement_in_ms(1000).unwrap();

// Configure interrupt thresholds
sensor.set_interrupt_configuration(
    100, 500, InterruptOn::InWindow
).unwrap();
```

## Crate Features

The crate can be compiled with the following features:

- `async`: Enables async I²C support
- `log`: Enables logging via the `log` crate
- `defmt`: Enables logging via the `defmt` crate

## Error Handling

```rust
use vl53l4cd_ulp::Error;

match sensor.sensor_init() {
    Ok(()) => println!("Sensor initialized"),
    Err(Error::Timeout) => println!("Initialization timed out"),
    Err(Error::I2cError(e)) => println!("I2C error: {:?}", e),
    _ => println!("Other error"),
}
```

## License

Licensed under either of

- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or
  <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or
  <http://opensource.org/licenses/MIT>)

at your option.

## References

- [STSW-IMG034 Ultra-Low-Power Driver](https://www.st.com/en/embedded-software/stsw-img034.html)
- [VL53L4CD Datasheet](https://www.st.com/resource/en/datasheet/vl53l4cd.pdf)
