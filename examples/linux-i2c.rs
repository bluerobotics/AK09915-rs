use ak09915_rs::Ak09915;
use ak09915_rs::Mode;
use clap::Parser;
use linux_embedded_hal::I2cdev;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Receive the i2c device as parameter
    #[arg(short, long, default_value = "/dev/i2c-1")]
    device: String,
}

fn main() {
    let args = Args::parse();
    let dev = I2cdev::new(args.device).unwrap();
    let mut sensor = Ak09915::new(dev);

    if sensor.self_test().unwrap() {
        println!("Self test -  OK");
    }

    println!("Test 5 single measurement");
    for _n in 1..=5 {
        sensor.set_mode(Mode::Single).unwrap();
        let (x, y, z) = sensor.read().unwrap();
        println!("Magnetometer: x={}, y={}, z={}", x, y, z);
    }
    println!("Test 5 measurement, without set single measurement(no updates)");
    for _n in 1..=5 {
        let (x, y, z) = sensor.read_unchecked().unwrap();
        println!("Magnetometer: x={}, y={}, z={}", x, y, z);
    }
    println!("Test 5 measurement, using continuous mode");
    sensor.set_mode(Mode::Cont200Hz).unwrap();
    for _n in 1..=5 {
        let (x, y, z) = sensor.read().unwrap();
        println!("Magnetometer: x={}, y={}, z={}", x, y, z);
        std::thread::sleep(std::time::Duration::from_secs(1 / 200));
    }
}
