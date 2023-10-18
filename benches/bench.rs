use ak09915_rs::{Ak09915, Mode, Register};
use criterion::{criterion_group, criterion_main, Criterion};
use linux_embedded_hal::I2cdev;

fn new() -> Ak09915<I2cdev> {
    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    Ak09915::new(dev)
}

fn ak09915(c: &mut Criterion) {
    #[macro_export]
    macro_rules! bench {
        ($bench_fn:ident($($arg:tt)*)) => {
            let mut sensor = new();
            sensor.set_mode(Mode::Cont200Hz).unwrap();
            c.bench_function(stringify!($bench_fn), |b| b.iter(|| sensor.$bench_fn($($arg)*).expect("Error during benchmark")));
        }}

    c.bench_function("new", |b| b.iter(|| new()));

    bench!(read_register(Register::ST1));
    bench!(read());
}

criterion_group!(benches, ak09915);
criterion_main!(benches);
