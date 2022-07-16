#![no_std]
#![no_main]

extern crate panic_halt; // need this row
use cortex_m_rt::entry;
use stm32f1::stm32f103;

// use `main` as the entry point of this application
#[entry]
fn main() -> ! {
    // get handles to the hardware
    let peripherals = stm32f103::Peripherals::take().unwrap();
    let gpioc = &peripherals.GPIOC;
    // reset and clock control
    let rcc = &peripherals.RCC;

    // enable the GPIO clock for IO port C
    rcc.apb2enr.write(|w| w.iopcen().set_bit());
    
    gpioc.crh.write(|w| unsafe{
        w.mode13().bits(0b11);
        w.cnf13().bits(0b00)
    });

    loop {
        gpioc.bsrr.write(|w| w.bs13().set_bit());
        cortex_m::asm::delay(1_000_000);
        gpioc.brr.write(|w| w.br13().set_bit());
        cortex_m::asm::delay(1_000_000);
    }
}