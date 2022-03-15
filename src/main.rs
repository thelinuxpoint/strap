// #![no_std]
// #![no_main]

// use panic_halt;
// use cortex_m::{asm::delay, peripheral::DWT};
// use cortex_m_rt::entry; // The runtime
// use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
// // Hardware abstract Layer
// use stm32f1xx_hal::{delay::Delay, pac, prelude::*}; // STM32F1 specific functions
// use stm32f1xx_hal::usb::{Peripheral, UsbBus};
// use stm32f1xx_hal::pac::{Interrupt};
// use stm32f1xx_hal::{stm32};
// use stm32f1xx_hal::pac::interrupt;
// use stm32f1xx_hal::gpio::*;
// use stm32f1xx_hal::pac::USART1;
// use stm32f1xx_hal::serial::Pins;
// use stm32f1xx_hal::gpio::gpioa::{PA9,PA10};
// use stm32f1xx_hal::usb::UsbBusType;
// use stm32f1xx_hal::serial::{Config,Tx,Rx, Serial};

// use usb_device::bus;
// use usb_device::prelude::*;

// use systick_monotonic::{fugit::Duration,fugit::ExtU32,fugit::ExtU64, Systick};
// use core::panic::PanicInfo;
// use core::mem::MaybeUninit;
// // RTIC
// use rtic::app;
// use rtic;
// // use rtt_target::{rprintln, rtt_init_print};
// use core::fmt::Write;
// use strap::behave::mouse::*;

// const PERIOD: u32 = 8_000_000;
// type LED = gpioc::PC13<Output<PushPull>>;

// #[rtic::app(device = stm32f1xx_hal::pac, peripherals = true,dispatchers = [TIM2])]
// mod app {
//     use super::*;
//     // shared 
//     #[shared]
//     struct Shared {
//         #[lock_free]
//         counter: u8,
//         // the blinker
//         #[lock_free]
//         led: gpioc::PC13<Output<PushPull>>,
//         //
//         #[lock_free]
//         usb_dev: UsbDevice<'static, UsbBusType>,
//         //
//         #[lock_free]
//         hid: HIDClass<'static, UsbBusType>}
//     // local
//     #[local]
//     struct Local {
        
//     }
    
//     #[monotonic(binds = SysTick, default = true)]
//     type MonoTimer = Systick<1000>;

//     #[init]
//     fn init(mut cx: init::Context) -> (Shared, Local, init::Monotonics) {

//         let mut rcc = cx.device.RCC.constrain();

//         cx.core.DCB.enable_trace();
//         DWT::unlock();
//         cx.core.DWT.enable_cycle_counter();

//         static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;
        
//         let mono = Systick::new(cx.core.SYST, 48_000_000);
//         /* acess the GPIO'S*/
//         let mut gpioa = cx.device.GPIOA.split(&mut rcc.apb2);
//         let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);
//         let mut gpioc = cx.device.GPIOC.split(&mut rcc.apb2);
//         /* */
//         let mut afio = cx.device.AFIO.constrain(&mut rcc.apb2);
//         let mut flash = cx.device.FLASH.constrain();
        
//         let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
//         //
//         let clocks = rcc
//             .cfgr
//             .use_hse(8.mhz())
//             .sysclk(48.mhz())
//             .pclk1(24.mhz())
//             .freeze(&mut flash.acr);
//         // usb plugs
//         let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
//         usb_dp.set_low().ok();
//         delay(clocks.sysclk().0 / 1000);

//         let usb_dm = gpioa.pa11;
//         let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

//         //
//         let usb = Peripheral {
//             usb: cx.device.USB,
//             pin_dm: usb_dm,
//             pin_dp: usb_dp,
//         };

//         unsafe{
//             USB_BUS = Some(UsbBus::new(usb));
//         }
      
//         let hid = HIDClass::new(unsafe{USB_BUS.as_ref().unwrap()});

//         let usb_dev = UsbDeviceBuilder::new(unsafe{USB_BUS.as_ref().unwrap()}, UsbVidPid(0x16c0, 0x27dd))
//             .manufacturer("Fake company")
//             .product("mouse")
//             .serial_number("TEsdkas98ds9adjsmadsa")
//             .device_class(0x03)
//             .build();
        


//         on_tick::spawn_after(1000_u64.millis());

//         //########################## RETURN ############################
//         (
//             Shared {
//                 counter: 0,
//                 led,
//                 usb_dev,
//                 hid
//             },
//             Local {},

//             init::Monotonics(mono),
//         )

//     }

//     //
//     #[task(shared = [counter, led, hid])]
//     fn on_tick(mut cx: on_tick::Context) {

//         let counter: &mut u8 = &mut cx.shared.counter;
//         let led = &mut cx.shared.led;
//         let hid = &mut cx.shared.hid;

//         const P: u8 = 2;
//         *counter = (*counter + 1) % P;

//         // move mouse cursor horizontally (x-axis) while blinking LED
//         if *counter < P / 2 {
//             led.set_high().ok();
//             hid.write(&report(100, 0));
//         } else {
//             led.set_low().ok();
//             hid.write(&report(-100, 0));
//         }
//         on_tick::spawn_after(500_u64.millis());
//     }
//     //
//     #[task(binds=USB_HP_CAN_TX, shared = [counter, led, usb_dev, hid])]
//     fn usb_tx(mut cx: usb_tx::Context) {
//         usb_poll(
//             &mut cx.shared.counter,
//             &mut cx.shared.led,
//             &mut cx.shared.usb_dev,
//             &mut cx.shared.hid,
//         );
//     }

//     #[task(binds=USB_LP_CAN_RX0, shared = [counter, led, usb_dev, hid])]
//     fn usb_rx(mut cx: usb_rx::Context) {
//         usb_poll(
//             &mut cx.shared.counter,
//             &mut cx.shared.led,
//             &mut cx.shared.usb_dev,
//             &mut cx.shared.hid,
//         );
//     }

// }

// fn usb_poll<B: bus::UsbBus>(_c: &mut u8,_l: &mut LED,usb_dev: &mut UsbDevice<'static, B>,hid: &mut HIDClass<'static, B>) {
//     if !usb_dev.poll(&mut [hid]) {

//         return;
//     }
// }

#![no_std]
#![no_main]

// extern crate panic_semihosting;
use panic_halt;

use cortex_m::asm::{delay, wfi};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use stm32f1xx_hal::stm32::{interrupt, Interrupt};
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use stm32f1xx_hal::{prelude::*, stm32};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;
static mut USB_SERIAL: Option<usbd_serial::SerialPort<UsbBusType>> = None;
static mut USB_DEVICE: Option<UsbDevice<UsbBusType>> = None;

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .freeze(&mut flash.acr);

    assert!(clocks.usbclk_valid());

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    // BluePill board has a pull-up resistor on the D+ line.
    // Pull the D+ pin down to send a RESET condition to the USB bus.
    // This forced reset is needed only for development, without it host
    // will not reset your device when you upload new firmware.
    let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
    usb_dp.set_low();
    delay(clocks.sysclk().0 / 100);

    let usb_dm = gpioa.pa11;
    let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

    let usb = Peripheral {
        usb: dp.USB,
        pin_dm: usb_dm,
        pin_dp: usb_dp,
    };

    // Unsafe to allow access to static variables
    unsafe {
        let bus = UsbBus::new(usb);

        USB_BUS = Some(bus);

        USB_SERIAL = Some(SerialPort::new(USB_BUS.as_ref().unwrap()));

        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(USB_CLASS_CDC)
            .build();

        USB_DEVICE = Some(usb_dev);
    }

    let mut nvic = p.NVIC;

    nvic.enable(Interrupt::USB_HP_CAN_TX);
    nvic.enable(Interrupt::USB_LP_CAN_RX0);

    loop {
        wfi();
    }
}

#[interrupt]
fn USB_HP_CAN_TX() {
    usb_interrupt();
}

#[interrupt]
fn USB_LP_CAN_RX0() {
    usb_interrupt();
}

fn usb_interrupt() {
    let usb_dev = unsafe { USB_DEVICE.as_mut().unwrap() };
    let serial = unsafe { USB_SERIAL.as_mut().unwrap() };

    if !usb_dev.poll(&mut [serial]) {
        return;
    }

    let mut buf = [0u8; 8];

    match serial.read(&mut buf) {
        Ok(count) if count > 0 => {
            // Echo back in upper case
            for c in buf[0..count].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }

            serial.write(&buf[0..count]).ok();
        }
        _ => {}
    }
}
