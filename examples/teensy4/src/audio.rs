//! Demonstrate a USB serial device
//!
//! Flash your Teensy 4 with this example. Then, connect a serial
//! interface to the USB device. You should see all inputs echoed
//! back to you.
//!
//! This example also supports debug logs over UART2, using pins
//! 14 and 15.

#![no_std]
#![no_main]

#![feature(alloc_error_handler)]

#[macro_use]
extern crate alloc;

//use imxrt_hal as hal;
//use teensy4_pins as pins;

use support::bsp::t41;
use support::hal;

use usb_device::prelude::*;
use usb_device::class_prelude::*;
use alloc_cortex_m::CortexMHeap;

const UART_BAUD: u32 = 115_200;
const GPT_OCR: hal::gpt::OutputCompareRegister = hal::gpt::OutputCompareRegister::One;
const BLINK_PERIOD: core::time::Duration = core::time::Duration::from_millis(500);

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
static mut HEAP: [u8; 128*1024] = [0; 128*1024];

struct TestClass<'a, B: usb_device::bus::UsbBus> {
    //bus: &'a B,
    cnt: usize,
    iface: usb_device::class_prelude::InterfaceNumber,
    iface2: InterfaceNumber,
    data_ep: usb_device::endpoint::EndpointIn<'a, B>,
}

impl<B: usb_device::bus::UsbBus> TestClass<'_, B> {
    fn new(alloc: &usb_device::bus::UsbBusAllocator<B>, max_packet_size: u16) -> TestClass<'_, B> {
        TestClass {
            cnt: 0,
            iface: alloc.interface(),
            iface2: alloc.interface(),
            data_ep: alloc.isochronous(max_packet_size, 1),
        }
    }
}

impl<'a, B: usb_device::bus::UsbBus> usb_device::class::UsbClass<B> for TestClass<'a, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.iad(self.iface, 1, 0xff, 0, 0x0)?;
        writer.interface(self.iface, 0xff, 0, 0x0)?;
        writer.endpoint(&self.data_ep)?;

        Ok(())
    }

    fn control_in(&mut self, xfer: ControlIn<'_, '_, '_, B>) {
        let req = xfer.request();
        log::info!("control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}", req.direction, req.request_type, req.recipient, req.request, req.value, req.index, req.length);
        /*if req.recipient == usb_device::control::Recipient::Interface && req.value == 256 && req.request == 2 && req.index == 514 {
            xfer.accept_with(&[
                0x1, 0x0, // 1 subrange
                0x44, 0xac, 0x00, 0x00, // MIN
                0x44, 0xac, 0x00, 0x00, // MAX
                0x0, 0x0, 0x0, 0x0, // RES
            ]).expect("Couldn't accept transfer!");
        } else if req.recipient == usb_device::control::Recipient::Interface && req.value == 256 && req.request == 1 && req.index == 514 {
            xfer.accept_with(&[
                0x44, 0xac, 0x00, 0x00, // CUR
            ]).expect("Couldn't accept transfer!");
        }*/
    }

    fn control_out(&mut self, xfer: ControlOut<'_, '_, '_, B>) {
        let req = xfer.request();
        log::info!("control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}", req.direction, req.request_type, req.recipient, req.request, req.value, req.index, req.length);
        /*if req.recipient == usb_device::control::Recipient::Interface && req.request == 11 && req.index == 3 {
            xfer.accept().expect("Couldn't accept");
        }*/
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        /*if self.data_ep.address().index() == addr.index() {
            let mut buf = [0; 768];
            let nbytes = match self.data_ep.read(&mut buf) {
                Ok(x) => x,
                Err(e) => {
                    log::error!("Received error {:?}", e);
                    return;
                }
            };
            log::info!("RX'd data: EP{}", addr.index());
        }*/
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr.index() == 1 {
            self.cnt += 1;
            //self.data_ep.write(&[5; 64]);
        }
    }
}

#[derive(PartialEq, Debug)]
enum ControlType {
    Current,
    Range,
    Memory,
}

impl ControlType {
    fn from(raw: u8) -> ControlType {
        match raw {
            0x1 => ControlType::Current,
            0x2 => ControlType::Range,
            0x3 => ControlType::Memory,
            _ => {
                panic!("Unrecognized raw type");
            }
        }
    }
}

struct UsbAudio<'a, B: usb_device::bus::UsbBus> {
    //bus: &'a B,
    nbytes: usize,
    iface: usb_device::class_prelude::InterfaceNumber,
    iface2: InterfaceNumber,
    data_ep: usb_device::endpoint::EndpointOut<'a, B>,
}

impl<B: usb_device::bus::UsbBus> UsbAudio<'_, B> {
    fn new(alloc: &usb_device::bus::UsbBusAllocator<B>, max_packet_size: u16) -> UsbAudio<'_, B> {
        UsbAudio {
            nbytes: 0,
            iface: alloc.interface(),
            iface2: alloc.interface(),
            data_ep: alloc.isochronous(max_packet_size, 1),
        }
    }
}

impl<'a, B: usb_device::bus::UsbBus> usb_device::class::UsbClass<B> for UsbAudio<'a, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut usb_device::descriptor::DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.iad(self.iface, 2, 1 /* AUDIO */, 0 /* CONTROL */, 0x20)?;
        writer.interface(self.iface, 1 /* AUDIO */, 1 /* CONTROL */, 0x20)?;
        let CS_INTERFACE = 0x24;
        // Setup the audio control
        writer.write(
            CS_INTERFACE,
            &[
                1, // Header
                0x00, 0x00, // BCD ADC
                0xff, // Category
                0x09, 0x00, // Total length
                0,    // Latency controls
            ],
        )?;
        writer.write(
            CS_INTERFACE,
            &[
                2, // Input terminal
                1, // Terminal ID (unique)
                0x01, 0x01, // Terminal type
                0,    // bAssocTerminal
                2,    // bCSourceID
                2,    // bNrChannels
                0x3, 0, 0, 0, // bmChannelConfig (front left, front right)
                0, // iChannelNames
                0, 0, // bmControls
                0, // iTerminal
            ],
        )?;
        writer.write(
            CS_INTERFACE,
            &[
                0xA, // CLOCK_SOURCE
                2, // clock ID
                0, // bmAttributes (external, not sync'd to SOF)
                0, // bmControls (no controls present)
                1, // bAssocTerminal
                0, // iClockSource
            ],
        )?;

        // Setup the audio streaming
        //writer.iad(self.iface2, 1, 1 /* AUDIO */, 2 /* STREAMING */, 0)?;
        writer.interface(self.iface2, 1 /* AUDIO */, 2 /* STREAMING */, 0x20)?;
        writer.write(
            CS_INTERFACE,
            &[
                1, // AS_GENERAL
                1, // bTerminalLink (terminal ID of output term)
                0, // bmControls
                1, // bFormatType (PCM)
                0x1, 0x0, 0x0, 0x0, // bmFormats (PCM)
                2,   // bNrChannels
                0x3, 0, 0, 0, // bmChannelConfig
                0, // iChannelNames
            ],
        )?;
        writer.write(
            CS_INTERFACE,
            &[
                0x02, // FORMAT_TYPE
                0x01, // FORMAT_TYPE_I
                1, // bSubslotSize (1 byte samples)
                8, // bBitResolution (8 bits in byte used)
            ]
        )?;

        writer.endpoint(&self.data_ep)?;

        // Class specific endpoint descriptor
        writer.write(
            0x25, // CS_ENDPOINT
            &[
                1, // EP_GENERAL
                0, // bmAttributes
                0, // bmControls
                0, // bLockDelayUnits
                0, 0, // wLockDelay
            ]
        )?;

        Ok(())
    }

    fn control_in(&mut self, xfer: ControlIn<'_, '_, '_, B>) {
        let req = xfer.request();
        if req.request_type == usb_device::control::RequestType::Class && req.recipient == usb_device::control::Recipient::Interface {
            let reqtype = ControlType::from(req.request);
            let control_selector = req.value >> 8;
            let channel_number = req.value & 0xFF;
            let interface = req.index & 0xFF;
            let entity_id = req.index >> 8;

            log::info!("audio GET: {:?} CS={} chan={} iface={} eid={}", reqtype, control_selector, channel_number, interface, entity_id);
            //log::info!("control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}", req.direction, req.request_type, req.recipient, req.request, req.value, req.index, req.length);
            //if req.recipient == usb_device::control::Recipient::Interface && req.value == 256 && req.request == 2 && req.index == 514 {
            if reqtype == ControlType::Range && entity_id == 2 {
                xfer.accept_with(&[
                    0x1, 0x0, // 1 subrange
                    0x44, 0xac, 0x00, 0x00, // MIN
                    0x44, 0xac, 0x00, 0x00, // MAX
                    0x0, 0x0, 0x0, 0x0, // RES
                ]).expect("Couldn't accept transfer!");
            } else if reqtype == ControlType::Current && entity_id == 2 {
                xfer.accept_with(&[
                    0x44, 0xac, 0x00, 0x00, // CUR
                ]).expect("Couldn't accept transfer!");
            }
        }
    }

    fn control_out(&mut self, xfer: ControlOut<'_, '_, '_, B>) {
        let req = xfer.request();
        log::info!("control: dir={:?} reqtype={:?} recip={:?} req={} value={} idx={} len={}", req.direction, req.request_type, req.recipient, req.request, req.value, req.index, req.length);
        if req.recipient == usb_device::control::Recipient::Interface && req.request == 11 && req.index == 3 {
            xfer.accept().expect("Couldn't accept");
        }
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if self.data_ep.address().index() == addr.index() {
            let mut buf = [0; 768];
            let nbytes = match self.data_ep.read(&mut buf) {
                Ok(x) => x,
                Err(e) => {
                    log::error!("Received error {:?}", e);
                    return;
                }
            };
            //log::info!("RX'd data: EP{}", addr.index());
            self.nbytes += nbytes;
        }
    }
}

#[alloc_error_handler]
fn handle_alloc_error(allocation: core::alloc::Layout) -> ! {
    log::error!("Failing allocation: {:?}", allocation);
    panic!("Allocation failed!");
}

#[cortex_m_rt::entry]
fn main() -> ! {
    unsafe { ALLOCATOR.init(HEAP.as_ptr() as usize, 128*1024) }

    let hal::Peripherals {
        iomuxc,
        mut ccm,
        dma,
        uart,
        mut dcdc,
        gpt1,
        ..
    } = hal::Peripherals::take().unwrap();
    let pins = t41::into_pins(iomuxc);
    let mut led = support::configure_led(pins.p13);

    // Timer for blinking
    let (_, ipg_hz) =
        ccm.pll1
            .set_arm_clock(hal::ccm::PLL1::ARM_HZ, &mut ccm.handle, &mut dcdc);

    let mut cfg = ccm.perclk.configure(
        &mut ccm.handle,
        hal::ccm::perclk::PODF::DIVIDE_3,
        hal::ccm::perclk::CLKSEL::IPG(ipg_hz),
    );

    let mut gpt1 = gpt1.clock(&mut cfg);

    gpt1.set_wait_mode_enable(true);
    gpt1.set_mode(hal::gpt::Mode::Reset);
    gpt1.set_enable(true);

    gpt1.set_output_compare_duration(GPT_OCR, BLINK_PERIOD);

    // DMA initialization (for logging)
    let mut dma_channels = dma.clock(&mut ccm.handle);
    let mut channel = dma_channels[7].take().unwrap();
    channel.set_interrupt_on_completion(false); // We'll poll the logger ourselves...

    //
    // UART initialization (for logging)
    //
    let uarts = uart.clock(
        &mut ccm.handle,
        hal::ccm::uart::ClockSelect::OSC,
        hal::ccm::uart::PrescalarSelect::DIVIDE_1,
    );
    let uart = uarts.uart6.init(pins.p1, pins.p0, UART_BAUD).unwrap();

    let (tx, _) = uart.split();
    imxrt_uart_log::dma::init(tx, channel, imxrt_uart_log::LoggingConfig {
        max_level: log::LevelFilter::Debug,
        filters: &[
            /*("serial", Some(log::LevelFilter::Debug)),
            ("support", Some(log::LevelFilter::Debug)),*/
        ],
    }).unwrap();

    let mut s: alloc::vec::Vec<usize> = alloc::vec::Vec::new();
    s.push(1);
    s.push(2);
    s.push(3);
    log::info!("{:?}", s);
    s[0] = 5;
    s[1] = 6;
    s[2] = 7;
    log::info!("{:?}", s);
    unsafe {
        log::info!("{:?}", &HEAP[0..30]);
    }

    let (ccm, _) = ccm.handle.raw();
    hal::ral::modify_reg!(hal::ral::ccm, ccm, CCGR6, CG1: 0b11, CG0: 0b11);

    let bus_adapter = support::new_bus_adapter();
    let bus = usb_device::bus::UsbBusAllocator::new(bus_adapter);

    //let mut serial = usbd_serial::SerialPort::new(&bus);
    //let mut serial2 = usbd_serial::SerialPort::new(&bus);
    let mut audio = UsbAudio::new(&bus, 768);
    //let mut test_class = TestClass::new(&bus, 64);
    let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x5824, 0x27dd))
        .product("imxrt-usbd")
        .max_packet_size_0(64)
        .device_class(0x0) //usbd_serial::USB_CLASS_CDC)
        .composite_with_iads()
        .build();

    log::info!("Starting to poll USB device...");

    loop {
        imxrt_uart_log::dma::poll();
        if !device.poll(&mut [/*&mut serial,*/ /*&mut test_class,*/ &mut audio]) {
            continue;
        }
        let state = device.state();
        if state == usb_device::device::UsbDeviceState::Configured {
            break;
        }
    }

    device.bus().configure();
    led.set();
    let data = [5; 64];

    let mut n_sent = 1;
    //test_class.data_ep.write(&data);

    loop {
        time_elapse(&mut gpt1, || {
            //bus.debug_poll();
            //log::info!("Toggling LED...");
            //test_class.data_ep.write(&data);
            //log::info!("Count = {} nsent={}", test_class.cnt, n_sent);
            log::info!("Num bytes recv'd = {}", audio.nbytes);
            led.toggle()
        });
        imxrt_uart_log::dma::poll();
        if !device.poll(&mut [/*&mut serial,*/ /*&mut test_class,*/ &mut audio]) {
            continue;
        }

        let mut buf = [0u8; 64];

        /*if n_sent == test_class.cnt {
            n_sent += 1;
            //test_class.data_ep.write(&data);
        }
        test_class.data_ep.write(&data);*/

        /*match serial.read(&mut buf[..]) {
            Ok(count) => {
                let s = core::str::from_utf8(&buf).unwrap();
                log::info!("{}", s);
                serial.write(&buf[..count]).ok();
            }
            Err(UsbError::WouldBlock) => log::warn!("WOULDBLOCK"),
            Err(_err) => {
                //panic!(),
                log::error!("Got explicit error {:?}", _err);
            }
        };*/

        /*let mut buf = [0; 64];
        let nbytes = match audio.data_ep.read(&mut buf) {
            Ok(x) => x,
            Err(e) => {
                log::error!("Error {:?} reading EP", e);
                0
            }
        };
        if nbytes > 0 {
            log::info!("Got {} bytes", nbytes);
        }*/
        //panic!("Aaaaaaa");

        /*match serial2.read(&mut buf[..]) {
            Ok(count) => {
                let s = core::str::from_utf8(&buf).unwrap();
                log::info!("{}", s);
                serial2.write(&buf[..count]).ok();
            }
            Err(UsbError::WouldBlock) => log::warn!("WOULDBLOCK"),
            Err(_err) => panic!(),
        };*/
    }
}

fn time_elapse(gpt: &mut hal::gpt::GPT, func: impl FnOnce()) {
    let mut status = gpt.output_compare_status(GPT_OCR);
    if status.is_set() {
        status.clear();
        func();
    }
}
