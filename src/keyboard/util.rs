use usb_device::bus::{InterfaceNumber, StringIndex, UsbBus, UsbBusAllocator};
use usb_device::class::{ControlIn, ControlOut, UsbClass};
use usb_device::control;
use usb_device::control::{Recipient, RequestType};
use usb_device::descriptor::DescriptorWriter;
use usb_device::endpoint::{EndpointAddress, EndpointIn};
use usb_device::UsbError;

const SPECIFICATION_RELEASE: u16 = 0x111;
const INTERFACE_CLASS_HID: u8 = 0x03;

