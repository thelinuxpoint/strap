pub const USB_CLASS_HID: u8 = 0x03;

pub const USB_SUBCLASS_NONE: u8 = 0x00;
pub const USB_SUBCLASS_BOOT: u8 = 0x01;

pub const USB_INTERFACE_NONE: u8 = 0x00;
pub const USB_INTERFACE_KEYBOARD: u8 = 0x01;
pub const USB_INTERFACE_MOUSE: u8 = 0x02;

pub const REQ_GET_REPORT: u8 = 0x01;
pub const REQ_GET_IDLE: u8 = 0x02;
pub const REQ_GET_PROTOCOL: u8 = 0x03;
pub const REQ_SET_REPORT: u8 = 0x09;
pub const REQ_SET_IDLE: u8 = 0x0a;
pub const REQ_SET_PROTOCOL: u8 = 0x0b;

//
pub const REPORT_DESCR: &[u8] = &[
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x02, // USAGE (Mouse)
    0xa1, 0x01, // COLLECTION (Application)
    0x09, 0x01, // USAGE (Pointer)
    0xa1, 0x00, // COLLECTION (Physical)
    0x05, 0x09, // USAGE_PAGE (Button)
    0x19, 0x01, // USAGE_MINIMUM (Button 1)
    0x29, 0x03, // USAGE_MAXIMUM (Button 3)
    0x15, 0x00, // LOGICAL_MINIMUM (0)
    0x25, 0x01, // LOGICAL_MAXIMUM (1)
    0x95, 0x03, // REPORT_COUNT (3)
    0x75, 0x01, // REPORT_SIZE (1)
    0x81, 0x02, // INPUT (Data,Var,Abs)
    0x95, 0x01, // REPORT_COUNT (1)
    0x75, 0x05, // REPORT_SIZE (5)
    0x81, 0x03, // INPUT (Cnst,Var,Abs)
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x30, // USAGE (X)
    0x09, 0x31, // USAGE (Y)
    0x15, 0x81, // LOGICAL_MINIMUM (-127)
    0x25, 0x7f, // LOGICAL_MAXIMUM (127)
    0x75, 0x08, // REPORT_SIZE (8)
    0x95, 0x02, // REPORT_COUNT (2)
    0x81, 0x06, // INPUT (Data,Var,Rel)
    0xc0,       // END_COLLECTION
    0xc0,       // END_COLLECTION
];
