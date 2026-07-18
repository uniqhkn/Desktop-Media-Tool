/*!
 * \file    main.c
 * \author  Hakan Korkutan
 * \brief   Milestone 2: EP0 control transfer + full enumeration (Boot Keyboard HID)
 * \version 0.3
 * \date    2026-07-15
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stddef.h>

// =======================================================================
// Low-level: XMega atomic "load and clear" for USB STATUS bits.
// The USB endpoint STATUS register is written by hardware asynchronously
// (on every bus transaction), so a plain &= from firmware can race with
// hardware and silently drop an event. XMega/megaAVR-0 parts have a real
// LAC (Load And Clear) instruction for exactly this situation.
// =======================================================================
static inline void reg_load_and_clear(volatile uint8_t *addr, uint8_t mask)
{
    __asm__ volatile(
        "movw r30, %0"   "\n\t"
        "ld   __tmp_reg__, Z" "\n\t"
        "lac  Z, %1"
        :
        : "r" (addr), "r" (mask)
        : "r30", "r31", "memory");
}

// =======================================================================
// Protected I/O write (CCP)
// =======================================================================
static void CCPWrite(volatile uint8_t *addr, uint8_t value) {
    uint8_t saved_sreg = SREG;
    cli();
    volatile uint8_t *tmpAddr = addr;
#if defined(__AVR_XMEGA__) && (__AVR_XMEGA_VERSION__ >= 100)
    asm volatile(
        "movw r30, %0"  "\n\t"
        "ldi  r16, %2"  "\n\t"
        "out  %3, r16"  "\n\t"
        "st   Z,  %1"
        :
        : "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
        : "r16", "r30", "r31");
#endif
    SREG = saved_sreg;
}

// =======================================================================
// Endpoint table (matches XMega AU manual §21: 8 bytes per direction)
// =======================================================================
typedef struct {
    volatile uint8_t  STATUS;
    volatile uint8_t  CTRL;
    volatile uint16_t CNT;
    volatile uint16_t DATAPTR;
    volatile uint16_t AUXDATA;
} usb_ep_t;

typedef struct {
    usb_ep_t out;
    usb_ep_t in;
} usb_ep_pair_t;

#define NUM_EP_PAIRS 2   // EP0 (control) + EP1 (keyboard IN, wired up now, used in Milestone 3)
static usb_ep_pair_t ep_table[NUM_EP_PAIRS];

#define EP0_SIZE 64
static uint8_t ep0_buf_out[EP0_SIZE];
static uint8_t ep0_buf_in[EP0_SIZE];
static uint8_t ep1_buf_in[8]; // boot keyboard report size, unused until Milestone 3

// =======================================================================
// USB standard request / descriptor definitions (USB 2.0 spec, fixed values)
// =======================================================================
typedef struct __attribute__((packed)) {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_t;

#define REQ_GET_STATUS        0
#define REQ_CLEAR_FEATURE     1
#define REQ_SET_FEATURE       3
#define REQ_SET_ADDRESS       5
#define REQ_GET_DESCRIPTOR    6
#define REQ_SET_DESCRIPTOR    7
#define REQ_GET_CONFIGURATION 8
#define REQ_SET_CONFIGURATION 9
#define REQ_GET_INTERFACE     10
#define REQ_SET_INTERFACE     11

#define DESC_DEVICE        1
#define DESC_CONFIGURATION 2
#define DESC_STRING        3
#define DESC_INTERFACE     4
#define DESC_ENDPOINT      5
#define DESC_HID           0x21
#define DESC_HID_REPORT    0x22

// ---- Descriptors (PROGMEM) --------------------------------------------

// TEST-ONLY VID/PID. Not a legally registered product ID — fine for
// development on your own machine, but swap before wider distribution.
#define USB_VID 0x1209   // pid.codes shared/test vendor ID
#define USB_PID 0x0001   // pid.codes "test" PID, open for hobbyist use

static const uint8_t PROGMEM device_descriptor[] = {
    18,                 // bLength
    DESC_DEVICE,        // bDescriptorType
    0x00, 0x02,         // bcdUSB 2.00
    0x00,               // bDeviceClass (defined at interface level)
    0x00,               // bDeviceSubClass
    0x00,               // bDeviceProtocol
    EP0_SIZE,           // bMaxPacketSize0
    (USB_VID & 0xFF), (USB_VID >> 8),
    (USB_PID & 0xFF), (USB_PID >> 8),
    0x00, 0x01,         // bcdDevice 1.00
    1,                  // iManufacturer -> string index 1
    2,                  // iProduct -> string index 2
    0,                  // iSerialNumber (none)
    1                   // bNumConfigurations
};

static const uint8_t PROGMEM hid_report_descriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0xE0,        //   Usage Minimum (224)
    0x29, 0xE7,        //   Usage Maximum (231)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)   -> modifier byte
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)    -> reserved byte
    0x81, 0x01,        //   Input (Const)
    0x95, 0x06,        //   Report Count (6)   -> 6 keycodes
    0x75, 0x08,        //   Report Size (8)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (101)
    0x05, 0x07,        //   Usage Page (Key Codes)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (101)
    0x81, 0x00,        //   Input (Data,Array)
    0xC0               // End Collection
};
#define HID_REPORT_DESC_LEN sizeof(hid_report_descriptor)

#define CONFIG_TOTAL_LEN (9 + 9 + 9 + 7)

static const uint8_t PROGMEM config_descriptor[] = {
    // Configuration descriptor
    9, DESC_CONFIGURATION,
    (CONFIG_TOTAL_LEN & 0xFF), (CONFIG_TOTAL_LEN >> 8),
    1,      // bNumInterfaces
    1,      // bConfigurationValue
    0,      // iConfiguration
    0x80,   // bmAttributes (bus powered)
    50,     // bMaxPower (100mA)

    // Interface descriptor
    9, DESC_INTERFACE,
    0,      // bInterfaceNumber
    0,      // bAlternateSetting
    1,      // bNumEndpoints
    3,      // bInterfaceClass = HID
    1,      // bInterfaceSubClass = Boot
    1,      // bInterfaceProtocol = Keyboard
    0,      // iInterface

    // HID descriptor
    9, DESC_HID,
    0x11, 0x01,   // bcdHID 1.11
    0,            // bCountryCode
    1,            // bNumDescriptors
    DESC_HID_REPORT,
    (HID_REPORT_DESC_LEN & 0xFF), (HID_REPORT_DESC_LEN >> 8),

    // Endpoint descriptor (EP1 IN, interrupt)
    7, DESC_ENDPOINT,
    0x81,   // bEndpointAddress (EP1 IN)
    0x03,   // bmAttributes (Interrupt)
    8, 0,   // wMaxPacketSize = 8
    10      // bInterval (ms)
};

// String descriptors: index 0 = language, 1 = manufacturer, 2 = product
static const uint8_t PROGMEM string_lang[] = { 4, DESC_STRING, 0x09, 0x04 }; // English (US)
static const uint8_t PROGMEM string_mfr[]  = {
    // "Hakan" as UTF-16LE
    2 + 5*2, DESC_STRING,
    'H',0, 'a',0, 'k',0, 'a',0, 'n',0
};
static const uint8_t PROGMEM string_prod[] = {
    2 + 12*2, DESC_STRING,
    'M',0,'e',0,'d',0,'i',0,'a',0,' ',0,'P',0,'a',0,'d',0,' ',0,'v',0,'1',0
};

// =======================================================================
// Clock bring-up (Milestone 1, unchanged / verified)
// =======================================================================
void clock_init_usb_48mhz(void)
{
    OSC.CTRL |= OSC_RC32MEN_bm;
    while (!(OSC.STATUS & OSC_RC32MRDY_bm)) { }

    NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
    DFLLRC32M.CALB = pgm_read_byte(offsetof(NVM_PROD_SIGNATURES_t, USBRCOSC));
    NVM_CMD = NVM_CMD_NO_OPERATION_gc;

    OSC.DFLLCTRL = OSC_RC32MCREF_USBSOF_gc;
    DFLLRC32M.COMP1 = 0x1B;
    DFLLRC32M.COMP2 = 0xB7;
    DFLLRC32M.CTRL  = DFLL_ENABLE_bm;

    CCPWrite(&CLK.PSCTRL, CLK_PSADIV_2_gc | CLK_PSBCDIV_1_1_gc);
    CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);

    CCPWrite(&CLK.USBCTRL, (CLK.USBCTRL & ~(CLK_USBSRC_gm | CLK_USBSEN_bm))
                             | CLK_USBSRC_RC32M_gc);
    CLK.USBCTRL |= CLK_USBSEN_bm;
}

// =======================================================================
// USB core: reset/init endpoint table, control transfer handling
// =======================================================================
static volatile uint8_t  pending_address = 0;
static volatile uint8_t  address_pending = 0;
static volatile uint8_t  current_config  = 0;

static void usb_ep0_setup(void)
{
    ep_table[0].out.STATUS  = 0;
    ep_table[0].out.CTRL    = USB_EP_TYPE_CONTROL_gc | USB_EP_BUFSIZE_64_gc;
    ep_table[0].out.DATAPTR = (uint16_t)(uintptr_t)ep0_buf_out;

    ep_table[0].in.STATUS   = USB_EP_BUSNACK0_bm;
    ep_table[0].in.CTRL     = USB_EP_TYPE_CONTROL_gc | USB_EP_BUFSIZE_64_gc;
    ep_table[0].in.DATAPTR  = (uint16_t)(uintptr_t)ep0_buf_in;

    // EP1 IN (keyboard) — configured but idle until Milestone 3
    ep_table[1].in.STATUS   = USB_EP_BUSNACK0_bm;
    ep_table[1].in.CTRL     = USB_EP_TYPE_BULK_gc | USB_EP_BUFSIZE_8_gc;
    ep_table[1].in.DATAPTR  = (uint16_t)(uintptr_t)ep1_buf_in;
}

void usb_device_reset(void)
{
    USB.EPPTR = (uint16_t)(uintptr_t)ep_table;
    USB.ADDR  = 0;
    usb_ep0_setup();
    USB.CTRLA = USB_ENABLE_bm | USB_SPEED_bm | (NUM_EP_PAIRS - 1);
    current_config = 0;
    address_pending = 0;
}

static void usb_ep0_send(const uint8_t *progmem_data, uint16_t len)
{
    if (len > EP0_SIZE) len = EP0_SIZE; // single-packet replies only, fine for these descriptors' typical GET size splits handled by host re-requesting
    for (uint16_t i = 0; i < len; i++) ep0_buf_in[i] = pgm_read_byte(progmem_data + i);
    ep_table[0].in.DATAPTR = (uint16_t)(uintptr_t)ep0_buf_in;
    ep_table[0].in.CNT     = len;
    reg_load_and_clear(&ep_table[0].in.STATUS, USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
}

static void usb_ep0_send_zlp(void)
{
    ep_table[0].in.CNT = 0;
    reg_load_and_clear(&ep_table[0].in.STATUS, USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
}

static void usb_ep0_stall(void)
{
    ep_table[0].out.CTRL |= USB_EP_STALL_bm;
    ep_table[0].in.CTRL  |= USB_EP_STALL_bm;
}

static void usb_ep0_out_ready(void)
{
    reg_load_and_clear(&ep_table[0].out.STATUS,
        USB_EP_SETUP_bm | USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm | USB_EP_OVF_bm);
}

static void handle_setup(usb_setup_t *s)
{
    uint8_t desc_type  = s->wValue >> 8;
    uint8_t desc_index = s->wValue & 0xFF;

    switch (s->bRequest) {

    case REQ_SET_ADDRESS:
        pending_address = (uint8_t)(s->wValue & 0x7F);
        address_pending = 1;
        usb_ep0_send_zlp();
        break;

    case REQ_GET_DESCRIPTOR:
        switch (desc_type) {
        case DESC_DEVICE:
            usb_ep0_send(device_descriptor, sizeof(device_descriptor));
            break;
        case DESC_CONFIGURATION:
            usb_ep0_send(config_descriptor,
                s->wLength < CONFIG_TOTAL_LEN ? s->wLength : CONFIG_TOTAL_LEN);
            break;
        case DESC_STRING:
            if (desc_index == 0)
                usb_ep0_send(string_lang, sizeof(string_lang));
            else if (desc_index == 1)
                usb_ep0_send(string_mfr, sizeof(string_mfr));
            else if (desc_index == 2)
                usb_ep0_send(string_prod, sizeof(string_prod));
            else
                usb_ep0_stall();
            break;
        case DESC_HID_REPORT:
            usb_ep0_send(hid_report_descriptor, HID_REPORT_DESC_LEN);
            break;
        default:
            usb_ep0_stall();
            break;
        }
        break;

    case REQ_SET_CONFIGURATION:
        current_config = (uint8_t)s->wValue;
        usb_ep0_send_zlp();
        break;

    case REQ_GET_CONFIGURATION:
        ep0_buf_in[0] = current_config;
        ep_table[0].in.DATAPTR = (uint16_t)(uintptr_t)ep0_buf_in;
        ep_table[0].in.CNT = 1;
        reg_load_and_clear(&ep_table[0].in.STATUS, USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
        break;

    case REQ_GET_STATUS:
        ep0_buf_in[0] = 0;
        ep0_buf_in[1] = 0;
        ep_table[0].in.DATAPTR = (uint16_t)(uintptr_t)ep0_buf_in;
        ep_table[0].in.CNT = 2;
        reg_load_and_clear(&ep_table[0].in.STATUS, USB_EP_BUSNACK0_bm | USB_EP_TRNCOMPL0_bm);
        break;

    case REQ_SET_INTERFACE:
        usb_ep0_send_zlp();
        break;

    default:
        usb_ep0_stall();
        break;
    }
}

// =======================================================================
// Interrupts
// =======================================================================
ISR(USB_BUSEVENT_vect)
{
    if (USB.INTFLAGSACLR & USB_SOFIF_bm) {
        USB.INTFLAGSACLR = USB_SOFIF_bm;
    } else if (USB.INTFLAGSACLR & (USB_CRCIF_bm | USB_UNFIF_bm | USB_OVFIF_bm)) {
        USB.INTFLAGSACLR = (USB_CRCIF_bm | USB_UNFIF_bm | USB_OVFIF_bm);
    } else {
        USB.INTFLAGSACLR = USB_SUSPENDIF_bm | USB_RESUMEIF_bm | USB_RSTIF_bm;
        if (USB.STATUS & USB_BUSRST_bm) {
            USB.STATUS &= ~USB_BUSRST_bm;
            usb_device_reset();
        }
    }
}

ISR(USB_TRNCOMPL_vect)
{
    USB.FIFOWP = 0;
    USB.INTFLAGSBCLR = USB_SETUPIF_bm | USB_TRNIF_bm;

    uint8_t out_status = ep_table[0].out.STATUS;

    if (out_status & USB_EP_SETUP_bm) {
        usb_setup_t setup;
        memcpy(&setup, ep0_buf_out, sizeof(setup));
        reg_load_and_clear(&ep_table[0].out.STATUS, USB_EP_TRNCOMPL0_bm | USB_EP_SETUP_bm);
        handle_setup(&setup);
        usb_ep0_out_ready();
    } else if (out_status & USB_EP_TRNCOMPL0_bm) {
        reg_load_and_clear(&ep_table[0].out.STATUS, USB_EP_TRNCOMPL0_bm);
        usb_ep0_out_ready();
    }

    if (ep_table[0].in.STATUS & USB_EP_TRNCOMPL0_bm) {
        reg_load_and_clear(&ep_table[0].in.STATUS, USB_EP_TRNCOMPL0_bm);
        // Address must be applied only AFTER the SET_ADDRESS status stage ZLP is sent
        if (address_pending) {
            USB.ADDR = pending_address;
            address_pending = 0;
        }
    }
}

// =======================================================================
void usb_attach(void)
{
    usb_device_reset();
    USB.INTCTRLA = USB_BUSEVIE_bm | USB_INTLVL_MED_gc;
    USB.INTCTRLB = USB_SETUPIE_bm | USB_TRNIE_bm | USB_INTLVL_MED_gc;
    PMIC.CTRL |= PMIC_MEDLVLEN_bm;
    sei();
    USB.CTRLB = USB_ATTACH_bm;
}

int main(void)
{
    PORTC.DIRSET = PIN1_bm; // status LED, adjust to your board

    clock_init_usb_48mhz();
    usb_attach();

    while (1) { /* idle */ }
}