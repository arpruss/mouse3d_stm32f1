// Needs USBComposite library: 
#include <USBComposite.h>

#define LED PB12

uint8_t descriptor_mouse3d[] = {
  0x05, 0x01,           /*  Usage Page (Generic Desktop) */ 
  0x09, 0x08,           /*  0x08: Usage (Multi-Axis Controller) */ 
  0xA1, 0x01,           /*  Collection (Application) */ 
  0xa1, 0x00,            // Collection (Physical)
  0x85, 0x01,         /*  Report ID */
  0x16,0x0c,0xfe,        //logical minimum (-500)
  0x26,0xf4,0x01,        //logical maximum (500)
  0x36,0x00,0x80,              // Physical Minimum (-32768)
  0x46,0xff,0x7f,              //Physical Maximum (32767)
  0x09, 0x30,           /*    Usage (X) */ 
  0x09, 0x31,           /*    Usage (Y) */ 
  0x09, 0x32,           /*    Usage (Z) */ 
  0x75, 0x10,           /*    Report Size (16) */ 
  0x95, 0x03,           /*    Report Count (3) */ 
  0x81, 0x02,           /*    Input (variable,absolute) */ 
  0xC0,                           /*  End Collection */ 

  0xa1, 0x00,            // Collection (Physical)
  0x85, 0x02,         /*  Report ID */
  0x16,0x0c,0xfe,        //logical minimum (-500)
  0x26,0xf4,0x01,        //logical maximum (500)
  0x36,0x00,0x80,              // Physical Minimum (-32768)
  0x46,0xff,0x7f,              //Physical Maximum (32767)
  0x09, 0x33,           /*    Usage (RX) */ 
  0x09, 0x34,           /*    Usage (RY) */ 
  0x09, 0x35,           /*    Usage (RZ) */ 
  0x75, 0x10,           /*    Report Size (16) */ 
  0x95, 0x03,           /*    Report Count (3) */ 
  0x81, 0x02,           /*    Input (variable,absolute) */ 
  0xC0,                           /*  End Collection */ 
    
  0xa1, 0x00,            // Collection (Physical)
  0x85, 0x03,         /*  Report ID */
  0x15, 0x00,           /*   Logical Minimum (0) */ 
  0x25, 0x01,           /*    Logical Maximum (1) */
  0x75, 0x01,           /*    Report Size (1) */ 
  0x95, 24,           /*    Report Count (24) */
  0x05, 0x09,           /*    Usage Page (Button) */ 
  0x19, 1,           /*    Usage Minimum (Button #1) */ 
  0x29, 24,           /*    Usage Maximum (Button #24) */ 
  0x81, 0x02,           /*    Input (variable,absolute) */ 
  0xC0,

  0xC0
};

USBHID HID;

typedef struct {
    uint8_t reportID;
    int16 x;
    int16 y;
    int16 z;
} __packed ReportXYZ_t;

typedef struct {
    uint8_t reportID;
    int16 rx;
    int16 ry;
    int16 rz;
} __packed ReportRXYZ_t;

typedef struct {
    uint8_t reportID;
    uint8_t buttons;
    uint8_t ignore[5];
} __packed ReportButtons_t;

class HIDMouse3D {
protected:
public:
    ReportXYZ_t xyz;
    ReportRXYZ_t rxyz;
    ReportButtons_t buttons;
    HIDReporter xyzReporter;
    HIDReporter rxyzReporter;
    HIDReporter buttonsReporter;
    HIDMouse3D(USBHID& HID) 
            : xyzReporter(HID, (uint8_t*)&xyz, sizeof(xyz), 1),
              rxyzReporter(HID, (uint8_t*)&rxyz, sizeof(rxyz), 2),
              buttonsReporter(HID, (uint8_t*)&buttons, sizeof(buttons), 3) 
              {}
    void send() {
      xyzReporter.sendReport();
      rxyzReporter.sendReport();
      buttonsReporter.sendReport();
    }
};

HIDMouse3D Mouse3D(HID);

void setup() {
  USBComposite.setManufacturerString("stm32duino");
  USBComposite.setProductString("Mouse3D");
  USBComposite.setVendorId(0x46D);
  USBComposite.setProductId(0xC625);
  HID.begin(descriptor_mouse3d, sizeof(descriptor_mouse3d));
  pinMode(LED, OUTPUT);
  
}

void loop() {
  Mouse3D.xyz.x = -400;
  Mouse3D.xyz.y = 0;
  Mouse3D.xyz.z = 0;
  Mouse3D.rxyz.rx = -400;
  Mouse3D.rxyz.ry = 0;
  Mouse3D.rxyz.rz = 0;
  digitalWrite(LED,0);
  for (int i=0;i<30;i++) {
    Mouse3D.send();
    delay(100);
  }
  Mouse3D.xyz.x = 400;
  Mouse3D.xyz.y = 0;
  Mouse3D.xyz.z = 0;
  Mouse3D.rxyz.rx = 400;
  Mouse3D.rxyz.ry = 0;
  Mouse3D.rxyz.rz = 0;
  digitalWrite(LED,1);
  for (int i=0;i<30;i++) {
    Mouse3D.send();
    delay(100);
  }
}

