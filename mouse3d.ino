// 24-button 3D mouse
// Needs USBComposite library: 
//#define SER CompositeSerial
#include <USBComposite.h>

#define JOYSTICK_MODE
#define LED PB12

uint8_t descriptor_mouse3d[] = {
  0x05, 0x01,           /*  Usage Page (Generic Desktop) */ 
#ifdef JOYSTICK_MODE
  0x09, 0x08,           /*  0x08: Usage (Multi-Axis Controller) */ 
#else  
  0x09, 0x04,           /*  0x08: Usage (Multi-Axis Controller) */ 
#endif  
  0xA1, 0x01,           /*  Collection (Application) */ 
  0xa1, 0x00,            // Collection (Physical)
  0x85, 0x01,         /*  Report ID */
//  0x16,0x0c,0xfe,        //logical minimum (-500)
//  0x26,0xf4,0x01,        //logical maximum (500)
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
//  0x16,0x0c,0xfe,        //logical minimum (-500)
//  0x26,0xf4,0x01,        //logical maximum (500)
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
USBCompositeSerial SER;

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
    uint16_t buttons;
    uint8_t ignore[1];
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
    void sendPosition() {
      xyzReporter.sendReport();
      rxyzReporter.sendReport();
    }

    void sendButtons() {
      buttonsReporter.sendReport();
    }
};

HIDMouse3D Mouse3D(HID);

void setup() {
  USBComposite.setManufacturerString("stm32duino");
  USBComposite.setProductString("Mouse3D");
  USBComposite.setVendorId(0x46D);
  USBComposite.setProductId(0xC625);
  HID.setReportDescriptor(descriptor_mouse3d, sizeof(descriptor_mouse3d));
  HID.registerComponent();
  SER.registerComponent();
  USBComposite.begin();
  delay(1000);
  SER.begin(9600);

  uint32 t = millis();
  while((millis()-t) < 5000) {
    if (SER.available()) Serial.read();
  }
  SER.write("\x4D\rS\rS\rS\rS\r");  
  
  pinMode(LED, OUTPUT);
}

#define BUFFER_SIZE 128
uint32 bufferPos = 0;
uint8 buffer[BUFFER_SIZE];
bool escape = false;

inline uint16 get16(const uint8* data, uint32 offset) {
  return data[offset+1] | ((uint16)data[offset+0] << 8);
}

void processBuffer(const uint8* buf, uint32 len) {
  if (buf[0] == '.') {
    if (len == 3) {
      Mouse3D.buttons.buttons = (buf[2]) | ((uint16)(buf[1])<<8);
      Mouse3D.sendButtons();
      digitalWrite(LED,1);
    }
  }
  else if (buf[0] == 'D') {
    if (len == 15) {
#ifdef JOYSTICK_MODE
      Mouse3D.xyz.x = get16(buf, 3);
      Mouse3D.xyz.z = get16(buf, 5);
      Mouse3D.xyz.y = get16(buf, 7);
      Mouse3D.rxyz.rx = get16(buf, 9);
      Mouse3D.rxyz.rz = get16(buf, 11);
      Mouse3D.rxyz.ry = get16(buf, 13);
#else      
      Mouse3D.xyz.x = get16(buf, 3);
      Mouse3D.xyz.y = -get16(buf, 5);
      Mouse3D.xyz.z = get16(buf, 7);
      Mouse3D.rxyz.rx = get16(buf, 9);
      Mouse3D.rxyz.ry = get16(buf, 11);
      Mouse3D.rxyz.rz = get16(buf, 13);
#endif      
      Mouse3D.sendPosition();
      digitalWrite(LED,0);
    }
  }
}

void loop() {
  while (SER.available()) {
    digitalWrite(LED,0);  
    uint8 c = SER.read();
    digitalWrite(LED,1);
    if (c == '\r') {
      if (bufferPos < BUFFER_SIZE) 
        processBuffer(buffer,bufferPos);
      bufferPos = 0;
      continue;
    }
    else if (escape) {
      if (c == 'Q' || c == 'S' || c == 'M')
        c &= 0b10111111;
      escape = false;
    }
    else if (c == '^') {
      escape = true;
      continue;
    }

    if (bufferPos < BUFFER_SIZE)
      buffer[bufferPos++] = c;    
  }
}

