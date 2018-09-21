// 24-button 3D mouse
// Needs USBComposite library: 
#include <USBComposite.h>

//#define MOUSE_ON_SERIAL1

#ifdef MOUSE_ON_SERIAL1
#define SER Serial1
#else
#define SER USBCompositeSerial
#endif

#undef JOYSTICK_MODE
#define LED PB12

static const int16 trimValue = 500;
static const uint8 minLow = (-trimValue)&0xFF;
static const uint8 minHigh = ((-trimValue)>>8)&0xFF;
static const uint8 maxLow = (trimValue)&0xFF;
static const uint8 maxHigh = ((trimValue)>>8)&0xFF;

const char preface[] = "P20\rYS\rAE\rA271006\rM\r"; // update period 32ms, sensitivity Standard (vs. Cubic), auto-rezero Enable (D to disable), auto-rezero after 10,000 ms assuming 6 movement units
const char rezero[] = "Z\r";

int16 trim(int16 v) {
  if (v<-trimValue)
    return -trimValue;
  else if (v>trimValue)
    return trimValue;
  else
    return v;
}

uint8_t descriptor_mouse3d[] = {
  0x05, 0x01,           /*  Usage Page (Generic Desktop) */ 
#ifdef JOYSTICK_MODE
  0x09, 0x04,           /*  0x08: Usage (Joystick) */ 
#else  
  0x09, 0x08,           /*  0x08: Usage (Multi-Axis Controller) */ 
#endif  
  0xA1, 0x01,           /*  Collection (Application) */ 
  0xa1, 0x00,            // Collection (Physical)
  0x85, 0x01,         /*  Report ID */
  0x16, minLow,minHigh,        //logical minimum (-500)
  0x26, maxLow,maxHigh,        //logical maximum (500)
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
  0x16, minLow,minHigh,        //logical minimum (-500)
  0x26, maxLow,maxHigh,        //logical maximum (500)
  0x36, 0x00,0x80,              // Physical Minimum (-32768)
  0x46, 0xff,0x7f,              //Physical Maximum (32767)
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
#ifndef MOUSE_ON_SERIAL1
USBCompositeSerial SER;
#endif

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
  pinMode(LED, OUTPUT);

#ifdef JOYSTICK_MODE
  USBComposite.setManufacturerString("stm32duino");
  USBComposite.setProductString("Mouse3D");
  USBComposite.setVendorId(0x1EAF);
#else
  USBComposite.setManufacturerString("3dconnexion"); // "stm32duino");
  USBComposite.setProductString("SpaceMouse Pro"); // "Mouse3D");
  USBComposite.setVendorId(0x46D);
#endif
  USBComposite.setProductId(0xc62b); 
  HID.setReportDescriptor(descriptor_mouse3d, sizeof(descriptor_mouse3d));
  HID.registerComponent();
#ifndef MOUSE_ON_SERIAL1  
  SER.registerComponent();
#endif  
  USBComposite.begin();
  delay(1000);
  SER.begin(9600);

  digitalWrite(LED,0);
  uint32 t = millis();
  while((millis()-t) < 2000) {
    if (SER.available()) Serial.read();
  }
  SER.write(preface);  
  digitalWrite(LED,1);
  Mouse3D.sendButtons();
  Mouse3D.sendPosition();
  
}

#define BUFFER_SIZE 128
uint32 lastD = 0;
uint32 bufferPos = 0;
uint8 buffer[BUFFER_SIZE];
bool escape = false;

inline uint16 get16(const uint8* data, uint32 offset) {
  return data[offset+1] | ((uint16)data[offset+0] << 8);
}

void processBuffer(const uint8* buf, uint32 len) {
  if (buf[0] == '.') {
    if (len == 3) {
      uint16 b = buf[2] | ((uint16)(buf[1])<<8);
      //rightHanded = (0 != (b & 0b10000000000000));
      //haveHandedness = true;
      b = ((b&0b111111) | ((b&~0b1111111)>>1)) & 0b111111111111;
      b = ((b >> 9) | (b << 3)) & 0b111111111111;
      Mouse3D.buttons.buttons = b;
      Mouse3D.sendButtons();
    }
  }
  else if (buf[0] == 'D') {
    if (len == 15) {
      lastD = millis();      
#ifdef JOYSTICK_MODE
      Mouse3D.xyz.x = trim(get16(buf, 3));
      Mouse3D.xyz.z = trim(get16(buf, 5));
      Mouse3D.xyz.y = trim(-get16(buf, 7));
      Mouse3D.rxyz.rx = trim(get16(buf, 9));
      Mouse3D.rxyz.rz = trim(get16(buf, 11));
      Mouse3D.rxyz.ry = trim(-get16(buf, 13));
#else      
      Mouse3D.xyz.x = trim(get16(buf, 3)); // rl adjusts
      Mouse3D.xyz.y = trim(get16(buf, 5));
      Mouse3D.xyz.z = trim(-get16(buf, 7)); // rl adjusts
      Mouse3D.rxyz.rx = trim(get16(buf, 9)); // rl adjusts
      Mouse3D.rxyz.ry = trim(get16(buf, 11));
      Mouse3D.rxyz.rz = trim(-get16(buf, 13)); //rl adjusts
#endif      
      Mouse3D.sendPosition();
    }
  }
}

void loop() {
  if (millis()-lastD > 5000) {
    SER.write(preface);  
    lastD = millis();
  }
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

