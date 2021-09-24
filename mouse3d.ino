// 15-button 3D mouse
// Needs USBComposite library: 
#include <USBComposite.h>
#include "debounce.h"

static const uint32 dominationPercent = 50;
static const uint16 joyVendorId=0x1EAF; 
static const uint16 mouseVendorId=0x046D;
static const char mouseManufacturerString[]="3dconnexion";
static const char mouseProductString[]="SpaceMouse Pro"; 
static const uint16 mouseProductId=0xc62b;

#define BUTTON12 PA0
#define BUTTON13 PA1
#define BUTTON14 PA2
Debounce button12(BUTTON12, LOW);
Debounce button13(BUTTON13, LOW);
Debounce button14(BUTTON14, LOW);

#define INPUT_MODE 1

#define MOUSE_ON_SERIAL1 // disable this if using the PC to bridge from rs232 to UART
//#define DEBUG
#define POWER_CONTROL PB11

#ifdef MOUSE_ON_SERIAL1
#define SER Serial1
#else
#define SER USBCompositeSerial
#ifdef DEBUG
# error Debug needs MOUSE_ON_SERIAL1
#endif
#endif

#define LED PB12 // change to PC13 if you have a blue pill

#define MODE_JOYSTICK      0x01
#define MODE_DOMINANT_TYPE 0x02
#define MODE_CUBIC         0x04
uint8_t mode = 0; // MODE_DOMINANT_TYPE;
static const int16 trimValue = 500;
static const uint8 minLow = (-trimValue)&0xFF;
static const uint8 minHigh = ((-trimValue)>>8)&0xFF;
static const uint8 maxLow = (trimValue)&0xFF;
static const uint8 maxHigh = ((trimValue)>>8)&0xFF;

const char preface[] = "P20\rAE\rA271006\rM\r"; // update period 32ms, sensitivity Standard (vs. Cubic), auto-rezero Enable (D to disable), auto-rezero after 10,000 ms assuming 6 movement units
const char rezero[] = "Z\r";

int16 trim(int16 v) {
  if (v<-trimValue)
    return -trimValue;
  else if (v>trimValue)
    return trimValue;
  else
    return v;
}

#define INPUT_MODE 0x02 // 0x06 = relative, 0x02 = absolute

uint8_t descriptor_mouse3d[] = {
  0x05, 0x01,           /*  Usage Page (Generic Desktop) */ 
  0x09, 0x08,           /*  0x08: Usage (Multi-Axis Controller) */ 
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
  0x09, 0x33,           /*    Usage (RX) */ 
  0x09, 0x34,           /*    Usage (RY) */ 
  0x09, 0x35,           /*    Usage (RZ) */ 
  0x75, 0x10,           /*    Report Size (16) */ 
  0x95, 0x06,           /*    Report Count (6) */ 
  0x81, INPUT_MODE,           /*    Input (variable,absolute) */ 

    0xa1, 0x02,                         //   COLLECTION (logical)     
  //  0x85, 0x05,         /*  Report ID */
    0x09, 0x52,                         //    USAGE (Input Mode)         
    0x15, 0x00,                         //    LOGICAL_MINIMUM (0)      
    0x25, 0x0a,                         //    LOGICAL_MAXIMUM (255)
    0x75, 0x08,                         //    REPORT_SIZE (8)         
    0x95, 0x01,                         //    REPORT_COUNT (8)         
    0xb1, 0x02,                         //    FEATURE (Data,Var,Abs    
    0xc0,                               //   END_COLLECTION
  
  0xC0,                           /*  End Collection */ 

  0xa1, 0x00,            // Collection (Physical)
  0x85, 0x03,         /*  Report ID */
  0x15, 0x00,           /*   Logical Minimum (0) */ 
  0x25, 0x01,           /*    Logical Maximum (1) */
  0x75, 0x01,           /*    Report Size (1) */ 
  0x95, 15,           /*    Report Count (24) */
  0x05, 0x09,           /*    Usage Page (Button) */ 
  0x19, 1,           /*    Usage Minimum (Button #1) */ 
  0x29, 15,           /*    Usage Maximum (Button #24) */ 
  0x81, 0x02,           /*    Input (variable,absolute) */ 
  0x95, 0x01,          //     REPORT_COUNT = 1
  0x75, 32-15,          //     REPORT_SIZE = 5
  0x81, 0x03,          //     INPUT = Cnst,Var,Abs
  0xC0,

  0xa1, 0x00,           // Collection (Physical)
  0x85, 0x04,           //  Report ID 
  0x05, 0x08,       /*   USAGE_PAGE (LEDs) */ 
  0x19, 0x01,       /*   USAGE_MINIMUM (1) */ 
  0x29, 0x08,       /*   USAGE_MAXIMUM (8)*/ 
  0x95, 0x08,       /*   REPORT_COUNT (8) */ 
  0x75, 0x01,       /*   REPORT_SIZE (1) */ 
  0x91, 0x02,         /*   OUTPUT (Data,Var,Abs) */    
  0xC0,

  0xC0
};

uint8_t descriptor_joy3d[] = {
  0x05, 0x01,           /*  Usage Page (Generic Desktop) */ 
  0x09, 0x04,           /*  0x04: Usage (Joystick) */ 
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
  0x09, 0x33,           /*    Usage (RX) */ 
  0x09, 0x34,           /*    Usage (RY) */ 
  0x09, 0x35,           /*    Usage (RZ) */ 
  0x75, 0x10,           /*    Report Size (16) */ 
  0x95, 0x06,           /*    Report Count (6) */ 
  0x81, 0x02,           /*    Input (variable,absolute) */ 
  0x15, 0x00,           /*   Logical Minimum (0) */ 
  0x25, 0x01,           /*    Logical Maximum (1) */
  0x75, 0x01,           /*    Report Size (1) */ 
  0x95, 15,           /*    Report Count (15) */
  0x05, 0x09,           /*    Usage Page (Button) */ 
  0x19, 1,           /*    Usage Minimum (Button #1) */ 
  0x29, 15,           /*    Usage Maximum (Button #16) */ 
  0x81, 0x02,           /*    Input (variable,absolute) */ 
  0x95, 0x01,          //     REPORT_COUNT = 1
  0x75, 32-15,         //     REPORT_SIZE = 5
  0x81, 0x03,          //     INPUT = Cnst,Var,Abs
  
    0x09, 0x52,                         //    USAGE (Input Mode)         
    0x15, 0x00,                         //    LOGICAL_MINIMUM (0)      
    0x25, 0x0a,                         //    LOGICAL_MAXIMUM (255)
    0x75, 0x08,                         //    REPORT_SIZE (1)         
    0x95, 0x01,                         //    REPORT_COUNT (8)         
    0xb1, 0x02,                         //    FEATURE (Data,Var,Abs)
  0xC0,

  0xa1, 0x00,           // Collection (Physical)
  0x85, 0x04,           //  Report ID 
  0x05, 0x08,       /*   USAGE_PAGE (LEDs) */ 
  0x19, 0x01,       /*   USAGE_MINIMUM (1) */ 
  0x29, 0x08,       /*   USAGE_MAXIMUM (8)*/ 
  0x95, 0x08,       /*   REPORT_COUNT (8) */ 
  0x75, 0x01,       /*   REPORT_SIZE (1) */ 
  0x91, 0x02,         /*   OUTPUT (Data,Var,Abs) */    
  0xC0,

  0xC0
};

USBHID HID;
#ifndef MOUSE_ON_SERIAL1
USBCompositeSerial SER;
#endif
#ifdef DEBUG
USBCompositeSerial CompositeSerial;
#endif

typedef struct {
    uint8_t reportID;
    int16 x;
    int16 y;
    int16 z;
    int16 rx;
    int16 ry;
    int16 rz;
} __packed ReportMovement_t;

typedef struct {
    uint8_t reportID;
    uint32_t buttons;
} __packed ReportButtons_t;

typedef struct {
    uint8_t reportID;
    int16 x;
    int16 y;
    int16 z;
    int16 rx;
    int16 ry;
    int16 rz;
    uint32_t buttons;
} __packed ReportJoy_t;

volatile uint8_t ledBuffer[HID_BUFFER_ALLOCATE_SIZE(1,1)] = {0};
volatile uint8_t modeBuffer[HID_BUFFER_ALLOCATE_SIZE(1,1)];

uint32 addExtraButtons(uint32 buttons) {
   buttons &= (1<<12)-1;
   if (button12.getLastState())
      buttons |= 1<<12;
   if (button13.getLastState())
      buttons |= 1<<13;
   if (button14.getLastState())
      buttons |= 1<<14;
   return buttons;
}

class HIDMouse3D {
protected:
public:
    ReportMovement_t movement;
    ReportButtons_t buttons;
    HIDReporter movementReporter;
    HIDReporter buttonsReporter;
    HIDBuffer_t ledData;
    HIDBuffer_t modeData;
    HIDMouse3D(USBHID& HID) 
            : movementReporter(HID, (uint8_t*)&movement, sizeof(movement), 1),
              buttonsReporter(HID, (uint8_t*)&buttons, sizeof(buttons), 3),
              ledData(ledBuffer, HID_BUFFER_SIZE(1,4), 4, HID_BUFFER_MODE_NO_WAIT),
              modeData(modeBuffer, HID_BUFFER_SIZE(1,4), 1, HID_BUFFER_MODE_NO_WAIT)
              {}

    void sendPosition() {
      movementReporter.sendReport();
    }

    void sendButtons() {
      buttons.buttons = addExtraButtons(buttons.buttons);
      buttonsReporter.sendReport();
    }

    void send() {
      sendPosition();
      sendButtons();
    }

    void begin() {
      HID.addOutputBuffer(&ledData);      
      HID.addFeatureBuffer(&modeData);
    }
};

class HIDJoy3D {
protected:
public:
    ReportJoy_t report;
    HIDReporter reporter;
    HIDBuffer_t ledData;
    volatile uint8_t leds[HID_BUFFER_ALLOCATE_SIZE(1,1)];
    HIDBuffer_t modeData;
    volatile uint8_t mode[HID_BUFFER_ALLOCATE_SIZE(1,1)];
    HIDJoy3D(USBHID& HID) 
            : reporter(HID, (uint8_t*)&report, sizeof(report), 1),
              ledData(ledBuffer, HID_BUFFER_SIZE(1,4), 4, HID_BUFFER_MODE_NO_WAIT),
              modeData(modeBuffer, HID_BUFFER_SIZE(1,4), 1, HID_BUFFER_MODE_NO_WAIT)
              {}

    void send() {
      report.buttons = addExtraButtons(report.buttons);
      reporter.sendReport();
    }

    void begin() {
      HID.addOutputBuffer(&ledData);      
      HID.addFeatureBuffer(&modeData);      
    }
};

HIDMouse3D Mouse3D(HID);
HIDJoy3D Joy3D(HID);

void startMouse3D() {  
  USBComposite.clear();
  USBComposite.setManufacturerString(mouseManufacturerString); 
  USBComposite.setProductString(mouseProductString); 
  USBComposite.setVendorId(mouseVendorId);
  USBComposite.setProductId(mouseProductId); 
  HID.clearBuffers();
  HID.setReportDescriptor(descriptor_mouse3d, sizeof(descriptor_mouse3d));
  HID.registerComponent();
  Mouse3D.begin();
#ifndef MOUSE_ON_SERIAL1
  SER.registerComponent();
#endif
#ifdef DEBUG
  CompositeSerial.registerComponent();
#endif
  USBComposite.begin();
}

void startJoy3D() {  
  USBComposite.clear();
  USBComposite.setManufacturerString("stm32duino");
  USBComposite.setProductString("Mouse3D");
  USBComposite.setVendorId(joyVendorId);
  USBComposite.setProductId(mouseProductId); 
  HID.clearBuffers();
  HID.setReportDescriptor(descriptor_joy3d, sizeof(descriptor_joy3d));
  HID.registerComponent();
  Joy3D.begin();
#ifndef MOUSE_ON_SERIAL1
  SER.registerComponent();
#endif
#ifdef DEBUG
  CompositeSerial.registerComponent();
#endif
  USBComposite.begin();
}

void loadMode() {
  EEPROM8_init();
  int i = EEPROM8_getValue(INPUT_MODE);
  if (i < 0)
    mode = 0;
  else
    mode = i & ~MODE_DOMINANT_TYPE; 
}

bool getFeature(uint8* newModeP) {
  if (mode & MODE_JOYSTICK) {
    return Joy3D.reporter.getFeature(newModeP);
  }
  else {
    return Mouse3D.movementReporter.getFeature(newModeP);
  }
}

void setFeature() {
  if (mode & MODE_JOYSTICK) {
    Joy3D.reporter.setFeature(&mode);
  }
  else {
    Mouse3D.movementReporter.setFeature(&mode);
  }
}

void saveMode() {
  EEPROM8_storeValue(INPUT_MODE, mode);
}

void writeCubic(bool cubic) {
  if (cubic)
    SER.write("YC\r");
  else
    SER.write("YS\r");
}

void writeConfiguration() {
  SER.write(preface);  
  writeCubic((mode & MODE_CUBIC) != 0);
}

void setMode(uint8 newMode) {
  if (mode == newMode)
    return;
  uint8_t delta = mode ^ newMode;
  if (delta & MODE_JOYSTICK) {
    if (0 == (newMode & MODE_JOYSTICK)) {
        HID.end();
        USBComposite.end();
        SER.write("BaA\r");
        delay(1000);
        startMouse3D();
        delay(1000);
    }
    else {
        HID.end();
        USBComposite.end();
        SER.write("BjA\r");
        delay(1000);
        startJoy3D();
        delay(1000);
    }
  }
  if (delta & MODE_CUBIC) {
    writeCubic(0 != (newMode & MODE_CUBIC));
  }
  mode = newMode;
  setFeature();
  saveMode();
}

void send() {
  if (mode & MODE_JOYSTICK) {
    Joy3D.send();
  }
  else
    Mouse3D.send();
}

void setup() {
#ifdef MOUSE_ON_SERIAL1
  pinMode(POWER_CONTROL, OUTPUT);
  digitalWrite(POWER_CONTROL, 1);
  pinMode(PA9, OUTPUT);
  digitalWrite(PA9, 1);
#endif

  pinMode(BUTTON12, INPUT_PULLUP);
  pinMode(BUTTON13, INPUT_PULLUP);
  pinMode(BUTTON14, INPUT_PULLUP);
  button12.begin();
  button13.begin();
  button14.begin();

  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0);

  loadMode();

  if (mode & MODE_JOYSTICK) {
    startJoy3D();
  }
  else {
    startMouse3D();
  }

  setFeature();
    
  delay(200);
  
  digitalWrite(LED,1);
#ifdef MOUSE_ON_SERIAL1
  digitalWrite(POWER_CONTROL, 0);
#endif  
  delay(300);
  SER.begin(9600,SERIAL_8N2);
  delay(500);

  uint32 t = millis();
  while((millis()-t) < 2000) {
    if (SER.available()) {
      uint8_t c = Serial.read();
#ifdef DEBUG
      CompositeSerial.write(c);      
#endif      
    }
  }
//  SER.write(preface);  
  while((millis()-t) < 2000) {
    if (SER.available()) Serial.read();
  }
  
  writeConfiguration();  

  send();
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
#ifdef DEBUG
      CompositeSerial.println(String(b));
#endif      
#ifndef SWITCHABLE_BROKEN
      if ((b & (0b111 << 6)) == (0b111 << 6)) {
        if (b & (1 << 3)) {
          setMode(mode & ~MODE_JOYSTICK);
        }
        else if (b & (1 << 4)) {
          setMode(mode | MODE_JOYSTICK);
        }
      }
#endif      
      if (mode & MODE_JOYSTICK) {
        Joy3D.report.buttons = b;
        Joy3D.send();
      } 
      else {
        Mouse3D.buttons.buttons = b;
        Mouse3D.send();
      }
    }
  }
  else if (buf[0] == 'D') {
    if (len == 15) {
      lastD = millis();
      if (mode & MODE_JOYSTICK) {      
        Joy3D.report.x = trim(get16(buf, 3));
        Joy3D.report.z = trim(get16(buf, 5));
        Joy3D.report.y = trim(-get16(buf, 7));
        Joy3D.report.ry = trim(get16(buf, 9));
        Joy3D.report.rz = trim(-get16(buf, 11));
        Joy3D.report.rx = trim(get16(buf, 13));
        Joy3D.send();
      }
      else {
        int16 x = get16(buf, 3);
        int16 z = get16(buf, 5);
        int16 y = get16(buf, 7);
        int16 rx = get16(buf, 9);
        int16 rz = get16(buf, 11);
        int16 ry = get16(buf, 13);
        if (mode & MODE_DOMINANT_TYPE) {
          uint32 t2 = (int32)x*x+(int32)y*y+(int32)z*z;
          uint32 r2 = (int32)rx*x+(int32)ry*ry+(int32)rz*rz;
          if (t2 >= (1<<(31-15)) || r2 >= (1<<(31-15))) {
            t2 >>= 15;
            r2 >>= 15;
          }
          if (t2 * (dominationPercent*dominationPercent) >= r2 * 10000) {
            rx = 0;
            ry = 0;
            rz = 0;
          }
          else if (r2 * (dominationPercent*dominationPercent) >= t2 * 10000) {
            x = 0;
            y = 0;
            z = 0;
          }
          else {
            x = 0;
            y = 0;
            z = 0;
            rx = 0;
            ry = 0;
            rz = 0;
          }
        }
        Mouse3D.movement.x = trim(x); 
        Mouse3D.movement.z = trim(-z);
        Mouse3D.movement.y = trim(-y); 
        Mouse3D.movement.rx = trim(rx);
        Mouse3D.movement.rz = trim(-rz);
        Mouse3D.movement.ry = trim(-ry);
        Mouse3D.send();
      }
    }
  }
}

void loop() {
  if (millis() >= lastD + 5000) {
     SER.write("M\r");
     lastD = millis();
  }
  digitalWrite(LED,(ledBuffer[1] & 0x3) == 0);
#ifdef DEBUG  
  CompositeSerial.println(String((uint32)ledBuffer[1],HEX)+" "+String((uint32)modeBuffer[1],HEX));
#endif

  uint8 newMode;  
  
  if (getFeature(&newMode)) {
    setMode(newMode);
  }

  // this code is written awkwardly to avoid short-circuiting the three
  // calls to Debounce::wasToggled()
  
  bool toggled = button12.wasToggled();
  toggled = button13.wasToggled() || toggled;
  if (button14.wasToggled() || toggled) {
    send();  
  }

  while (SER.available()) {
    uint8 c = SER.read();
#ifdef DEBUG
    CompositeSerial.write(c);
#endif        
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

