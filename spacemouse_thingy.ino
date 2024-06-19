#include <Tlv493d.h>
#include <SimpleKalmanFilter.h>

// Tlv493d Opject
Tlv493d mag = Tlv493d();

//Kalman filter
float v1 = 300;
float v2 = 0.3;
SimpleKalmanFilter xFilter(v1, v1, v2), yFilter(v1, v1, v2), zFilter(v1, v1, v2);

float Tx = 0, Ty = 0, Tz = 0, Rx = 0, Ry = 0, Rz = 0;
int calSamples = 300;
// Magnetometer reading oscilates around 50 uT around center point
// kinda like deadzone
// possibly can be lowered by improving filtering or startup loop
int magTreshold = 60;

// Basic flow:
// While setup take a certain amount of samples to get default position
// Read magnetometer to get translations
// Ready gyroscope to get rotations
// Convert and send

// Include inbuilt Arduino HID library by NicoHood: https://github.com/NicoHood/HID 
#include "HID.h"

// Debug level
int debug = 3;

// This portion sets up the communication with the 3DConnexion software. The communication protocol is created here.
// hidReportDescriptor webpage can be found here: https://eleccelerator.com/tutorial-about-usb-hid-report-descriptors/ 
static const uint8_t _hidReportDescriptor[] PROGMEM = {
  0x05, 0x01,           //  Usage Page (Generic Desktop)
  0x09, 0x08,           //  0x08: Usage (Multi-Axis)
  0xa1, 0x01,           //  Collection (Application)
  0xa1, 0x00,           // Collection (Physical)
  0x85, 0x01,           //  Report ID
  0x16, 0x00, 0x80,     //logical minimum (-500)
  0x26, 0xff, 0x7f,     //logical maximum (500)
  0x36, 0x00, 0x80,     //Physical Minimum (-32768)
  0x46, 0xff, 0x7f,     //Physical Maximum (32767)
  0x09, 0x30,           //    Usage (X)
  0x09, 0x31,           //    Usage (Y)
  0x09, 0x32,           //    Usage (Z)
  0x75, 0x10,           //    Report Size (16)
  0x95, 0x03,           //    Report Count (3)
  0x81, 0x02,           //    Input (variable,absolute)
  0xC0,                 //  End Collection
  0xa1, 0x00,           // Collection (Physical)
  0x85, 0x02,           //  Report ID
  0x16, 0x00, 0x80,     //logical minimum (-500)
  0x26, 0xff, 0x7f,     //logical maximum (500)
  0x36, 0x00, 0x80,     //Physical Minimum (-32768)
  0x46, 0xff, 0x7f,     //Physical Maximum (32767)
  0x09, 0x33,           //    Usage (RX)
  0x09, 0x34,           //    Usage (RY)
  0x09, 0x35,           //    Usage (RZ)
  0x75, 0x10,           //    Report Size (16)
  0x95, 0x03,           //    Report Count (3)
  0x81, 0x02,           //    Input (variable,absolute)
  0xC0,                 //  End Collection
 
  0xa1, 0x00,           // Collection (Physical)
  0x85, 0x03,           //  Report ID
  0x15, 0x00,           //   Logical Minimum (0)
  0x25, 0x01,           //    Logical Maximum (1)
  0x75, 0x01,           //    Report Size (1)
  0x95, 32,             //    Report Count (24)
  0x05, 0x09,           //    Usage Page (Button)
  0x19, 1,              //    Usage Minimum (Button #1)
  0x29, 32,             //    Usage Maximum (Button #24)
  0x81, 0x02,           //    Input (variable,absolute)
  0xC0,
  0xC0
};

//initial centerpoints of magnetometer readout
float centerpointsMag[3];
float centerpointsGyro[3];

void readMagnetometer (float *data){
  data[0] = mag.getX();
  data[1] = mag.getY();
  data[2] = mag.getZ();
}

// Function to send translation and rotation data to the 3DConnexion software using the HID protocol outlined earlier. Two sets of data are sent: translation and then rotation.
// For each, a 16bit integer is split into two using bit shifting. The first is mangitude and the second is direction.
void send_command(int16_t rx, int16_t ry, int16_t rz, int16_t x, int16_t y, int16_t z) {
  uint8_t trans[6] = { x & 0xFF, x >> 8, y & 0xFF, y >> 8, z & 0xFF, z >> 8 };
  HID().SendReport(1, trans, 6);
  uint8_t rot[6] = { rx & 0xFF, rx >> 8, ry & 0xFF, ry >> 8, rz & 0xFF, rz >> 8 };
  HID().SendReport(2, rot, 6);
}

float xOffset = 0, yOffset = 0, zOffset = 0;

void setup() {
  // HID protocol is set.
  static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
  HID().AppendDescriptor(&node);
  // Begin Seral for debugging
  Serial.begin(250000);
  delay(100);
  
  while(!Serial);
  mag.begin();

  Serial.print("Initializing");
  // crude offset calibration on first boot
  for (int i = 1; i <= calSamples; i++)
  {

    // delay(mag.getMeasurementDelay());
    mag.updateData();

    xOffset += mag.getX();
    yOffset += mag.getY();
    zOffset += mag.getZ();

    Serial.print(".");
  }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;
}

void loop() {
  mag.updateData();
  // delay(50);

  //print absolute mag readouts in uT
  if (debug == 1) {
    Serial.print("X = ");
    Serial.print(mag.getX() * 1000);
    Serial.print(" uT; Y = ");
    Serial.print(mag.getY() * 1000);
    Serial.print(" uT; Z = ");
    Serial.print(mag.getZ() * 1000);
    Serial.println(" uT");
  }

  Tx = xFilter.updateEstimate((mag.getX() - xOffset) * 1000);
  Ty = yFilter.updateEstimate((mag.getY() - yOffset) * 1000);
  Tz = zFilter.updateEstimate((mag.getZ() - zOffset) * 1000);

  //print mag readout relative to starting positions in uT
  if (debug == 2) {
    Serial.print("Tx = ");
    Serial.print(Tx);
    Serial.print(" uT; Ty = ");
    Serial.print(Ty);
    Serial.print(" uT; Tz = ");
    Serial.print(Tz);
    Serial.println(" uT");
  }

  Tx = (abs(Tx) - magTreshold > 0) ? Tx : 0;
  Ty = (abs(Ty) - magTreshold > 0) ? Ty : 0;
  Tz = (abs(Tz) - magTreshold > 0) ? Tz : 0;

  //print mag readout relative to starting positions in uT accounting for the deadzone
  if (debug == 3) {
    Serial.print("Tx = ");
    Serial.print(Tx);
    Serial.print(" uT; Ty = ");
    Serial.print(Ty);
    Serial.print(" uT; Tz = ");
    Serial.print(Tz);
    Serial.println(" uT");
  }
   
  delay(50);
}

