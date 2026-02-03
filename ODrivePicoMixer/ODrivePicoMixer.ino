/*

Includes a copy of https://github.com/eziron/ODriveArduino with typos fixed under src to avoid having to install in the main Arduino libraries folder

*/

#include "src\ODriveArduino.h"

#define odrive_serial Serial1
unsigned long baudrate = 115200; // Must match what you configure on the ODrive (see docs for details)


ODriveArduino odrive(odrive_serial);
uint32_t lastFeedback = 0;

void setup() {
  //odrive_serial.setRX(0);
  //odrive_serial.setTX(1);
  odrive_serial.begin(baudrate);
  Serial.begin(); // Serial to PC
  delay(1000);
  Serial.println("ODrive test");
  delay(1000);
  odrive.run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  odrive.run_state(1, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
}

void loop() {
  Serial.println("Go");
  odrive.SetVelocityBoth(5.0f, 5.0f);
  delay(1000);
  Serial.println("Stop");
  odrive.SetVelocityBoth(0.0f, 0.0f);
  delay(1000);
}
