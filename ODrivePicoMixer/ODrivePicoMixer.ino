/*

Includes a copy of https://github.com/eziron/ODriveArduino with typos fixed under src to avoid having to install in the main Arduino libraries folder

Includes copies of https://github.com/IoT-gamer/pico-ble-secure and https://github.com/IoT-gamer/pico-ble-notify under src to avoid having to install in the main Arduino libraries folder

*/
#if defined ARDUINO_RASPBERRY_PI_PICO_W || defined ARDUINO_RASPBERRY_PI_PICO_2W
  #pragma message "Building for PICOW/PICO2W with BLE features"
  #define SUPPORT_BLE
#else
  #pragma message "Building for PICO/PICO2 without BLE features"
#endif

#define SUPPORT_SERIAL_DEBUG
#define DEBUG_ODRIVE
#define DEBUG_STATE_MACHINE

#define THROTTLE_PIN A0
#define STEERING_PIN A1
#define LEFT_BRAKE_PIN 10
#define RIGHT_BRAKE_PIN 11
#define REVERSING_SWITCH_PIN 12
#define KILL_SWITCH_PIN 13


//ODrive
#include "src\ODriveArduino.h"
#define odrive_serial Serial1                   //First hardware UART on the Pico, usual GPIO 0/1
uint32_t odrive_baudrate = 115200;              //This is the default but could be changed with Odrivetool
ODriveArduino odrive(odrive_serial);            //ODrive object
struct odrive_state {
  bool enabled = false;                         //Track if velocity control enabled
  float target_velocity = 0;                    //This may ramp up if using slow acceleration for kids etc.
  float requested_velocity = 0;                 //May not match reported velocity
  float reported_velocity = 0;                  //Hopefully up to daye
  float current_limit = 0;                      //What has been requested
  float reported_current = 0;                   //What is reported
};
odrive_state current_odrive_state[2];           //Track state of the ODrive so we don't need to keep querying it over Serial
uint32_t odrive_last_report = 0;                //State tracking timer
uint32_t odrive_report_interval = 1e3;          //State tracking interval (500ms)

//Vehicle profiles
struct vehicle_profile {
    float odrive_max_velocity = 50;             //Absolute maximum velocity
    float odrive_reverse_velocity_ratio = 0.1f; //Multiplier for reverse
    float odrive_max_current = 30;              //Absolute maximum current

    float odrive_mixing_value = 0.5;            //How much steering alters things
    uint8_t acceleration = 255;                 //Acceleration rate
    uint8_t deceleration = 255;                 //Deceleration rate
};
vehicle_profile profile[10];                  //Up to ten profiles
uint8_t selected_profile = 0;

//Vehicle state
enum vehicle_state {
  vehicle_unknown,
  vehicle_locked,
  vehicle_stationary,
  vehicle_moving_forwards,
  vehicle_moving_backwards
};
vehicle_state current_vehicle_state = vehicle_state::vehicle_unknown;

//GPIO
uint32_t last_gpio_read = 0;                  //rate limit GPIO reads, they're not instant
uint32_t gpio_read_interval = 50;             //Specify the rate limit
uint16_t throttle_value = 0;                  //Current throttle value
uint16_t previous_throttle_value = 0;         //Previous throttle value
uint16_t min_throttle_value = 4095;           //Limit for calibration (will be overwritten)
uint16_t max_throttle_value = 0;              //Limit for calibration (will be overwritten)
uint16_t throttle_value_deadspot = 16;        //Need a deadspot for noisy ADC and physical vibration of the potentiometer
uint16_t throttle_low_value_deadspot = 64;    //Need a deadspot for noisy ADC and physical vibration of the potentiometer
uint16_t throttle_high_value_deadspot = 64;   //Need a deadspot for noisy ADC and physical vibration of the potentiometer
uint8_t adjusted_throttle_value = 0;          //Scaled within limits and deadspotted
uint16_t steering_value = 0;                  //Current steering value
uint16_t previous_steering_value = 0;         //Previous steering value
uint16_t min_steering_value = 4095;           //Limit for calibration (will be overwritten)
uint16_t max_steering_value = 0;              //Limit for calibration (will be overwritten)
uint16_t steering_value_deadspot = 16;        //Need a deadspot for noisy ADC and physical vibration of the potentiometer
uint8_t adjusted_steering_value = 0;          //Scaled within limits and deadspotted
uint8_t steering_amount = 0;                  //Magnitude of the steering input
bool left_brake_value = false;                //Current brake value
bool previous_left_brake_value = false;       //Previous brake value
bool right_brake_value = false;               //Current brake value
bool previous_right_brake_value = false;      //Previous brake value
bool reversing_switch_value = false;          //Current reversing switch value
bool previous_reversing_switch_value = false; //Previous reversing switch value
bool kill_switch_value = false;               //Current kill switch value
bool previous_kill_switch_value = false;      //Previous kill switch value
bool gpio_changed = false;                    //Avoid sending constant changes to the ODrive, only update targetted velocities when something happens
uint32_t limits_changed = 0;                  //Need to save limits to non-volatile storage once set

//BLE
#if defined SUPPORT_BLE
  #include <BTstackLib.h>
  #include <SPI.h>
  #define BLE_NAME "VelociRacer"
  #define REMOTE_CONTROL_SERVICE_UUID "bbf8d1fd-1916-4301-b252-59ff26cc6ff2"
  uint16_t ble_remote_control_service_handle = 0;
  #define BLE_KILL_SWITCH_UUID "04bb92cb-f560-489f-9065-8c1428c68777"
  bool ble_kill_switch_value = false;         //Current kill switch value
  uint16_t ble_kill_switch_handle = 0;
  #define PROFILE_SERVICE_UUID "bbf8d1fd-1916-4301-b252-59ff26cc6ff2"
  #define BLE_PROFILE_NUMBER_UUID "8e1214ec-1f22-4aba-8414-399f3ca029fd"
  #define BLE_PROFILE_NAME_UUID "b7a12b97-8c82-480c-b85e-8a99f8e2ea8e"
  #define BLE_MIXING_VALUE_UUID "e870244c-90d2-49c0-948b-2428bab8ad75"
  uint16_t ble_mixing_value_handle = 0;
  #define BLE_MAX_VELOCITY_UUID "ee1524b4-e2fa-4ddc-aba6-b6675739158a5"
  uint16_t ble_max_velocity_handle = 0;
  #define BLE_MAX_CURRENT_UUID "3dbf0f04-9e46-4205-9477-e36d9b898bde"
  uint16_t ble_max_current_handle = 0;
  #define BLE_ACCELERATION_UID "96ea1112-c56a-43d1-879a-1a73c4e97638"
  #define BLE_DECELERATION_UID "bf6974ee-4749-4e8e-baaf-0a723fe7504a"
  //static char characteristic_data = 'H';
  hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
/*
   @section Device Connected Callback

   @text When a remove device connects, device connected callback is callec.
*/
/* LISTING_START(LEPeripheralDeviceConnectedCallback): Device Connected Callback */
void deviceConnectedCallback(BLEStatus status, BLEDevice *device) {
  (void) device;
  switch (status) {
    case BLE_STATUS_OK:
      Serial.println("Device connected!");
      connection_handle = device->getHandle();          // Get connection handle
      break;
    default:
      break;
  }
}
/* LISTING_END(LEPeripheralDeviceConnectedCallback): Device Connected Callback */

/*
   @section Device Disconnected Callback

   @text If the connection to a device breaks, the device disconnected callback
   is called.
*/
/* LISTING_START(LEPeripheralDeviceDisconnectedCallback): Device Disconnected Callback */
void deviceDisconnectedCallback(BLEDevice * device) {
  (void) device;
  Serial.println("Disconnected.");
  connection_handle = HCI_CON_HANDLE_INVALID;
}
/* LISTING_END(LEPeripheralDeviceDisconnectedCallback): Device Disconnected Callback */

/*
   @section Read Callback

   @text In BTstack, the Read Callback is first called to query the size of the
   Characteristic Value, before it is called to provide the data.
   Both times, the size has to be returned. The data is only stored in the provided
   buffer, if the buffer argument is not NULL.
   If more than one dynamic Characteristics is used, the value handle is used
   to distinguish them.
*/
uint16_t gattReadCallback(uint16_t value_handle, uint8_t * buffer, uint16_t buffer_size) {
  if(buffer){ //Check there's a buffer to fill
    if(value_handle == ble_kill_switch_handle && buffer_size > 0) {
      if(ble_kill_switch_value == true) { //Send 0/1 to represent true/false
        buffer[0] = 0x01;
      } else {
        buffer[0] = 0x00;
      }
      return 1;
    } else {  //Return a 0x00 if unhandled
      Serial.println("Unhandled gattReadCallback");
      buffer[0] = 0x00;
      return 1;
    }
  }
  return 0;
}

/*
   @section Write Callback

   @text When the remove device writes a Characteristic Value, the Write callback
   is called. The buffer arguments points to the data of size size/
   If more than one dynamic Characteristics is used, the value handle is used
   to distinguish them.
*/
/* LISTING_START(LEPeripheralWriteCallback): Write Callback */
int gattWriteCallback(uint16_t value_handle, uint8_t *buffer, uint16_t size) {
  //(void) value_handle;
  //(void) size;
  if(value_handle == ble_kill_switch_handle && size == 1) {
    ble_kill_switch_value = (buffer[0] == 1); //Set this off a simple 0/1 value
    gpio_changed = true;  //This acts like GPIO
    #if defined SUPPORT_SERIAL_DEBUG
      Serial.print("BLE kill switch updated:");
      Serial.println(ble_kill_switch_value);
    #endif
  } else {
    #if defined SUPPORT_SERIAL_DEBUG
      Serial.println("Unhandled gattWriteCallback");
    #endif
  }
  return 0;
}
/* LISTING_END(LEPeripheralWriteCallback): Write Callback */
#endif

void setup() {
  #if defined SUPPORT_SERIAL_DEBUG
    delay(1000);  //Tiresome delay for USB to come up
    Serial.begin(); //USB Serial to connected PC
    Serial.println("ODrive mixer starting");
  #endif
  #if defined SUPPORT_BLE
    ble_setup();  //Set up BLE
  #endif
  delay(1000);    //The ODrive does not start instantly
  odrive_setup(); //Do initial setup of the ODrive
  gpio_setup();   //Do initial setup of throttle/steering/brake GPIO
  configuration_storage_setup();  //Do initial setup of non-volatile storage
  configuration_storage_read(); //Read the stored configuration
  state_machine_setup();
}

void loop() {
  gpio_management();  //Read GPIO periodically
  odrive_management();  //Send current velocity targets to the ODrive and get feedback
  configuration_storage_management(); //Update non-volatile storage if necessary
  #if defined SUPPORT_BLE
    ble_management(); //Process updates sent over BLE, which happen as interrupts but are deferred
  #endif
  /*
  #if defined SUPPORT_SERIAL_DEBUG
    Serial.println("Go");
  #endif
  odrive.SetVelocityBoth(5.0f, 5.0f);
  delay(1000);
  #if defined SUPPORT_SERIAL_DEBUG
    Serial.println("Stop");
  #endif
  odrive.SetVelocityBoth(0.0f, 0.0f);
  delay(1000);
  */
  state_machine_management(); //Manage the vehicle state
}
