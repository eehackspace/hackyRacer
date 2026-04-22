#include <LittleFS.h>
#include <ArduinoJson.h>
char configuration_storage_filename[] = "conf.json";  //Configuration file
char min_throttle_value_key[] = "min_t";
char max_throttle_value_key[] = "max_t";
char min_steering_value_key[] = "min_s";
char max_steering_value_key[] = "max_s";
char throttle_value_deadspot_key[] = "sds";
char steering_value_deadspot_key[] = "tds";
char selected_profile_key[] = "sel_prof";
char odrive_max_velocity_key[] = "max_v";
char odrive_reverse_velocity_ratio_key[] = "rev_v";
char odrive_max_current_key[] = "max_a";
char odrive_mixing_value_key[] = "steer_mix";
char acceleration_key[] = "accel";
char deceleration_key[] = "decel";

void configuration_storage_setup() {
  LittleFS.begin(); //Store configuration on LittleFS as JSON
}

void configuration_storage_management() {
  if(limits_changed != 0 && millis() - limits_changed > 60e3) {
    limits_changed = 0;
    configuration_storage_update();
  }
}

void configuration_storage_read() {
  #if defined SUPPORT_SERIAL_DEBUG
    Serial.println("Reading configuration storage");
  #endif
  File file = LittleFS.open(configuration_storage_filename,"r");  //Try to open the file
  if(file) {
    JsonDocument doc; //Temporary JSON document
    DeserializationError error = deserializeJson(doc, file); //Attempt to deserialise the file into the JSON document
    if(error) {
      #if defined SUPPORT_SERIAL_DEBUG
        Serial.print("Failed to parse configuration ");
        Serial.println(configuration_storage_filename);
      #endif
    } else {
      min_throttle_value = doc[min_throttle_value_key] | 4095; //Read values, with default values if not specified
      max_throttle_value = doc[max_throttle_value_key] | 0;
      min_steering_value = doc["min_s"] | 4095;
      max_steering_value = doc["max_s"] | 0;
      configuration_storage_print();
    }
    file.close(); //Close the file!
  } else {
    #if defined SUPPORT_SERIAL_DEBUG
      Serial.print("Failed to open configuration ");
      Serial.println(configuration_storage_filename);
    #endif
  }
}

void configuration_storage_update() {
  #if defined SUPPORT_SERIAL_DEBUG
    Serial.println("Updating configuration storage");
  #endif
  File file = LittleFS.open(configuration_storage_filename, "w"); //Try to open the file
  if(file) {
    JsonDocument doc; //Temporary JSON document
    doc[min_throttle_value_key] = min_throttle_value;  //Populate with value pairs
    doc[max_throttle_value_key] = max_throttle_value;
    doc[min_steering_value_key] = min_steering_value;
    doc[max_steering_value_key] = max_steering_value;
    if (serializeJson(doc, file) == 0) {
      #if defined SUPPORT_SERIAL_DEBUG
        Serial.print("Failed to serialise configuration to ");
        Serial.println(configuration_storage_filename);
      #endif
    }
    file.close(); //Close the file!
  } else {
    #if defined SUPPORT_SERIAL_DEBUG
      Serial.print("Failed to open configuration ");
      Serial.println(configuration_storage_filename);
    #endif
  }
}

#if defined SUPPORT_SERIAL_DEBUG
void configuration_storage_print() {
  Serial.print(min_throttle_value_key);Serial.print(':');Serial.println(min_throttle_value);
  Serial.print(max_throttle_value_key);Serial.print(':');Serial.println(max_throttle_value);
  Serial.print(min_steering_value_key);Serial.print(':');Serial.println(min_steering_value);
  Serial.print(max_steering_value_key);Serial.print(':');Serial.println(max_steering_value);
  Serial.print(throttle_value_deadspot_key);Serial.print(':');Serial.println(throttle_value_deadspot);
  Serial.print(steering_value_deadspot_key);Serial.print(':');Serial.println(steering_value_deadspot);  
  Serial.print(selected_profile_key);Serial.print(':');Serial.println(selected_profile);
  Serial.print(odrive_max_velocity_key);Serial.print(':');Serial.println(profile[selected_profile].odrive_max_velocity);
  Serial.print(odrive_reverse_velocity_ratio_key);Serial.print(':');Serial.println(profile[selected_profile].odrive_reverse_velocity_ratio);
  Serial.print(odrive_max_current_key);Serial.print(':');Serial.println(profile[selected_profile].odrive_max_current);
  Serial.print(odrive_mixing_value_key);Serial.print(':');Serial.println(profile[selected_profile].odrive_mixing_value);
  Serial.print(acceleration_key);Serial.print(':');Serial.println(profile[selected_profile].acceleration);
  Serial.print(deceleration_key);Serial.print(':');Serial.println(profile[selected_profile].deceleration);


}
#endif