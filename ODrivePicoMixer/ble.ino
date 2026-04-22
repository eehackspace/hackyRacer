#if defined SUPPORT_BLE

void ble_setup() {
  #if defined SUPPORT_SERIAL_DEBUG
    Serial.print("Starting BLE as ");
    Serial.println(BLE_NAME);
    Serial.print("Service UUID: ");
    Serial.println(REMOTE_CONTROL_SERVICE_UUID);
    Serial.print("Kill switch UUID: ");
    Serial.println(BLE_KILL_SWITCH_UUID);
  #endif
  // set callbacks
  BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
  BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
  BTstack.setGATTCharacteristicRead(gattReadCallback);
  BTstack.setGATTCharacteristicWrite(gattWriteCallback);

  // setup GATT Database
  BTstack.addGATTService(new UUID(REMOTE_CONTROL_SERVICE_UUID));
  ble_kill_switch_handle = BTstack.addGATTCharacteristicDynamic(new UUID(BLE_KILL_SWITCH_UUID), ATT_PROPERTY_READ | ATT_PROPERTY_WRITE | ATT_PROPERTY_NOTIFY, 0);
  BTstack.addGATTService(new UUID(PROFILE_SERVICE_UUID));
  ble_mixing_value_handle = BTstack.addGATTCharacteristicDynamic(new UUID(BLE_MIXING_VALUE_UUID), ATT_PROPERTY_READ | ATT_PROPERTY_WRITE | ATT_PROPERTY_NOTIFY, 0);
  ble_max_velocity_handle = BTstack.addGATTCharacteristicDynamic(new UUID(BLE_MAX_VELOCITY_UUID), ATT_PROPERTY_READ | ATT_PROPERTY_WRITE | ATT_PROPERTY_NOTIFY, 0);
  ble_max_current_handle = BTstack.addGATTCharacteristicDynamic(new UUID(BLE_MAX_CURRENT_UUID), ATT_PROPERTY_READ | ATT_PROPERTY_WRITE | ATT_PROPERTY_NOTIFY, 0);

  // startup Bluetooth and activate advertisements
  BTstack.setup(BLE_NAME);
  BTstack.startAdvertising();
}

void ble_management() {
  BTstack.loop();
}

#endif