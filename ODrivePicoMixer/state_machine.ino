void state_machine_setup() {
  state_machine_change(vehicle_state::vehicle_locked);
}

void state_machine_management() {
  if(current_vehicle_state == vehicle_state::vehicle_locked) {
    if(adjusted_throttle_value == 0 && kill_switch_engaged() == false) {
      state_machine_change(vehicle_state::vehicle_stationary); //Only unlock if the throttle is at zero and killswitch disengaged
      odrive_enable_velocity_control(0);
      odrive_enable_velocity_control(1);
    }
  } else if(current_vehicle_state == vehicle_state::vehicle_stationary) {
    if(kill_switch_engaged() == true) {
      state_machine_change(vehicle_state::vehicle_locked); //Engage the kill switch at idle
      odrive_disable_velocity_control(0);
      odrive_disable_velocity_control(1);
    } else if(adjusted_throttle_value != 0) {
      if(reversing_switch_value == false) {
        state_machine_change(vehicle_state::vehicle_moving_forwards); //Only move forwards from idle
      } else {
        state_machine_change(vehicle_state::vehicle_moving_backwards); //Only move backwards from idle
      }
    }
  } else if(current_vehicle_state == vehicle_state::vehicle_moving_forwards || current_vehicle_state == vehicle_state::vehicle_moving_backwards) {
    if(kill_switch_engaged() == true) {
      state_machine_change(vehicle_state::vehicle_locked); //Engage the kill switch at idle
      odrive_disable_velocity_control(0);
      odrive_disable_velocity_control(1);
    } else if(adjusted_throttle_value == 0) {
      state_machine_change(vehicle_state::vehicle_stationary); //Only go idle if the throttle is at zero and reported speed is zero
    }
  }
}

void state_machine_change(vehicle_state new_state) {
  if(new_state != current_vehicle_state) {
    #if defined SUPPORT_SERIAL_DEBUG && defined DEBUG_STATE_MACHINE
      Serial.print("New vehicle state:");
      if(new_state == vehicle_state::vehicle_locked) {
        Serial.println("locked");
      } else if(new_state == vehicle_state::vehicle_stationary) {
        Serial.println("stationary");
      } else if(new_state == vehicle_state::vehicle_moving_forwards) {
        Serial.println("moving forwards");
      } else if(new_state == vehicle_state::vehicle_moving_backwards) {
        Serial.println("moving backwards");
      }
    #endif
    current_vehicle_state = new_state;
  }
}

bool kill_switch_engaged() {
  #if defined SUPPORT_BLE
    return ble_kill_switch_value || kill_switch_value;
  #else
    return kill_switch_value;
  #endif
}