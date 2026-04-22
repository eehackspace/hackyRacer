void odrive_setup() {
  //odrive_serial.setRX(0);
  //odrive_serial.setTX(1);
  odrive_serial.begin(odrive_baudrate);
  odrive_set_current_limit(0, profile[selected_profile].odrive_max_current);
  odrive_set_current_limit(1, profile[selected_profile].odrive_max_current);
  //odrive.SetVelocityBoth(current_odrive_state[0].requested_velocity, current_odrive_state[1].requested_velocity);
  odrive_disable_velocity_control(0);
  odrive_disable_velocity_control(1);
}

void odrive_management() {
  if(current_vehicle_state == vehicle_state::vehicle_moving_forwards || current_vehicle_state == vehicle_state::vehicle_moving_backwards) {
    odrive_select_velocities();
    if(millis() - odrive_last_report > odrive_report_interval) {
      odrive_last_report = millis();
      #if defined SUPPORT_SERIAL_DEBUG && defined DEBUG_ODRIVE
        Serial.printf("ODrv - target: %02.1f/%02.1f requested: %02.1f/%02.1f reported: %02.1f/%02.1f steering:%u = %s %u\r\n",current_odrive_state[0].target_velocity,current_odrive_state[1].target_velocity,
          current_odrive_state[0].requested_velocity,current_odrive_state[1].requested_velocity,
          current_odrive_state[0].reported_velocity,current_odrive_state[1].reported_velocity,
          adjusted_steering_value,
          (turning_left()==true) ? "left" : ((turning_right()==true) ? "right" : "straight"),
          steering_amount
          );
      #endif
    }
    odrive.SetVelocity(0, current_odrive_state[0].requested_velocity);
    odrive.SetVelocity(1, current_odrive_state[1].requested_velocity);
  }
}

void odrive_select_velocities() {
  //Set the base speed
  if(current_vehicle_state == vehicle_state::vehicle_moving_backwards) {  //Reverse is slower
    current_odrive_state[0].target_velocity = -profile[selected_profile].odrive_reverse_velocity_ratio * float(adjusted_throttle_value)*profile[selected_profile].odrive_max_velocity/255.0f;
    current_odrive_state[1].target_velocity = -profile[selected_profile].odrive_reverse_velocity_ratio * float(adjusted_throttle_value)*profile[selected_profile].odrive_max_velocity/255.0f;
  } else if (current_vehicle_state == vehicle_state::vehicle_moving_forwards) {
    current_odrive_state[0].target_velocity = float(adjusted_throttle_value)*profile[selected_profile].odrive_max_velocity/255.0f;
    current_odrive_state[1].target_velocity = float(adjusted_throttle_value)*profile[selected_profile].odrive_max_velocity/255.0f;
  }
  //Apply torque vectoring/differential in a VERY naive way
  if(turning_left()) { //Steering left
    steering_amount = 128 - adjusted_steering_value;
    if(current_vehicle_state == vehicle_state::vehicle_moving_backwards) {
      current_odrive_state[0].target_velocity = current_odrive_state[0].target_velocity * (1.0 - (profile[selected_profile].odrive_mixing_value * steering_amount / 255.0f));
      current_odrive_state[1].target_velocity = current_odrive_state[1].target_velocity * (1.0 + (profile[selected_profile].odrive_mixing_value * steering_amount / 255.0f));
    } else if (current_vehicle_state == vehicle_state::vehicle_moving_forwards) {
      current_odrive_state[0].target_velocity = current_odrive_state[0].target_velocity * (1.0 - (profile[selected_profile].odrive_mixing_value * steering_amount / 255.0f));
      current_odrive_state[1].target_velocity = current_odrive_state[1].target_velocity * (1.0 + (profile[selected_profile].odrive_mixing_value * steering_amount / 255.0f));
    }
  } else if(turning_right()) { //Steering right
    steering_amount = adjusted_steering_value - 127;
    if(current_vehicle_state == vehicle_state::vehicle_moving_backwards) {
      current_odrive_state[0].target_velocity = current_odrive_state[0].target_velocity * (1.0 + (profile[selected_profile].odrive_mixing_value * steering_amount / 255.0f));
      current_odrive_state[1].target_velocity = current_odrive_state[1].target_velocity * (1.0 - (profile[selected_profile].odrive_mixing_value * steering_amount / 255.0f));
    } else if (current_vehicle_state == vehicle_state::vehicle_moving_forwards) {
      current_odrive_state[0].target_velocity = current_odrive_state[0].target_velocity * (1.0 + (profile[selected_profile].odrive_mixing_value * steering_amount / 255.0f));
      current_odrive_state[1].target_velocity = current_odrive_state[1].target_velocity * (1.0 - (profile[selected_profile].odrive_mixing_value * steering_amount / 255.0f));
    }
  } else {
    steering_amount = 0;
  }
  if(profile[selected_profile].acceleration == 255 && profile[selected_profile].deceleration == 255) {  //Maximum acceleration & deceleration just asks it of the ODrive straight away
    current_odrive_state[0].requested_velocity = current_odrive_state[0].target_velocity;
    current_odrive_state[1].requested_velocity = current_odrive_state[1].target_velocity;
  } else { //Lower acceleration/deceleration ramps the requested velocity
    current_odrive_state[0].requested_velocity = current_odrive_state[0].target_velocity;
    current_odrive_state[1].requested_velocity = current_odrive_state[1].target_velocity;
  }
  odrive.SetVelocityBoth(current_odrive_state[0].requested_velocity, current_odrive_state[1].requested_velocity);
}

bool turning_left() {
  return adjusted_steering_value < 127 - (steering_value_deadspot)/2;
}

bool turning_right() {
  return adjusted_steering_value > 127 + (steering_value_deadspot)/2;
}

void odrive_enable_velocity_control(uint8_t axis) {
  #if defined SUPPORT_SERIAL_DEBUG
    Serial.print("Enabling velocity control on axis:");
    Serial.println(axis);
  #endif
  odrive.SetVelocity(axis,current_odrive_state[axis].requested_velocity);
  odrive.run_state(axis, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  current_odrive_state[axis].enabled = true;
}

void odrive_disable_velocity_control(uint8_t axis) {
  #if defined SUPPORT_SERIAL_DEBUG
    Serial.print("Disabling velocity control on axis:");
    Serial.println(axis);
  #endif
  current_odrive_state[axis].requested_velocity = 0;
  odrive.SetVelocity(axis,current_odrive_state[axis].requested_velocity);
  odrive.run_state(axis, AXIS_STATE_IDLE, false);
  current_odrive_state[axis].enabled = false;
}

void odrive_set_current_limit(uint8_t axis, float current) {
  #if defined SUPPORT_SERIAL_DEBUG
    Serial.print("Setting current limit ");
    Serial.print(current);
    Serial.print("A on axis:");
    Serial.println(axis);
  #endif
  odrive.SetCurrent(axis, current);
  current_odrive_state[axis].current_limit = current;
}