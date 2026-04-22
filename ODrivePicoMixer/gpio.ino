void gpio_setup() {
  pinMode(LEFT_BRAKE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BRAKE_PIN, INPUT_PULLUP);
  pinMode(REVERSING_SWITCH_PIN, INPUT_PULLUP);
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
}
void gpio_management() {
  if(millis() - last_gpio_read > gpio_read_interval) {
    last_gpio_read = millis();  //Read all the GPIO now
    throttle_value = analogRead(THROTTLE_PIN);
    steering_value = analogRead(STEERING_PIN);
    left_brake_value = !digitalRead(LEFT_BRAKE_PIN);
    right_brake_value = !digitalRead(RIGHT_BRAKE_PIN);
    reversing_switch_value = !digitalRead(REVERSING_SWITCH_PIN);
    kill_switch_value = !digitalRead(KILL_SWITCH_PIN);
    if(  //Do some basic state change detection. The ADC on throttle and steering is noisy so there is a 'dead spot'
      //Throttle deadspot
      (throttle_value > previous_throttle_value) && (throttle_value - previous_throttle_value > throttle_value_deadspot) ||
      (throttle_value < previous_throttle_value) && (previous_throttle_value - throttle_value > throttle_value_deadspot) ||
      //Steering deadspot
      (steering_value > previous_steering_value) && (steering_value - previous_steering_value > steering_value_deadspot) ||
      (steering_value < previous_steering_value) && (previous_steering_value - steering_value > steering_value_deadspot) ||
      //Brakes
      left_brake_value != previous_left_brake_value ||
      right_brake_value != previous_right_brake_value ||
      //Switches
      reversing_switch_value != previous_reversing_switch_value ||
      kill_switch_value != previous_kill_switch_value) {
      gpio_changed = true;
    }
  }
  if(gpio_changed == true) {
    gpio_update_previous_values();
    #if defined SUPPORT_SERIAL_DEBUG
      gpio_print_state();
    #endif
    gpio_changed = false;
  }
}

void gpio_update_previous_values() {
  //Update throttle
  previous_throttle_value = throttle_value;
  //Track max/min throtte values
  if(throttle_value > max_throttle_value) {max_throttle_value = throttle_value;limits_changed = millis();}
  if(throttle_value < min_throttle_value) {min_throttle_value = throttle_value;limits_changed = millis();}
  //Track extreme high/low thrttle
  if(throttle_value < min_throttle_value + throttle_low_value_deadspot) {throttle_value = min_throttle_value;}
  if(throttle_value > max_throttle_value - throttle_high_value_deadspot) {throttle_value = max_throttle_value;}
  adjusted_throttle_value = map(throttle_value, min_throttle_value, max_throttle_value, 0, 255);
  //Update steering
  previous_steering_value = steering_value;
  //Track max/min steering values
  if(steering_value > max_steering_value) {max_steering_value = steering_value;limits_changed = millis();}
  if(steering_value < min_steering_value) {min_steering_value = steering_value;limits_changed = millis();}
  //Handle extreme left/right values
  if(steering_value < min_steering_value + steering_value_deadspot) {steering_value = min_steering_value;}
  if(steering_value > max_steering_value - steering_value_deadspot) {steering_value = max_steering_value;}
  adjusted_steering_value = map(steering_value, min_steering_value, max_steering_value, 0, 255);
  previous_left_brake_value = left_brake_value;
  previous_right_brake_value = right_brake_value;
  previous_reversing_switch_value = reversing_switch_value;
  previous_kill_switch_value = kill_switch_value;
}



#if defined SUPPORT_SERIAL_DEBUG
void gpio_print_state() {
  Serial.printf("GPIO - throttle:%04u/%03u steering:%04u/%03u", throttle_value,adjusted_throttle_value,steering_value,adjusted_steering_value);
  /*
  Serial.print("GPIO: throttle:");
  Serial.print(throttle_value);
  Serial.print('/');
  Serial.print(adjusted_throttle_value);
  Serial.print(" steering:");
  Serial.print(steering_value);
  Serial.print('/');
  Serial.print(adjusted_steering_value);
  */
  Serial.print(" left brake:");
  Serial.print(left_brake_value);
  Serial.print(" right brake:");
  Serial.print(right_brake_value);
  Serial.print(" reverse:");
  Serial.print(reversing_switch_value);
  Serial.print(" kill:");
  Serial.println(kill_switch_value);
}
#endif