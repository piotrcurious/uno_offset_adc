

// Measure the voltage using binary search and ADC readout
float measure_voltage() {
  int pwm_value = (1 << (PWM_BITS - 1)); // Start with half of the PWM range
  int pwm_step = pwm_value / 2; // Start with half of the PWM step
  int adc_value = analogRead(ADC_PIN); // Read the ADC value
  while (abs(adc_value - (1 << (ADC_BITS - 1))) > window) { // While the ADC value is not in the middle of the range
    if (adc_value > (1 << (ADC_BITS - 1))) { // If the ADC value is too high
      pwm_value -= pwm_step; // Decrease the PWM value by one step
    }
    else { // If the ADC value is too low
      pwm_value += pwm_step; // Increase the PWM value by one step
    }
    pwm_step /= 2; // Halve the PWM step
    if (pwm_step == 0) { // If the PWM step is zero
      break; // Exit the loop
    }
    pwmWrite(PWM_PIN, pwm_value); // Write the new PWM value to the pin
    delay(10); // Wait for the RC filter to settle
    adc_value = analogRead(ADC_PIN); // Read the new ADC value
  }
  float v_upper = v_offset + (float)pwm_value / (1 << PWM_BITS) * ADC_REF; // Calculate the upper part of the voltage from PWM value and offset correction
  float v_lower = (float)adc_value / (1 << ADC_BITS) * v_divider * ADC_REF; // Calculate the lower part of the voltage from ADC value and divider correction
  float v_total = v_upper + v_lower; // Calculate the total voltage by adding both parts
  return v_total; // Return the total voltage
}

