
// Define constants and variables
const int A0_pin = A0; // Analog input pin for voltage divider
const int PWM_pin = 9; // PWM output pin for op-amp
const int CAL_pin = 10; // Digital output pin for calibration
const int LED_pin = 13; // LED indicator pin
const float Vref = 1.1; // Internal reference voltage in volts
const float Rcal = 1000.0; // Calibration resistor in ohms
const float Ccal = 100e-6; // Calibration capacitor in farads
const float RC_filter = 1e-6; // RC filter time constant in seconds
const int ADC_bits = 10; // ADC resolution in bits
const int PWM_bits = 16; // PWM resolution in bits
const int window = 10; // Acceptable window for ADC readout in counts

float Rdiv = 10000.0; // Voltage divider resistance in ohms (initial value)
float offset = 0.0; // Offset correction factor (initial value)
float scale = 1.0; // Scale correction factor (initial value)
bool calibrated = false; // Calibration flag

// Initialize the timer1 for 16-bit PWM output
void initPWM () {
  // Set the PWM pin as output
  pinMode (PWM_pin, OUTPUT);

  // Set the timer1 prescaler to 1 (no prescaling)
  TCCR1B = (TCCR1B & B11111000) | B00000001;

  // Set the timer1 mode to phase and frequency correct PWM (mode 8)
  TCCR1A &= B11111100;
  TCCR1B |= B00010000;

  // Set the timer1 output compare mode to non-inverting (COM1A1)
  TCCR1A |= B10000000;
}

// Write a 16-bit value to the PWM output
void writePWM (int value) {
  // Constrain the value between 0 and 65535
  value = constrain (value, 0, 65535);

  // Write the low byte to OCR1AL register
  OCR1AL = value & 0xFF;

  // Write the high byte to OCR1AH register
  OCR1AH = (value >> 8) & 0xFF;
}

// Read a combined value from the ADC and PWM registers
long readCombined () {
  long value = 0; // Combined value
  int adc = 0; // ADC value
  int pwm = 0; // PWM value
  int low = 0; // Lower bound for binary search
  int high = 65535; // Upper bound for binary search

  // Perform a binary search to find the optimal PWM value
  while (low <= high) {
    // Calculate the mid point of the search range
    pwm = (low + high) /2;

    // Write the PWM value to the output pin
    writePWM (pwm);

    // Wait for the RC filter to settle
    delayMicroseconds (RC_filter *1000000);

    // Read the ADC value from the input pin
    adc = analogRead (A0_pin);

    // Check if the ADC value is within the acceptable window
    if (adc >= (512 - window) && adc <= (512 + window)) {
      // Exit the loop
      break;
    }
    else if (adc < (512 - window)) {
      // Increase the lower bound of the search range
      low = pwm +1;
    }
    else if (adc > (512 + window)) {
      // Decrease the upper bound of the search range
      high = pwm -1;
    }
    
    }

    // Combine the ADC and PWM values into a long value
    value = ((long) pwm << ADC_bits) | adc;

    // Return the combined value
    return value;
}

// Calibrate the voltage divider and correction factors using a known RC circuit
void calibrate () {
  // Set the calibration pin as output
  pinMode (CAL_pin, OUTPUT);

  // Set the LED pin as output and turn it on
  pinMode (LED_pin, OUTPUT);
  digitalWrite (LED_pin, HIGH);

  // Charge the capacitor by setting the calibration pin high
  digitalWrite (CAL_pin, HIGH);

  // Wait for the capacitor to charge fully
  delay (1000);

  // Discharge the capacitor by setting the calibration pin low
  digitalWrite (CAL_pin, LOW);

  // Measure the initial voltage across the capacitor
  long V0 = readCombined ();

  // Calculate the expected time constant of the RC circuit
  float tau = Rcal * Ccal;

  // Wait for one time constant
  delay (tau *1000);

  // Measure the final voltage across the capacitor
  long V1 = readCombined ();

  // Calculate the actual time constant of the RC circuit using the formula:
  // V1 = V0 * exp (-t / tau)
  // tau = -t / ln (V1 / V0)
  float t = tau; // Time elapsed in seconds
  float V0f = (float) V0; // Initial voltage in float
  float V1f = (float) V1; // Final voltage in float
  float tauf = -t / log (V1f / V0f); // Actual time constant in seconds

  // Calculate the actual voltage across the capacitor using the formula:
  // Vc = Vref * Rdiv / (Rcal + Rdiv)
  float Vcf = Vref * Rdiv / (Rcal + Rdiv); // Actual voltage in volts

  // Calculate the scale correction factor using the formula:
  // scale = Vc / V0
  scale = Vcf / V0f;

  // Calculate the offset correction factor using the formula:
  // offset = Vref - scale * tauf * (1 << ADC_bits)
  offset = Vref - scale * tauf * (1 << ADC_bits);

  // Store the correction factors in EEPROM
  EEPROM.put (0, scale);
  EEPROM.put (4, offset);

  // Set the calibration flag to true
  calibrated = true;

  // Turn off the LED pin
  digitalWrite (LED_pin, LOW);
}


//Source: Conversation with Bing, 4/19/2023
//(1) Voltage Measurement Using Arduino : 5 Steps - Instructables. https://www.instructables.com/Voltage-Measurement-Using-Arduino/.
//(2) Read Analog Voltage | Arduino Documentation. https://www.arduino.cc/en/Tutorial/ReadAnalogVoltage.
//(3) Measuring Voltage with Arduino - Measure Voltage Upto 25V ... - techZeero. https://techzeero.com/arduino-tutorials/measure-voltage-with-arduino/.
//(4) Arduino & Advanced 16-bit PWM - Codrey Electronics. https://www.codrey.com/arduino-projects/arduino-advanced-16-bit-pwm/.
//(5) Generating an Arduino 16-bit PWM | Microcontroller Tutorials. https://www.teachmemicro.com/generate-arduino-16-bit-pwm/.
//(6) Secrets of Arduino PWM | Arduino Documentation. https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM.
