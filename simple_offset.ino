
// Arduino uno code for voltage measurement using op-amp and PWM
// Disclaimer: This code is generated by Bing and has not been tested or verified. Use it at your own risk.

// Constants and variables
const int opAmpPin = A0; // op-amp output connected to A0 pin through voltage divider
const int pwmPin = 9; // 16-bit PWM pin connected to op-amp inverting input
const int capPin = 10; // digital pin connected to 100uF capacitor for calibration
const int adcRef = INTERNAL; // ADC reference voltage
const int pwmMax = 65535; // maximum value for 16-bit PWM
const int adcMax = 1023; // maximum value for 10-bit ADC
const int window = 10; // acceptable window for ADC readout in binary search
int offset = 0; // offset value for PWM
float scale = 0; // scaling factor for PWM
int divider = 1; // voltage divider ratio for A0 pin
float correction = 1.0; // correction factor for measurement
const float vcc = 5.0; // max vcc for op-amp
const float vref = 1.1 ; // vref value for ADC


// Initialize the PWM and ADC settings
void setup() {
  Serial.begin(9600); // start serial communication
  pinMode(pwmPin, OUTPUT); // set PWM pin as output
  pinMode(capPin, OUTPUT); // set capacitor pin as output
  analogReference(adcRef); // set ADC reference voltage
  scale = vcc / pwmMax; // calculate the scaling factor using the formula above
  float ADC_scale = vref/adcMax ; // calculate the scaling for ADC
}

// Main loop
void loop() {
  float voltage = measureVoltage(); // measure the voltage using the function below
  Serial.println(voltage); // print the voltage to serial monitor
  delay(1000); // wait for a second
}

// Function to measure the voltage using op-amp and PWM
float measureVoltage() {
  int adcUpper = binarySearch() * scale; // find the upper 16 bits of measurement by binary search
  int adcLower = analogRead(opAmpPin) * ADC_scale; // find the lower 10 bits of measurement by ADC readout
  float voltage = (adcUpper + adcLower) * correction; // combine the results and apply correction factor
//  float voltage = (adcUpper + adcLower) * correction; // combine the results and apply correction factor
  return voltage; // return the voltage value
}

// Function to perform binary search on PWM value to find the measurement window
int binarySearch() {
  int low = 0; // lower bound of PWM value
  int high = pwmMax; // upper bound of PWM value
  int mid = (low + high) / 2; // middle value of PWM value
  while (true) { // loop until break condition is met
    analogWrite(pwmPin, mid); // write the PWM value to the pin
    delay(10); // wait for the op-amp and ADC to stabilize
    int adcValue = analogRead(opAmpPin); // read the ADC value from the pin
    if (adcValue > adcMax - window) { // if the ADC value is too high, lower the PWM value
      high = mid - 1;
    }
    else if (adcValue < window) { // if the ADC value is too low, raise the PWM value
      low = mid + 1;
    }
    else { // if the ADC value is within the window, break the loop and return the PWM value
      break;
    }
    mid = (low + high) / 2; // update the middle value of PWM value
    if (low > high) { // if the bounds cross each other, something went wrong, return -1 as an error code
      return -1;
    }
  }
  return mid;
}

// Function to calibrate the voltage divider and correction factor using a capacitor and resistor circuit
void calibrate() {
  Serial.println("Calibration mode. Connect a 100uF capacitor to measurement input and a 1k ohm resistor to pin 10.");
  Serial.println("Press any key to continue.");
  while (!Serial.available()); // wait for user input
  // Charge the capacitor by setting pin 10 high
  digitalWrite(capPin, HIGH);
  delay(1000); // wait for a second
  // Discharge the capacitor by setting pin 10 low and measure the time it takes to reach half of the ADC reference voltage
  digitalWrite(capPin, LOW);
  unsigned long startTime = micros(); // record the start time
  while (analogRead(opAmpPin) > adcMax / 2); // wait until the ADC value drops below half
  unsigned long endTime = micros(); // record the end time
  float rcTime = (endTime - startTime) / 1000000.0; // calculate the RC time constant in seconds
  Serial.print("RC time constant: ");
  Serial.println(rcTime); // print the RC time constant to serial monitor
  // Calculate the reference voltage using the formula Vref = Vcc * R / (R + r), where Vcc is the supply voltage, R is the resistor value and r is the internal resistance of pin 10
  float vcc = 5.0; // assume a supply voltage of 5V
  float r = 1000.0; // assume a resistor value of 1k ohm
  float rInternal = rcTime / (log(2) * 100e-6) - r; // calculate the internal resistance of pin 10 using the formula r = RC / ln(2) - R, where C is the capacitor value
  Serial.print("Internal resistance of pin 10: ");
  Serial.println(rInternal); // print the internal resistance of pin 10 to serial monitor
  float vRef = vcc * r / (r + rInternal); // calculate the reference voltage using the formula above
  Serial.print("Reference voltage: ");
  Serial.println(vRef); // print the reference voltage to serial monitor
  // Find the maximum PWM value that gives a ADC value close to adcMax by binary search
  int pwmMaxValue = binarySearch();
  if (pwmMaxValue == -1) { // if something went wrong, abort calibration and return
    Serial.println("Error: PWM value not found.");
    return;
  }

  // Calculate the voltage divider ratio using the formula Vout = Vin * R2 / (R1 + R2), where Vin is the reference voltage, Vout is the op-amp output voltage and R1 and R2 are the resistors in the voltage divider
  float vOut = pwmMaxValue * vcc / pwmMax; // calculate the op-amp output voltage using the PWM value and the supply voltage
  Serial.print("Op-amp output voltage: ");
  Serial.println(vOut); // print the op-amp output voltage to serial monitor
  float r1 = 10000.0; // assume a resistor value of 10k ohm for R1
  float r2 = vOut * r1 / (vRef - vOut); // calculate the resistor value for R2 using the formula above
  Serial.print("Voltage divider ratio: ");
  Serial.println(r2 / r1); // print the voltage divider ratio to serial monitor
  // Calculate the correction factor using the formula Vmeasured = Vactual * correction, where Vmeasured is the voltage value obtained from the ADC and PWM combination and Vactual is the actual voltage value
  float vMeasured = (pwmMaxValue + adcMax) * vcc / (pwmMax + adcMax); // calculate the measured voltage value using the PWM and ADC values and the supply voltage
  Serial.print("Measured voltage: ");
  Serial.println(vMeasured); // print the measured voltage value to serial monitor
  float vActual = vRef; // assume that the actual voltage value is equal to the reference voltage value
  float correctionFactor = vActual / vMeasured; // calculate the correction factor using the formula above
  Serial.print("Correction factor: ");
  Serial.println(correctionFactor); // print the correction factor to serial monitor
  // Store the divider and correction values in EEPROM for future use
  EEPROM.put(0, correctionFactor); // store the correction value in address 0 to 3
  EEPROM.put(6, r2/r1); // store the divider value in address 4 to 7
  Serial.println("Calibration done. Values stored in EEPROM.");
}