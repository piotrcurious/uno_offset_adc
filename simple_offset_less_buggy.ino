
// Define constants and variables
const int opampPin = A0; // Unity gain op-amp output connected to A0 pin
const int measurePin = 9; // Measurement voltage source connected to op-amp non-inverting input and PWM pin 9
const int filterCap = 1; // Filter capacitor value in uF
const int eepromAddr = 0; // EEPROM address to store calibration data
float vref = 1.1; // Internal reference voltage in volts
float vdiv = 1.0; // Voltage divider ratio on A0 pin
float voffset = 0.0; // Voltage offset correction
int window = 10; // Acceptable window for ADC readout in counts

// Initialize PWM settings for Timer1
void initPWM () {
  // Set PWM pin as output
  pinMode (measurePin, OUTPUT);
  // Set Timer1 to fast PWM mode with ICR1 as top
  TCCR1A = (1 << WGM11);
  TCCR1B = (1 << WGM12) | (1 << WGM13);
  // Set Timer1 prescaler to 1 (16MHz)
  TCCR1B |= (1 << CS10);
  // Set Timer1 frequency to 244Hz (ICR1 = 65535)
  ICR1 = 65535;
  // Set Timer1 duty cycle to 50% (OCR1A = 32767)
  OCR1A = 32767;
  // Enable Timer1 output on OC1A pin
  TCCR1A |= (1 << COM1A1);
}

// Measure voltage using binary search algorithm
float measureVoltage () {
  // Declare local variables
  int adcValue = 0; // ADC value from A0 pin
  int pwmValue = OCR1A; // PWM value from OCR1A register
  int low = 0; // Lower bound of PWM value
  int high = ICR1; // Upper bound of PWM value
  float voltage = 0.0; // Measured voltage in volts

  // Perform binary search until ADC value is within window
  while (abs (adcValue - window) > window) {
    // Read ADC value from A0 pin
    adcValue = analogRead (opampPin);
    // Adjust PWM value based on ADC value
    if (adcValue < window) {
      low = pwmValue;
      pwmValue = (pwmValue + high) / 2;
    }
    else if (adcValue > window) {
      high = pwmValue;
      pwmValue = (pwmValue + low) / 2;
    }
    // Update OCR1A register with new PWM value
    OCR1A = pwmValue;
    // Wait for PWM and ADC to stabilize
    delay (10);
  }

  // Calculate voltage from PWM and ADC values
  voltage = ((float) pwmValue / ICR1) * vref + ((float) adcValue / window) * vref / window;
  
   return voltage;
}

// Calibrate voltage divider and offset using RC circuit
void calibrateVoltage () {
   Serial.println ("Calibration started");
   Serial.println ("Connect a 100uF capacitor to measurement input");
   Serial.println ("Connect a 1k ohm resistor to digital pin 10");
   Serial.println ("Press any key to continue");
   while (!Serial.available ()) {} // Wait for user input
   Serial.read (); // Clear serial buffer

   // Declare local variables
   float rcTime = 0.0; // RC time constant in seconds
   float rcVoltage = 0.0; // RC voltage in volts
   float maxVoltage = 0.0; // Maximum voltage in volts

   // Set digital pin 10 as output and charge the capacitor
   pinMode (10, OUTPUT);
   digitalWrite (10, HIGH);
   delay (1000); // Wait for capacitor to charge fully

   // Set digital pin 10 as input and measure the time for capacitor to discharge
   pinMode (10, INPUT);
   unsigned long startTime = micros (); // Start time in microseconds
   while (digitalRead (10) == HIGH) {} // Wait for pin

// to discharge
   unsigned long endTime = micros (); // End time in microseconds
   rcTime = (float) (endTime - startTime) / 1000000.0; // Time in seconds

   // Calculate RC voltage from time constant and resistor value
   rcVoltage = vref / (rcTime * 1000.0);

   // Measure voltage using binary search algorithm
   maxVoltage = measureVoltage ();

   // Calculate voltage divider ratio from maximum voltage and RC voltage
   vdiv = maxVoltage / rcVoltage;

   // Calculate voltage offset from PWM value and RC voltage
   voffset = ((float) OCR1A / ICR1) * vref - rcVoltage;

   // Store calibration data in EEPROM
   EEPROM.put (eepromAddr, vdiv);
   EEPROM.put (eepromAddr + sizeof (float), voffset);

   Serial.println ("Calibration done");
   Serial.print ("Voltage divider ratio: ");
   Serial.println (vdiv);
   Serial.print ("Voltage offset: ");
   Serial.println (voffset);
}

// Setup function
void setup () {
  // Initialize serial communication at 9600 baud
  Serial.begin (9600);
  // Initialize PWM settings
  initPWM ();
  // Initialize ADC settings
  analogReference (INTERNAL); // Use internal reference voltage of 1.1V
  // Read calibration data from EEPROM
  EEPROM.get (eepromAddr, vdiv);
  EEPROM.get (eepromAddr + sizeof (float), voffset);
}

// Loop function
void loop () {
  // Declare local variable
  float voltage = 0.0; // Measured voltage in volts

  // Check if user wants to calibrate
  if (Serial.available () > 0) {
    char c = Serial.read (); // Read user input
    if (c == 'c' || c == 'C') { // If user input is 'c' or 'C'
      calibrateVoltage (); // Call calibration function
    }
  }

  // Measure voltage using binary search algorithm
  voltage = measureVoltage ();

  // Apply voltage divider and offset correction
  voltage = voltage / vdiv - voffset;

  // Print measured voltage to serial monitor
  Serial.print ("Measured voltage: ");
  Serial.print (voltage);
  Serial.println (" V");

  // Wait for a second
  delay (1000);
}


//Source: Conversation with Bing, 4/19/2023
//(1) Voltage Measurement Using Arduino : 5 Steps - Instructables. https://www.instructables.com/Voltage-Measurement-Using-Arduino/.
//(2) Read Analog Voltage | Arduino Documentation. https://www.arduino.cc/en/Tutorial/ReadAnalogVoltage.
//(3) Measuring Voltage with Arduino - Measure Voltage Upto 25V ... - techZeero. https://techzeero.com/arduino-tutorials/measure-voltage-with-arduino/.
//(4) Arduino & Advanced 16-bit PWM - Codrey Electronics. https://www.codrey.com/arduino-projects/arduino-advanced-16-bit-pwm/.
//(5) Generating an Arduino 16-bit PWM | Microcontroller Tutorials. https://www.teachmemicro.com/generate-arduino-16-bit-pwm/.
//(6) Secrets of Arduino PWM | Arduino Documentation. https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM.
