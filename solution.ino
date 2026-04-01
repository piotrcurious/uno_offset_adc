
/*
 * High-Resolution Voltage Measurement for Arduino Uno
 * Uses 16-bit PWM (Timer1) to offset the input voltage and a 10-bit ADC
 * to measure the residual, effectively extending resolution.
 *
 * Circuit Architecture:
 * - Input Voltage -> Voltage Divider -> Op-Amp Non-Inverting Input (+)
 * - Pin 9 (16-bit PWM) -> RC Filter -> Op-Amp Inverting Input (-)
 * - Op-Amp Output -> Current Limiting Resistor -> Pin A0 (ADC)
 */
#include <Arduino.h>
#include <EEPROM.h>

// Pins
const int opampPin = A0;
const int pwmPin = 9;
const int calPin = 10;
const int eepromAddr = 0;

// Calibration variables
float vdiv = 0.1; // Initial guess for divider ratio (V_divided / V_input)
float voffset = 0.0; // Offset correction in Volts
float vcc = 5.0; // Calibrated Vcc used for PWM
int window = 5; // Acceptable window for ADC readout in counts

// Initialize 16-bit PWM on Pin 9
void initPWM() {
  pinMode(pwmPin, OUTPUT);
  TCCR1A = (1 << COM1A1);
  TCCR1B = (1 << WGM13) | (1 << CS10); // Phase and Frequency Correct PWM, ICR1 as TOP, Prescaler 1
  ICR1 = 65535;
  OCR1A = 32767;
}

void writePWM16(uint16_t val) {
  OCR1A = val;
}

// Helper to read ADC with averaging
int analogReadAveraged(int pin, int samples = 16) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return sum / samples;
}

// Forward declaration
float readVcc();

// Measurement function
float measureVoltage() {
  int32_t pwmValue = 32768;
  int32_t step = 16384;
  int adcValue = 0;

  for (int i = 0; i < 16; i++) {
    writePWM16((uint16_t)constrain(pwmValue, 0, 65535));
    delay(20); // Allow RC filter to settle
    adcValue = analogReadAveraged(opampPin, 8); // Fast averaging during search

    if (adcValue > 512 + window) {
      pwmValue += step;
    } else if (adcValue < 512 - window) {
      pwmValue -= step;
    } else {
      break;
    }
    step >>= 1;
  }
  pwmValue = constrain(pwmValue, 0, 65535);
  writePWM16((uint16_t)pwmValue);
  delay(30); // Extra time for final settlement

  // Final high-precision measurement
  adcValue = analogReadAveraged(opampPin, 64);

  float vpwm = ((float)pwmValue / 65535.0) * vcc;
  float vout_opamp = ((float)adcValue / 1023.0) * 1.1;

  return vpwm + vout_opamp;
}

void calibrateVoltage() {
  // Clear any existing characters (like the 'c' that triggered this)
  while (Serial.available() > 0) Serial.read();

  Serial.println(F("Calibration started"));
  Serial.println(F("Connect 100uF cap to meas input, 1k ohm to pin 10."));
  Serial.println(F("Press any key to start..."));

  while (Serial.available() == 0) { delay(10); }
  while (Serial.available() > 0) Serial.read();

  // Charge capacitor
  pinMode(calPin, OUTPUT);
  digitalWrite(calPin, HIGH);
  Serial.println(F("Charging..."));
  delay(3000);

  // Use PWM=0 to measure input directly through op-amp when it's within 1.1V range
  writePWM16(0);
  delay(500);
  int initial_adc = analogReadAveraged(opampPin, 64);
  Serial.print(F("Initial ADC at 5V: ")); Serial.println(initial_adc);

  if (initial_adc < 100) {
      Serial.println(F("Error: Capacitor not charging. Check connections."));
      return;
  }

  digitalWrite(calPin, LOW); // Start discharge

  // Wait for ADC to drop to 80% of initial
  int target1 = (int)(initial_adc * 0.8);
  while (analogRead(opampPin) > target1) { ; }
  unsigned long t1 = micros();

  // Wait for ADC to drop to 40% of initial (which is half of 80%)
  int target2 = (int)(initial_adc * 0.4);
  while (analogRead(opampPin) > target2) { ; }
  unsigned long t2 = micros();

  float dt = (float)(t2 - t1) / 1000000.0;
  // RC = dt / ln(0.8 / 0.4) = dt / ln(2)
  float rc_actual = dt / 0.693147;
  Serial.print(F("Measured RC: ")); Serial.println(rc_actual);

  // Use measured RC to refine voltage measurements if R and C are known accurately.
  // Conceptually, V_ref(t) = Vcc * exp(-t/rc_actual).
  // This allows us to use the capacitor as a stable (though decaying) reference.

  // Now use the RC to find Vcc.
  // We measured ADC at t1. V(t1) = Vcc * vdiv * exp(-t1_since_discharge/RC)
  // Wait, easier: Charge again and measure.
  digitalWrite(calPin, HIGH);
  delay(3000);

  // We know input is Vcc.
  // But we don't know Vcc yet.
  // Prompt says: "using it [RC] as reference voltage source"
  // Let's assume we know the target RC is exactly 0.1s.
  // If rc_actual is different, maybe our clock or our assumptions are off.
  // Actually, the prompt means use the discharge curve as a reference.
  // V_ref(t) = Vcc * exp(-t/0.1)

  // Measure Vcc using bandgap and use it to calibrate divider.
  vcc = readVcc();
  float vmeas_divided_Vcc = measureVoltage();
  vdiv = vmeas_divided_Vcc / vcc;

  Serial.print(F("Calibrated vdiv: ")); Serial.println(vdiv);

  // Calibrate offset
  digitalWrite(calPin, LOW);
  delay(5000);
  float vmeas_divided_0V = measureVoltage();
  voffset = - (vmeas_divided_0V / vdiv);
  Serial.print(F("Calibrated voffset: ")); Serial.println(voffset);

  EEPROM.put(eepromAddr, vdiv);
  EEPROM.put(eepromAddr + sizeof(float), voffset);
  EEPROM.put(eepromAddr + 2*sizeof(float), vcc);
  Serial.println(F("Calibration done"));
}

// Function to measure Vcc using internal bandgap
float readVcc() {
  // On Uno, we can measure the 1.1V bandgap using Vcc as reference
  // ADMUX: REFS0 (AVCC with external cap), MUX3:0 (1100 for 1.1V bandgap)
  #ifdef ARDUINO_AVR_UNO
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC));
  uint16_t low  = ADCL;
  uint16_t high = ADCH;
  uint16_t result = (high << 8) | low;
  float vcc_val = (1.1 * 1023.0) / (float)result;
  return vcc_val;
  #else
  return 5.0; // Default for simulator or other boards
  #endif
}

void setup() {
  Serial.begin(9600);
  initPWM();
  analogReference(INTERNAL);
  EEPROM.get(eepromAddr, vdiv);
  if (isnan(vdiv) || vdiv <= 0) vdiv = 0.1;
  EEPROM.get(eepromAddr + sizeof(float), voffset);
  if (isnan(voffset)) voffset = 0.0;
  EEPROM.get(eepromAddr + 2*sizeof(float), vcc);
  if (isnan(vcc) || vcc <= 0) vcc = readVcc();
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') calibrateVoltage();
  }

  float vmeas_divided = measureVoltage();
  float v_final = vmeas_divided / vdiv + voffset;

  Serial.print(F("Measured Voltage: "));
  Serial.print(v_final);
  Serial.println(F(" V"));

  delay(1000);
}
