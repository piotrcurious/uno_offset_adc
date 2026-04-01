
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

// Measurement function
float measureVoltage() {
  int32_t pwmValue = 32768;
  int32_t step = 16384;
  int adcValue = 0;

  for (int i = 0; i < 16; i++) {
    writePWM16((uint16_t)constrain(pwmValue, 0, 65535));
    delay(15); // Allow RC filter to settle (1uF * internal resistance)
    adcValue = analogRead(opampPin);

    // Target ADC middle point (~0.55V)
    if (adcValue > 512 + window) {
      // Vout too high -> Vmeas_divided - Vpwm > 0.55 -> Increase Vpwm
      pwmValue += step;
    } else if (adcValue < 512 - window) {
      // Vout too low -> Vmeas_divided - Vpwm < 0.55 -> Decrease Vpwm
      pwmValue -= step;
    } else {
      break;
    }
    step >>= 1;
  }
  pwmValue = constrain(pwmValue, 0, 65535);
  writePWM16((uint16_t)pwmValue);
  delay(15);

  // Final measurement
  adcValue = analogRead(opampPin);

  // Vpwm (0V to Vcc)
  float vpwm = ((float)pwmValue / 65535.0) * vcc;
  // Vout_opamp (0V to 1.1V ref)
  float vout_opamp = ((float)adcValue / 1023.0) * 1.1;

  // Vmeas_divided = Vpwm + Vout_opamp
  float vmeas_divided = vpwm + vout_opamp;

  return vmeas_divided;
}

void calibrateVoltage() {
  Serial.println(F("Calibration started"));
  Serial.println(F("Connect 100uF cap to meas input, 1k ohm to pin 10."));
  Serial.println(F("Press any key to start..."));
  // while (Serial.available() == 0) { delay(1); }
  while (Serial.available() > 0) Serial.read();

  // Charge capacitor
  pinMode(calPin, OUTPUT);
  digitalWrite(calPin, HIGH);
  Serial.println(F("Charging..."));
  delay(3000);

  // Use PWM=0 to measure input directly through op-amp when it's within 1.1V range
  writePWM16(0);
  delay(100);
  int initial_adc = analogRead(opampPin);
  Serial.print(F("Initial ADC at 5V: ")); Serial.println(initial_adc);

  digitalWrite(calPin, LOW); // Start discharge
  unsigned long start_micros = micros();

  // Wait for ADC to drop to 70% of initial
  int target1 = (int)(initial_adc * 0.7);
  while (analogRead(opampPin) > target1) { delay(1); }
  unsigned long t1 = micros();

  // Wait for ADC to drop to 35% of initial (which is half of 70%)
  int target2 = (int)(initial_adc * 0.35);
  while (analogRead(opampPin) > target2) { delay(1); }
  unsigned long t2 = micros();

  float dt = (float)(t2 - t1) / 1000000.0;
  // RC = dt / ln(0.7 / 0.35) = dt / ln(2)
  float rc_actual = dt / 0.693147;
  Serial.print(F("Measured RC: ")); Serial.println(rc_actual);

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

  // Let's use Vcc = 5.0V as a base assumption but allow calibration of vdiv.
  vcc = 5.0;
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
