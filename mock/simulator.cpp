
#include "Arduino.h"
#include <chrono>

uint8_t DDRB = 0;
uint8_t PORTB = 0;
uint8_t TCCR1A = 0;
uint8_t TCCR1B = 0;
uint16_t OCR1A = 0;
uint16_t ICR1 = 65535;

SerialMock Serial;
EEPROMMock EEPROM;

float measurement_voltage = 0.0;
float filtered_pwm_voltage = 0.0;
bool pin10_high = false;
bool pin10_output = false;
float simulator_vdiv = 0.1; // Divider on the input: Vplus = Vmeas * vdiv

unsigned long simulated_micros = 0;
void set_input_voltage(float voltage) {
    measurement_voltage = voltage;
}

void pinMode(int pin, int mode) {
    if (pin == 10) pin10_output = (mode == OUTPUT);
}

void digitalWrite(int pin, int value) {
    if (pin == 10) pin10_high = (value == HIGH);
}

int digitalRead(int pin) {
    // For simplicity, during discharge we'll just check measurement_voltage
    if (pin == 10) {
        if (pin10_output) return pin10_high;
        else return (measurement_voltage > 0.55) ? HIGH : LOW;
    }
    return LOW;
}

int analogRead(int pin) {
    if (pin == A0) {
        // Vplus = measurement_voltage * simulator_vdiv
        // Vminus = filtered_pwm_voltage
        // Vout = Vplus - Vminus
        float v_plus = measurement_voltage * simulator_vdiv;
        float v_out = v_plus - filtered_pwm_voltage;

        // ADC 10-bit, 1.1V reference
        int adc = (int)(v_out * 1023.0 / 1.1);

        if (adc > 1023) adc = 1023;
        if (adc < 0) adc = 0;
        return adc;
    }
    return 0;
}

void delay(unsigned long ms) {
    simulated_micros += ms * 1000;
    float pwm_target = (float)OCR1A / (float)ICR1 * 5.0; // Assume 5V VCC for PWM
    filtered_pwm_voltage = pwm_target; // Instant settling

    // Capacitor simulation for pin 10 calibration
    if (pin10_output) {
        if (pin10_high) {
            measurement_voltage = 5.0; // Charging to VCC (5V)
        } else {
            // Discharging: V = V0 * exp(-t/RC)
            // RC = 0.1s (100uF * 1k)
            for (unsigned int i = 0; i < ms; ++i) {
                measurement_voltage *= exp(-1.0 / 100.0);
            }
        }
    }
}

void delayMicroseconds(unsigned int us) {
    simulated_micros += us;
    delay(0);
}

unsigned long micros() {
    return simulated_micros;
}

unsigned long millis() {
    return micros() / 1000;
}

void analogReference(int mode) {}

extern void setup();
extern void loop();

int main() {
    setup();
    // Calibration first
    std::cout << "--- STARTING CALIBRATION ---" << std::endl;
    Serial.avail = 1; // Trigger calibration in loop()
    loop(); // This will call calibrateVoltage()

    std::cout << "--- STARTING MEASUREMENTS ---" << std::endl;

    pin10_output = false; // Stop pin10 from overriding measurement_voltage
    for (int i = 0; i < 20; ++i) {
        float input_v = i * 1.0;
        set_input_voltage(input_v);
        extern float measureVoltage();
        extern float vdiv;
        extern float voffset;
        float v = measureVoltage();
        float corrected_v = v / vdiv + voffset;
        printf("SIM INPUT: %f V, MEASURED: %f V (vdiv: %f, voffset: %f)\n", input_v, corrected_v, vdiv, voffset);
    }
    return 0;
}
