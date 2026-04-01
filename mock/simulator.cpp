
/*
 * Arduino Hardware Physics Simulator
 * Simulates a differential op-amp circuit with input voltage divider,
 * 16-bit PWM RC filter, and capacitor discharge for calibration.
 */
#include "Arduino.h"
#include <chrono>
#include <random>

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
float pwm_target_voltage = 0.0;
bool pin10_high = false;
bool pin10_output = false;
float simulator_vdiv = 0.1; // Divider on the input: Vplus = Vmeas * vdiv

std::default_random_engine generator;
std::normal_distribution<float> noise_dist(0.0, 0.001); // 1mV RMS noise

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
        // Corrected Circuit:
        // Vplus = measurement_voltage * simulator_vdiv
        // Vminus = filtered_pwm_voltage (16-bit PWM, RC filtered)
        // Vout = Vplus - Vminus
        // A0 = Vout
        float v_plus = measurement_voltage * simulator_vdiv;
        float v_out = v_plus - filtered_pwm_voltage;

        // Add noise
        v_out += noise_dist(generator);

        // ADC 10-bit, 1.1V reference
        int adc = (int)(v_out * 1023.0 / 1.1);

        // Physical update during polling
        simulated_micros += 100; // 100us per analogRead
        // Small settling during poll
        float rc_pwm = 0.040;
        filtered_pwm_voltage = pwm_target_voltage + (filtered_pwm_voltage - pwm_target_voltage) * exp(-0.1 / rc_pwm);
        if (pin10_output && !pin10_high) {
            measurement_voltage *= exp(-0.1 / 100.0);
        }

        if (adc > 1023) adc = 1023;
        if (adc < 0) adc = 0;
        return adc;
    }
    return 0;
}

void delay(unsigned long ms) {
    if (ms == 10) { // Special case for calibration wait
        Serial.avail = 1;
    }
    simulated_micros += ms * 1000;
    pwm_target_voltage = (float)OCR1A / (float)ICR1 * 5.0; // Assume 5V VCC for PWM

    // PWM RC Filter simulation:
    // RC = internal resistance * 1uF.
    // Assume internal resistance = 40 ohms -> RC = 40us.
    // Settling time is about 5*RC = 200us.
    // Since delay is in ms, it usually settles instantly unless ms=0 or very small.
    // Let's use a smaller time step for settling if needed.
    float rc_pwm = 0.040; // 40us in ms
    if (ms > 0) {
        // For ms > 0, we can assume it reaches target or use formula:
        // V = Vtarget + (Vstart - Vtarget) * exp(-t/RC)
        filtered_pwm_voltage = pwm_target_voltage + (filtered_pwm_voltage - pwm_target_voltage) * exp(-(float)ms / rc_pwm);
    }

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

/**
 * Main simulation loop
 * 1. Runs the Arduino setup()
 * 2. Triggers the calibration routine
 * 3. Sweeps through input voltages and compares simulator truth with Arduino measurement.
 */
int main() {
    setup();
    // Calibration first
    std::cout << "--- STARTING CALIBRATION ---" << std::endl;

    // Use a separate thread or just manually advance state
    // In our simplified mock, we can just call it and make sure Serial.avail is set when needed.

    Serial.avail = 1; // Trigger 'c' in loop()
    // This will enter calibrateVoltage(). Inside, it will call analogRead which increments time.
    // We need to make sure Serial.avail is reset and then set again for the "Press any key"

    // We can't easily do async input in this simple main.
    // Let's modify SerialMock to return 'c' then something else.

    loop();

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
