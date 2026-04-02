# uno_offset_adc
High-resolution voltage measurement using an offset-subtracting ADC technique on the Arduino Uno.

## Overview
This project implements a system to measure DC voltages with resolution exceeding the native 10-bit capability of the Arduino Uno. It uses a 16-bit PWM signal to create a coarse offset, which is subtracted from the input voltage using a differential op-amp circuit. The residual voltage is then measured by the Arduino's 10-bit ADC.

## Hardware Architecture
The system is designed with the following circuit topology:
- **Input Divider**: The input voltage passes through a voltage divider to bring it into a manageable range.
- **Op-Amp (Differential)**: A unity-gain op-amp acts as a subtractor.
    - **Non-Inverting Input (+)**: Connected to the divided input voltage.
    - **Inverting Input (-)**: Connected to Pin 9 (16-bit PWM) via an RC low-pass filter (1uF capacitor).
- **ADC Input**: The op-amp output is connected to Pin A0 through a current-limiting resistor. The ADC is configured to use the internal 1.1V reference.

## Measurement Algorithm
1. **Binary Search**: On each measurement, the system performs a binary search on the 16-bit PWM value (using Timer1) to bring the op-amp output as close as possible to the center of the 1.1V ADC range (approx. 0.55V).
2. **Settling**: Delays are included to allow the PWM RC filter to settle between steps.
3. **Oversampling**: After the PWM offset is converged, the ADC reads the residual voltage. Multiple samples (64x) are averaged to improve noise immunity.
4. **Recombination**: The total voltage is calculated as:
   `V_total = (V_pwm + V_adc_residual) / divider_ratio + offset_correction`

## Calibration Procedure
The device includes an automated calibration routine triggered by sending 'c' over Serial.
1. **RC Constant Determination**: By discharging a 100uF capacitor through a 1k resistor (connected to Pin 10), the system measures the discharge time to find the actual RC constant.
2. **Vcc Calibration**: The system measures the actual supply voltage (Vcc) using the internal bandgap reference.
3. **Divider & Offset Calibration**: Using the known 5V (Vcc) and 0V states of the calibration circuit, the system calculates the precise voltage divider ratio and op-amp offset, storing them in EEPROM.

## Testing & Simulation
A comprehensive C++ mock environment and physics simulator are provided in the `mock/` directory.
- `mock/Arduino.h`: Mocks Arduino registers, pins, and serial communication.
- `mock/simulator.cpp`: Simulates the op-amp physics, RC filter behavior, Gaussian noise, and capacitor discharge curves.
- `test_ino.cpp`: A utility to wrap and compile `.ino` sketches against the simulator using `g++`.

### Running the Tests
1. Compile the test runner: `g++ -o test_ino test_ino.cpp -I.`
2. Run the simulation for the main solution: `./test_ino solution.ino`

The simulator will perform a calibration and then sweep through a range of input voltages, comparing the "true" simulated voltage with the Arduino's measured result.
