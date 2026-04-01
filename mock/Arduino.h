
#ifndef ARDUINO_H
#define ARDUINO_H

#include <iostream>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define A0 14
#define INTERNAL 3

extern uint8_t DDRB;
extern uint8_t PORTB;
extern uint8_t TCCR1A;
extern uint8_t TCCR1B;
extern uint16_t OCR1A;
extern uint16_t OCR1AH;
extern uint16_t OCR1AL;
extern uint16_t ICR1;

#define WGM10 0
#define WGM11 1
#define WGM12 3
#define WGM13 4
#define CS10 0
#define COM1A1 7

#include <cmath>
#ifndef isnan
#define isnan(x) std::isnan(x)
#endif

#define F(x) x

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
int analogRead(int pin);
void analogReference(int mode);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long micros();
unsigned long millis();

struct SerialMock {
    void begin(int baud) {}
    void print(const char* s) { std::cout << s; }
    void print(float f) { printf("%f", f); }
    void print(double f) { printf("%f", f); }
    void print(long l) { std::cout << l; }
    void println(const char* s) { std::cout << s << std::endl; }
    void println(float f) { printf("%f\n", f); }
    void println(double f) { printf("%f\n", f); }
    void println(int i) { std::cout << i << std::endl; }
    void println(long l) { std::cout << l << std::endl; }
    int avail = 0;
    int available() { return avail; }
    char read() { avail = 0; return 'c'; }
};

extern SerialMock Serial;

struct EEPROMMock {
    float storage[128];
    EEPROMMock() { for(int i=0; i<128; i++) storage[i]=NAN; }
    void get(int addr, float& t) { t = storage[addr/4]; }
    void put(int addr, const float& t) { storage[addr/4] = t; }
};

extern EEPROMMock EEPROM;

// Physics Simulation Interface
void set_input_voltage(float voltage);

#endif
