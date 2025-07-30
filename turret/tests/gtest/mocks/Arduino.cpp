#include "Arduino.h"

unsigned long current_time = 0;

void pinMode(int, int) {}
void digitalWrite(int, int) {}
int analogRead(int) { return 0; }
long random(long) { return 0; }
long random(long, long) { return 0; }
void delay(int) {}
unsigned long millis() { return current_time++; }
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


Stream Serial;

void Stream::begin(int) {}
void Stream::print(const char*) {}
void Stream::print(int) {}
void Stream::println(const char*) {}
void Stream::println(int) {}
void Stream::println() {}
Stream::operator bool() { return true; }
