#include "Arduino.h"

void pinMode(int, int) {}
void digitalWrite(int, int) {}
int analogRead(int) { return 0; }
long random(long) { return 0; }
long random(long, long) { return 0; }
void delay(int) {}

Stream Serial;

void Stream::begin(int) {}
void Stream::print(const char*) {}
void Stream::print(int) {}
void Stream::println(const char*) {}
void Stream::println(int) {}
void Stream::println() {}
Stream::operator bool() { return true; }
