#ifndef ARDUINO_H
#define ARDUINO_H

#include <cstdint>

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7

void pinMode(int, int);
void digitalWrite(int, int);
int analogRead(int);
long random(long);
long random(long, long);
void delay(int);

#define F(str) str

class Stream {
public:
  void begin(int);
  void print(const char*);
  void print(int);
  void println(const char*);
  void println(int);
  void println();
  operator bool();
};

extern Stream Serial;

#endif // ARDUINO_H
