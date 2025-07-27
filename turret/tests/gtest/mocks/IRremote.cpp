#include "IRremote.hpp"
#include "Arduino.h"

decode_results results;
IRrecv IrReceiver;

void IRrecv::enableIRIn() {}
bool IRrecv::decode() { return false; }
void IRrecv::resume() {}
void IRrecv::begin(int, int) {}

void printActiveIRProtocols(Stream*) {}
