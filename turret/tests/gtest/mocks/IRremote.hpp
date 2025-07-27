#ifndef IRREMOTE_HPP
#define IRREMOTE_HPP

class Stream;

#define ENABLE_LED_FEEDBACK 1
#define VERSION_IRREMOTE "mocked"
#ifndef STR
#define STR(s) #s
#endif
#define IRDATA_FLAGS_IS_REPEAT 0

class IRrecv {
public:
  struct {
    int command;
    int flags;
  } decodedIRData;

  void enableIRIn();
  bool decode();
  void resume();
  void begin(int, int);
};

struct decode_results {
  unsigned long value;
};

extern decode_results results;
extern IRrecv IrReceiver;

void printActiveIRProtocols(Stream*);

#endif // IRREMOTE_HPP
