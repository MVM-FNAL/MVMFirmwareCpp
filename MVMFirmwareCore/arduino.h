#ifdef DOXYGEN
struct Serial {
  void begin();
  void print();
  void println();
  int readStringUntil();
};
void digitalWrite();
void ledcWrite();
#else
#include <Arduino.h>
#endif
