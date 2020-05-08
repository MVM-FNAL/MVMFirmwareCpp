#ifdef DOXYGEN
struct Serial {
  void begin();
  void print();
  void println();
  int readStringUntil();
};
void digitalWrite();
void ledcWrite();
vod malloc(); // memory allocation
#else
#include <Arduino.h>
#endif
