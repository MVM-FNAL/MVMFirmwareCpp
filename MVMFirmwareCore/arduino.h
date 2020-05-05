#pragma once
#ifdef DOXYGEN
struct String {
   String() {}; // ctor
   String(const char*) {}; // ctor
   String(int) {}; // ctor
   String(float, int) {}; // ctor
   String operator+(String);
   String operator+(const char*);
   String operator+=(String);
   String operator+=(const char*);
   bool operator==(const char*);
   int toInt() const;
   float toFloat() const;
};
String operator+(const char*, String);
struct Serial {
   void begin(int);
   bool available();
   String readStringUntil(char);
   String readStringUntil(int);
   void print(const char*);
   void print(String);
   void println(const char*);
   void println(String);
} Serial;
struct Argument {};
struct Command {};
struct CommandError {};
struct SimpleCLI {
   void parse(String s);
};
typedef const char* cmd;
typedef const char* cmd_error;
#else
#include <Arduino.h>
#endif

