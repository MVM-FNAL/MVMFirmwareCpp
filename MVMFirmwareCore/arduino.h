#pragma once
#ifdef DOXYGEN
struct Serial {} Serial;
//typedef const char* String;
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
   //String readStringUntil(char);
};
String operator+(const char*, String);
struct Argument {};
struct Command {};
struct CommandError {};
struct SimpleCLI {};
typedef const char* cmd;
typedef const char* cmd_error;
#else
#include <Arduino.h>
#endif

