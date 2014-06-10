/*
  Pinball.h - Library for Spice 2014 engineering camp.
  v1.0 was created by Dileep V. Reddy, May 24, 2014.
  View README for version details.
  No release license yet at the time of this writing.
  Meant for ATMEGA168/328P Port numbers.
*/


#ifndef Pinball_h
#define Pinball_h

#include "Arduino.h"

#include "notes.h"

// This is used to play an array of notes on a speaker
// for an array of timelengths. Uses tone() and noTone().
class Pb_speaker
{
 public:
  Pb_speaker(uint8_t pin);
  void stop();
  void start(int* melody, int* timing, int len);
  void update();
 private:
  uint8_t _pin;
  int* _melody;
  int* _timing;
  int _melsize, _curpos, _pflag;
  unsigned long _curtime;
};


// This is meant to be used with 74HC595 shift registers.
// Its faster than shiftOut() as it doesn't use digitalWrite().
// Accepts number of shift registers in series as the final argument.
class Pb_outputs
{
 public:
  Pb_outputs(uint8_t dataPin, uint8_t clkPin, uint8_t latchPin, uint8_t numregs);
  void update(byte* bitarray);
 private:
  uint8_t _dpin, _cpin, _lpin, _nregs;
  byte* _barray;
  volatile uint8_t *_dport, *_cport, *_lport;
};


// This class is for debouncing digital inputs shorting to ground.
// Push buttons, for example. Use for toggles only.
class Pb_switch
{
 public:
  Pb_switch(uint8_t dtime);
  boolean pushed(boolean val);
  boolean released(boolean val);
 private:
  uint8_t _dt;
  unsigned long _ctime;
  boolean _flag;
};


// Simple millisecond stopwatch.
class Pb_stopwatch
{
 public:
  Pb_stopwatch();
  void start();
  void stop();
  unsigned long time();
 private:
  unsigned long _stime, _time;
  boolean _flag;
};


#endif
