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
  void loopstop();
  void start(int* melody, int* timing, int len);
  void loopstart(int* lmelody, int* ltiming, int llen);
  void update();
 private:
  uint8_t _pin;
  int* _melody;
  int* _lmelody;
  int* _timing;
  int* _ltiming;
  int _melsize,  _curpos, _pflag;
  int _lmelsize,  _lcurpos, _lflag;
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

// This class runs a display board
class Pb_display : public Pb_outputs
{
 public:
    Pb_display(uint8_t dataPin, uint8_t clkPin, uint8_t latchPin);
    void print_number(int num);
};


// This class controls a motor controller board.
class Pb_motor
{
 public:
  Pb_motor(uint8_t pin1, uint8_t pin2);
  void test_loop(); 
  void set_speed(int mspeed);
  void forward(int mspeed);
  void forward();
  void back(int mspeed);
  void back();
  void coast();
  void stop();
 private:
  int time = 1000; 
  int mspeed1 = 64; 
  uint_8 motor[2];  
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


// Test for passing function pointers as arguments
void testfunc (void (*f)(), int );


// This is used to execute a function in a timed sequence
// for an array of integers and timelengths as arguments.
class Pb_timedevent
{
 public:
  Pb_timedevent( void (*func)(int ) );
  void stop();
  void loopstop();
  void start(int* iarray, int* timing, int len);
  void loopstart(int* liarray, int* ltiming, int llen);
  void update();
 private:
  void (*_f)(int );
  int* _iarray;
  int* _liarray;
  int* _timing;
  int* _ltiming;
  int _arrsize, _curpos, _pflag;
  int _larrsize, _lcurpos, _lflag;
  unsigned long _curtime;
};

#endif
