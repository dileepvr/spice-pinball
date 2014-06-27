/*
  Pinball.cpp - Library for Spice 2014 engineering camp.
  v1.0 was created by Dileep V. Reddy, May 24, 2014.
  No release license yet at the time of this writing.
*/

#include "Pinball.h"
#include "Arduino.h"

#include "notes.h"


// Speaker input pin as argument
Pb_speaker::Pb_speaker(uint8_t pin)
{
  pinMode(pin, OUTPUT);
  _pin = pin;
  _pflag = 0;
  _lflag = 0;
}

// Halt currently playing melody
void Pb_speaker::stop()
{
  noTone(_pin);
  _pflag = 0;
  if (_lflag == 2) {   // If looptrack existed, restore it
    _lflag = 1;
  }
}


// Halt currently playing melody
void Pb_speaker::loopstop()
{
  noTone(_pin);
  _lflag = 0;
}

// Start playing supplied melody with timing array
// See also Pb_speaker.update()
void Pb_speaker::start(int* melody, int* timing, int len)
{
  if (_lflag == 1) { _lflag = 2; }   // If looptrack existed, pause it
  _melsize = len;
  _melody = melody;
  _timing = timing;
  _curtime = millis();
  noTone(_pin);
  _curpos = 0;
  if ((_melody[_curpos] != 0) && (_melsize > 0)) {
    tone(_pin, _melody[_curpos], _timing[_curpos]);
  }
  _pflag = 1;
}


// Start playing supplied loop melody with timing array
// See also Pb_speaker.update()
void Pb_speaker::loopstart(int* lmelody, int* ltiming, int llen)
{
  _lmelsize = llen;
  _lmelody = lmelody;
  _ltiming = ltiming;
  _curtime = millis();
  noTone(_pin);
  _lcurpos = 0;
  if ((_lmelody[_curpos] != 0) && (_lmelsize > 0)) {
    tone(_pin, _lmelody[_lcurpos], _ltiming[_lcurpos]);
  }
  _lflag = 1;
}


// This needs to be called inside main loop to advance
// through the notes in the melody array
void Pb_speaker::update()
{
  if (_lflag != 1) {         // If looptrack is paused or nonexistent
    if (_pflag == 1) {
      if (_curpos+1 == _melsize) {
	_pflag = 0;
	if (_lflag == 2) {_lflag = 1; }   // Restore looptrack if existed
      } else {
	if ((millis() - _curtime) > _timing[_curpos]) {
	  _curpos++;
	  _curtime = millis();
	  if (_melody[_curpos] != 0) {
	    tone(_pin, _melody[_curpos], _timing[_curpos]);
	  }
	}
      }
    }
  } else {                 // Else continue with looptrack
    if (_lflag == 1) {
      if (_lcurpos+1 == _lmelsize) {
	_lcurpos = 0;
      }
      if ((millis() - _curtime) > _ltiming[_lcurpos]) {
	_lcurpos++;
	_curtime = millis();
	if (_lmelody[_lcurpos] != 0) {
	  tone(_pin, _lmelody[_lcurpos], _ltiming[_lcurpos]);
	}
      }
    }
  }
}


// This is meant to be used with 74HC595 shift registers
// Its faster than shiftOut() as it doesn't use digitalWrite().
// Accepts number of shift registers in series as the final argument.
Pb_outputs::Pb_outputs(uint8_t dataPin, uint8_t clkPin, uint8_t latchPin, uint8_t numregs)
{
  pinMode(dataPin, OUTPUT);
  pinMode(clkPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  digitalWrite(clkPin, LOW);
  digitalWrite(latchPin, HIGH);
  
  _nregs = numregs;
  
  // This is where it gets specific to ATMEGA168/328P
  if (dataPin < 8) {_dpin = dataPin; _dport = &PORTD; }
  else {_dpin = dataPin - 8; _dport = &PORTB; }
  if (clkPin < 8) {_cpin = clkPin; _cport = &PORTD; }
  else {_cpin = clkPin - 8; _cport = &PORTB; }
  if (latchPin < 8) {_lpin = latchPin; _lport = &PORTD; }
  else {_lpin = latchPin - 8; _lport = &PORTB; }

}


// Call this with byte array to shift values out
void Pb_outputs::update(byte* bitarray)
{
  uint8_t icnt, jcnt;

  *_lport &= ~(1 << _lpin); // Latch low
  for (jcnt = 0; jcnt < _nregs; jcnt++) {
    _barray = bitarray + jcnt;
    for (icnt = 0; icnt < 8; icnt++) {
      // MSBFIRST ONLY
      if (*_barray & (1 << (7 - icnt)))
	{   *_dport |= (1 << _dpin);  } 
      else
	{   *_dport &= ~(1 << _dpin);  } 

      *_cport |= (1 << _cpin);   // Clock high
      *_cport &= ~(1 << _cpin);  // Clock low
    }
  }
  *_lport |= (1 << _lpin); // Latch high

}

// Set up a display using 3 pins.
Pb_display::Pb_display(uint8_t dataPin, uint8_t clkPin, uint8_t latchPin) : Pb_outputs(dataPin, clkPin, latchPin, 2)
{

}

// Definitions for the display function below.
// These are placeholders (needs fixing!)
const uint8 DISPLAY_NUMBER[] = {0b00000001, // 0
				0b00000010, // 1
				0b00000100, // 2
				0b00001000, // 3
				0b00010000, // 4
				0b00100000, // 5
				0b01000000, // 6
				0b10000000, // 7
				0b00000011, // 8
				0b00001100, // 9
				0b11111111}; // Blank

// Prints an integer between 00-100 on the display.
Pb_display::print_number(int num)
{
  int digit1 = num % 10;
  int digit2 = (num % 100) / 10;
  byte display_array[2];
  display_array[0]=DISPLAY_NUMBER[digit1];
  display_array[1]=DISPLAY_NUMBER[digit2];
  update(display_array);
}


// Debounce time in milliseconds as argument
Pb_switch::Pb_switch(uint8_t dtime)
{
  _dt = dtime;
  _flag = false;
  _ctime = millis();
}

// To check if it was pushed down
boolean Pb_switch::pushed(boolean val)
{
  if (millis() - _ctime > _dt) {
    _flag = false;
  }
  if (!val && !_flag) {
    _flag = true;
    _ctime = millis();
    return true;
  } else {
    return false;
  }
}

// Motor controller board
// Pins must be PWM for speed control
Pb_motor::Pb_motor(uint8_t pin1, uint8_t pin2)
{ 
  time = 1000; //milliseconds delay for testing onyl
  mspeed1 = 64; // 0 - 255, speed
  motor[0] = pin1;
  motor[1] = pin2;
  pinMode(motor[0], OUTPUT);
  pinMode(motor[1], OUTPUT);
  digitalWrite(motor[0], LOW);
  digitalWrite(motor[1], LOW);
}

// for testing purposes
void Pb_motor::test_loop()
{
  Serial.println("Forward");
  move_front(mspeed1);
  delay(time);
  stop_motor();

  Serial.println("Backward"); 
  move_back(mspeed1);
  delay(time);
  stop_motor(); 
}

// sets speed for test_loop and argument free move functions.
// Speed will only update on next function call
void Pb_motor::set_speed(int mspeed)
{   
  mspeed1 = mspeed;
}

void Pb_motor::forward(int mspeed)
{   
   //digitalWrite(motor[0],HIGH);
   analogWrite(motor[0],mspeed); 
   digitalWrite(motor[1],LOW);

}

void Pb_motor::forward()
{   
   //digitalWrite(motor[0],HIGH);
   analogWrite(motor[0],mspeed1); 
   digitalWrite(motor[1],LOW);

}

void Pb_motor::back(int mspeed)
{   
   digitalWrite(motor[0],LOW); 
   //digitalWrite(motor[1], HIGH); 
   analogWrite(motor[1],mspeed);
}

void Pb_motor::back()
{   
   digitalWrite(motor[0],LOW); 
   //digitalWrite(motor[1], HIGH); 
   analogWrite(motor[1],mspeed1);
}

void Pb_motor::stop()
{
  digitalWrite(motor[0],LOW);
  digitalWrite(motor[1],LOW);
}

//untested!
void Pb_motor::coast()
{
  digitalWrite(motor[0],HIGH);
  digitalWrite(motor[1],HIGH);
}

// Simple millisecond stopwatch
Pb_stopwatch::Pb_stopwatch()
{
  _flag = false;
  _time = 0;
  _stime = 0;
}


void Pb_stopwatch::start()
{
  _flag = true;
  _stime = millis();
}

void Pb_stopwatch::stop()
{
  _flag = false;
  _time = millis() - _stime;
}

// Don't need to stop to check time elapsed
unsigned long Pb_stopwatch::time()
{
  if (_flag) { return (millis() - _stime); }
  else       { return _time; }
}


// Test for passing function pointers as arguments
void testfunc (void (*f)(), int val) {

  (*f)();

}


// Supply timed event function pointer as argument
// Function must return void and accept int arguement
Pb_timedevent::Pb_timedevent(void (*func)(int ) )
{
  _f = func;
  _pflag = 0;
  _lflag = 0;
}


// Halt timed event function execution
void Pb_timedevent::stop()
{
  _pflag = 0;
  if (_lflag == 2) {   // If looped event existed, restore it
    _lflag = 1;
  }
}

// Halt currently looping event sequence
void Pb_timedevent::loopstop()
{
  _lflag = 0;
}

// Start executing timed event function with integer
// argument and timing from supplied arrays
// See also Pb_timedevent.update()
void Pb_timedevent::start(int* iarray, int* timing, int len)
{
  if (_lflag == 1) { _lflag = 2; }   // If looped event existed, pause it
  _arrsize = len;
  _iarray = iarray;
  _timing = timing;
  _curtime = millis();
  _curpos = 0;

  if ((_arrsize > 0) && (_timing[_curpos] > -1)) {
    (*_f)(_iarray[_curpos]);
  }
  _pflag = 1;
}


// Start running supplied loop sequence with timing array
// See also Pb_speaker.update()
void Pb_timedevent::loopstart(int* liarray, int* ltiming, int llen)
{
  _larrsize = llen;
  _liarray = liarray;
  _ltiming = ltiming;
  _curtime = millis();
  _lcurpos = 0;

  if ((_larrsize > 0) && (_ltiming[_lcurpos] > -1)) {
    (*_f)(_liarray[_lcurpos]);
  }
  _lflag = 1;
}


// This needs to be called inside main loop to advance
// through the timed sequence
void Pb_timedevent::update()
{
  if (_lflag != 1) {   // If looped event is paused or nonexistent
    if (_pflag == 1) {
      if (_curpos+1 == _arrsize) {
	_pflag = 0;
	if (_lflag == 2) {_lflag = 1; }   // Restore looped event if existed
      } else {
	if ((millis() - _curtime) > _timing[_curpos]) {
	  _curpos++;
	  _curtime = millis();
	  if (_timing[_curpos] > -1) {
	    (*_f)(_iarray[_curpos]);
	  }
	}
      }
    }
  } else {             // Else continue with looped sequence
    if (_lflag == 1) {
      if ((millis() - _curtime) > _ltiming[_lcurpos]) {
	_lcurpos++;
	if (_lcurpos == _larrsize) {
	  _lcurpos = 0;
	}
	_curtime = millis();
	if (_ltiming[_lcurpos] > -1) {
	    (*_f)(_liarray[_lcurpos]);
	}
      }
    }
  }
}
