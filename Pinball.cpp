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
  _bflag = 0;
}

// Halt currently playing melody
void Pb_speaker::stop()
{
  noTone(_pin);
  _pflag = 0;
  if (_bflag == 2) {          // If beattrack existed, restore it
    _bflag = 1;
  } else if (_lflag == 2) {   // If looptrack existed, restore it
    _lflag = 1;
  }
}


// Halt currently playing melody
void Pb_speaker::loopstop()
{
  noTone(_pin);
  _lflag = 0;
  _bflag = 0;
}

// Halt currently playing melody
void Pb_speaker::beatstop()
{
  noTone(_pin);
  _bflag = 0;
}

// Start playing supplied melody with timing array
// See also Pb_speaker.update()
void Pb_speaker::start(int* melody, int* timing, int len)
{
  if (_lflag == 1) { _lflag = 2; }   // If looptrack existed, pause it
  if (_bflag == 1) { _bflag = 2; }   // If beattrack existed, pause it
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


// Start playing supplied melody with timing beat
// See also Pb_speaker.update()
void Pb_speaker::startbeat(int* melody, int beat_t, int len)
{
  if (_lflag == 1) { _lflag = 2; }   // If looptrack existed, pause it
  _bmelsize = len;
  _bmelody = melody;
  _beattime = beat_t;
  _curtime = millis();
  noTone(_pin);
  _bcurpos = 0;
  if ((_bmelody[_bcurpos] != 0) && (_bmelsize > 0)) {
    tone(_pin, _bmelody[_bcurpos], _beattime);
  }
  _bflag = 1;
}


// This needs to be called inside main loop to advance
// through the notes in the melody array
void Pb_speaker::update()
{
  if ((_lflag != 1)&&(_bflag != 1)) { // If looptrack and beattrack are paused or nonexistent
    if (_pflag == 1) {
      if (_curpos == _melsize) {
	_pflag = 0;
	if (_bflag == 2) {_bflag = 1; } // Restore beattrack if existed
	else if (_lflag == 2) {_lflag = 1; } // Restore looptrack if existed
      } else {
	if ((millis() - _curtime) > _timing[_curpos]) {
	  _curpos++;
	  _curtime = millis();
	  if ((_curpos < _melsize) && (_melody[_curpos] != 0)) {
	    tone(_pin, _melody[_curpos], _timing[_curpos]);
	  }
	}
      }
    }
  } else if (_bflag == 1) {
    if (_bcurpos == _bmelsize) {
      _bflag = 0;
      if (_lflag == 2) {_lflag = 1; } // Restore looptrack if existed
    } else {
      if ((millis() - _curtime) > _beattime) {
	_bcurpos++;
	_curtime = millis();
	if ((_bcurpos < _bmelsize) && (_melody[_bcurpos] != 0)) {
	  tone(_pin, _bmelody[_bcurpos], _beattime);
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
  display_array[0] = 0b11111111;
  display_array[1] = 0b11111111;
  update(display_array);
}

// Definitions for the display function below.
// 
// These designations do NOT match the printed circuit board!
//  LOW = LED ON
// HIGH = LED OFF
//   ---0---
//  |       |     
//  5       1     
//  |       |     
//   ---6---
//  |       |     
//  7       3     
//  |       |     
//   ---2---
//          * 4                     76543210
const byte DISPLAY_NUMBER[] =    {0b01010000, // 0
				  0b11110101, // 1
				  0b00111000, // 2
				  0b10110000, // 3
				  0b10010101, // 4
				  0b10010010, // 5
				  0b00010010, // 6
				  0b11110100, // 7
				  0b00010000, // 8
				  0b10010000, // 9
				  0b11111111}; // Blank

// Prints an integer between 00-100 on the display.
void Pb_display::print_number(int num)
{
  int digit1 = num % 10;
  int digit2 = (num % 100) / 10;
  display_array[0]=DISPLAY_NUMBER[digit1];
  display_array[1]=DISPLAY_NUMBER[digit2];
  update(display_array);
}


// Debounce time in milliseconds as argument
Pb_switch::Pb_switch(uint8_t dtime)
{
  _dt = dtime;
  _flag = false;
  _oldval = true;
  _ctime = millis();
}

// To check if it was pushed down
boolean Pb_switch::pushed(boolean val)
{
  if (millis() - _ctime > _dt) {
    if (!val && _flag) { _flag = false; return true; }
    _flag = false;
  }
  if (!val && !_flag && _oldval) {
    _flag = true;
    _ctime = millis();
  }
  _oldval = val;
  return false;
}

// Motor controller board
// Pins must be PWM for speed control
// Avoid pins 3 and 11 if using the audio classes and/or Tone()
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
  forward(mspeed1);
  delay(time);
  stop();

  Serial.println("Backward"); 
  back(mspeed1);
  delay(time);
  stop(); 
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
      if (_curpos == _arrsize) {
	_pflag = 0;
	if (_lflag == 2) {_lflag = 1; }   // Restore looped event if existed
      } else {
	if ((millis() - _curtime) > _timing[_curpos]) {
	  _curpos++;
	  _curtime = millis();
	  if ((_curpos < _arrsize) && (_timing[_curpos] > -1)) {
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


// This is used to interface with 4051 multiplexers (8-channel)
// For digital only for now
Pb_muxin::Pb_muxin(uint8_t pinA, uint8_t pinB, uint8_t pinC, uint8_t COMpin)
{
  pinMode(pinA, OUTPUT); pinMode(pinB, OUTPUT); pinMode(pinC, OUTPUT);
  pinMode(COMpin, INPUT); digitalWrite(COMpin, HIGH);

  _COMpin = COMpin;

  // This is where it gets specific to ATMEGA168/328P
  if (pinA < 8) {_pinA = pinA; _Aport = &PORTD; }
  else {_pinA = pinA - 8; _Aport = &PORTB; }
  if (pinB < 8) {_pinB = pinB; _Bport = &PORTD; }
  else {_pinB = pinB - 8; _Bport = &PORTB; }
  if (pinC < 8) {_pinC = pinC; _Cport = &PORTD; }
  else {_pinC = pinC - 8; _Cport = &PORTB; }

}


// Read channel number snum (starts from zero)
boolean Pb_muxin::probe(int snum)
{
  if ((snum & 0x01))
    {   *_Aport |= (1 << _pinA);  } 
  else
    {   *_Aport &= ~(1 << _pinA);  } 
  if ((snum & 0x02))
    {   *_Bport |= (1 << _pinB);  } 
  else
    {   *_Bport &= ~(1 << _pinB);  } 
  if ((snum & 0x04))
    {   *_Cport |= (1 << _pinC);  } 
  else
    {   *_Cport &= ~(1 << _pinC);  } 

  return digitalRead(_COMpin);
  
}

// Read all 8 digital channels at once and return binary byte
byte Pb_muxin::read()
{
  uint8_t icnt;

  byte _dvals = 0x00;

  for (icnt = 0; icnt < 8; icnt++) {
    
    if( probe(icnt) ) {
      _dvals |= (1 << icnt);
    }

  }
  return _dvals;
  
}


// The following is from TM1637Display library by avishorp@gmail.com
// with modifications

extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
}


#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] = {
 // XGFEDCBA
  0b00111111,    // 0
  0b00000110,    // 1
  0b01011011,    // 2
  0b01001111,    // 3
  0b01100110,    // 4
  0b01101101,    // 5
  0b01111101,    // 6
  0b00000111,    // 7
  0b01111111,    // 8
  0b01101111,    // 9
  0b01110111,    // A
  0b01111100,    // B
  0b00111001,    // C
  0b01000111,    // D
  0b01111001,    // E
  0b01110001,    // F
  0b00000000     // NULL
  };

const uint8_t digitpToSegment[] = {
 // XGFEDCBA
  0b10111111,    // 0
  0b10000110,    // 1
  0b11011011,    // 2
  0b11001111,    // 3
  0b11100110,    // 4
  0b11101101,    // 5
  0b11111101,    // 6
  0b10000111,    // 7
  0b11111111,    // 8
  0b11101111,    // 9
  0b11110111,    // A
  0b11111100,    // B
  0b10111001,    // C
  0b11000111,    // D
  0b11111001,    // E
  0b11110001,    // F
  0b10000000     // NULL
  };



Pb_scoreboard::Pb_scoreboard(uint8_t pinClk, uint8_t pinDIO)
{
	// Copy the pin numbers
	m_pinClk = pinClk;
	m_pinDIO = pinDIO;
	m_partition = 0;
	m_brightness = 0xff;
	
	// Set the pin direction and default value.
	// Both pins are set as inputs, allowing the pull-up resistors to pull them up
    pinMode(m_pinClk, INPUT);
    pinMode(m_pinDIO,INPUT);
	digitalWrite(m_pinClk, LOW);
	digitalWrite(m_pinDIO, LOW);
}

void Pb_scoreboard::setBrightness(uint8_t brightness)
{
	m_brightness = brightness;
}

void Pb_scoreboard::setSegments(const uint8_t segments[], uint8_t length, uint8_t pos)
{
    // Write COMM1
	start();
	writeByte(TM1637_I2C_COMM1);
	stop();
	
	// Write COMM2 + first digit address
	start();
	writeByte(TM1637_I2C_COMM2 + (pos & 0x03));
	
	// Write the data bytes
	for (uint8_t k=0; k < length; k++) 
	  writeByte(segments[k]);
	  
	stop();

	// Write COMM3 + brightness
	start();
	writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
	stop();
}
 
void Pb_scoreboard::showdisplay(int num, bool leading_zero, uint8_t length, uint8_t pos)
{
	uint8_t digits[4];
	const static int divisors[] = { 1, 10, 100, 1000 };
	bool leading = true;
	
	for(int8_t k = 0; k < 4; k++) {
	    int divisor = divisors[4 - 1 - k];
		int d = num / divisor;
		
		if (d == 0) {
		  if (leading_zero || !leading || (k == 3))
		    digits[k] = encodeDigit(d);
	      else
		    digits[k] = 0;
		}
		else {
			digits[k] = encodeDigit(d);
			num -= d * divisor;
			leading = false;
		}
	}
	
	setSegments(digits + (4 - length), length, pos);
}

void Pb_scoreboard::bitDelay()
{
	delayMicroseconds(50);
}
   
void Pb_scoreboard::start()
{
  pinMode(m_pinDIO, OUTPUT);
  bitDelay();
}
   
void Pb_scoreboard::stop()
{
	pinMode(m_pinDIO, OUTPUT);
	bitDelay();
	pinMode(m_pinClk, INPUT);
	bitDelay();
	pinMode(m_pinDIO, INPUT);
	bitDelay();
}
  
bool Pb_scoreboard::writeByte(uint8_t b)
{
  uint8_t data = b;

  // 8 Data Bits
  for(uint8_t i = 0; i < 8; i++) {
    // CLK low
    pinMode(m_pinClk, OUTPUT);
    bitDelay();
    
	// Set data bit
    if (data & 0x01)
      pinMode(m_pinDIO, INPUT);
    else
      pinMode(m_pinDIO, OUTPUT);
    
    bitDelay();
	
	// CLK high
    pinMode(m_pinClk, INPUT);
    bitDelay();
    data = data >> 1;
  }
  
  // Wait for acknowledge
  // CLK to zero
  pinMode(m_pinClk, OUTPUT);
  pinMode(m_pinDIO, INPUT);
  bitDelay();
  
  // CLK to high
  pinMode(m_pinClk, INPUT);
  bitDelay();
  uint8_t ack = digitalRead(m_pinDIO);
  if (ack == 0)
    pinMode(m_pinDIO, OUTPUT);
	
	
  bitDelay();
  pinMode(m_pinClk, OUTPUT);
  bitDelay();
  
  return ack;
}


void Pb_scoreboard::setpartition(uint8_t par)
{
  if (par > 4) { m_partition = 0; }
  else { m_partition = par; }
}

void Pb_scoreboard::predisplay(int num, bool leading_zero)
{
  uint8_t digits[4], length, pos;
  const static int divisors[] = { 1, 10, 100, 1000 };
  bool leading = true;

  pos = 0;
  if( m_partition == 0 ) { length = 4; }
  else { length = m_partition; }
	
  for(int8_t k = 0; k < 4; k++) {
    int divisor = divisors[4 - 1 - k];
    int d = num / divisor;
	  
    if (d == 0) {
      if (leading_zero || !leading || (k == 3))
	if (m_partition == 0 || (k != 3) ) {
	  digits[k] = encodeDigit(d);
	} else {
	  digits[k] = encodeDigitp(d);	  
	}
      else
	digits[k] = 0;
    }
    else {
      if (m_partition == 0 || (k != 3) ) {
	digits[k] = encodeDigit(d);
      } else {
	digits[k] = encodeDigitp(d);	  
      }      
      num -= d * divisor;
      leading = false;
    }
  }
	
  setSegments(digits + (4 - length), length, pos);
}


void Pb_scoreboard::postdisplay(int num, bool leading_zero)
{
  uint8_t digits[4], length, pos;
  const static int divisors[] = { 1, 10, 100, 1000 };
  bool leading = true;

  if( m_partition == 0 ) { length = 4; pos = 0; }
  else { length = 4 - m_partition; pos = m_partition; }

  if( pos < 4) {
    for(int8_t k = 0; k < 4; k++) {
      int divisor = divisors[4 - 1 - k];
      int d = num / divisor;
	  
      if (d == 0) {
	if (leading_zero || !leading || (k == 3))
	  digits[k] = encodeDigit(d);
	else
	  digits[k] = 0;
      }
      else {
	digits[k] = encodeDigit(d);
	num -= d * divisor;
	leading = false;
      }
    }
	
    setSegments(digits + (4 - length), length, pos);
  }
}

void Pb_scoreboard::blankdisplay() {
  uint8_t data[] = { 0x00, 0x00, 0x00, 0x00 };
  setSegments(data);
}

void Pb_scoreboard::blankpredisplay() {

  uint8_t length, pos;
  uint8_t digits[] = {0x00, 0x00, 0x00, 0x80};

  pos = 0;
  if( m_partition == 0 ) { length = 4; }
  else { length = m_partition; }
	
  setSegments(digits + (4 - length), length, pos);  
  
}

void Pb_scoreboard::blankpostdisplay() {

  uint8_t length, pos;
  uint8_t digits[] = {0x00, 0x00, 0x00, 0x00};

  if( m_partition == 0 ) { length = 4; pos = 0; }
  else { length = 4 - m_partition; pos = m_partition; }

  setSegments(digits + (4 - length), length, pos);    
}
  
uint8_t Pb_scoreboard::encodeDigit(uint8_t digit)
{
	return digitToSegment[digit & 0x0f];
}

uint8_t Pb_scoreboard::encodeDigitp(uint8_t digit)
{
	return digitpToSegment[digit & 0x0f];
}

