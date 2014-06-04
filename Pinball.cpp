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
}

// Halt currently playing melody
void Pb_speaker::stop()
{
  noTone(_pin);
  _pflag = 0;
}

// Start playing supplied melody with timing array
// See also Pb_speaker.update()
void Pb_speaker::start(int* melody, int* timing, int len)
{
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


// This needs to be called inside main loop to advance
// through the notes in the melody array
void Pb_speaker::update()
{
  if (_pflag == 1) {
    if (_curpos+1 == _melsize) {
      _pflag = 0;
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
