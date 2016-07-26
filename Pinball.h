/*
  Pinball.h - Library for Spice 2014 engineering camp.
  v1.0 was created by Dileep V. Reddy, May 24, 2014.
  View README for version details.

  This project has been released under a Modified MIT License, the text
  of which is presented in the accopanying file LICENSE.md.
  Future versions of the MIT license may not automatically apply.

  Meant for ATMEGA168/328P Port numbers.
*/


#ifndef Pinball_h
#define Pinball_h

#include "Arduino.h"

#include "notes.h"

#include <inttypes.h>

// This is used to play an array of notes on a speaker
// for an array of timelengths. Uses tone() and noTone().
class Pb_speaker
{
 public:
  Pb_speaker(uint8_t pin);
  void stop();
  void loopstop();
  void beatstop();
  void start(int* melody, int* timing, int len);
  void loopstart(int* lmelody, int* ltiming, int llen);
  void startbeat(int* melody, int beat_t, int len);
  void update();
 private:
  uint8_t _pin;
  int* _melody;
  int* _lmelody;
  int* _bmelody;
  int* _timing;
  int* _ltiming;
  int _melsize,  _curpos, _pflag;
  int _lmelsize,  _lcurpos, _lflag;
  int _bmelsize, _bcurpos;
  int _beattime,  _bflag;
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
 private:
    byte display_array[2];
};


// This class controls a motor controller board.
// Avoid pwm pins 3 and 11, as there is timer conflict with Tone()
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
  int time; 
  int mspeed1; 
  uint8_t motor[2];  
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
  boolean _flag, _oldval;
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


// This is used to interface with 4051 multiplexers
// For digital only for now
class Pb_muxin
{
 public:
  Pb_muxin(uint8_t pinA, uint8_t pinB, uint8_t pinC, uint8_t COMpin);
  boolean probe(int snum);
  byte read();
 private:
  uint8_t _pinA, _pinB, _pinC, _COMpin;
  volatile uint8_t *_Aport, *_Bport, *_Cport;
};


// The following is from TM1637Display Library by avishorp@gmail.com
// with modifications and a class name change

#define SEG_A   0b00000001
#define SEG_B   0b00000010
#define SEG_C   0b00000100
#define SEG_D   0b00001000
#define SEG_E   0b00010000
#define SEG_F   0b00100000
#define SEG_G   0b01000000



class Pb_scoreboard {

public:
  //! Initialize a Pb_scoreboard object, setting the clock and
  //! data pins.
  //!
  //! @param pinClk - The number of the digital pin connected to the clock pin of the module
  //! @param pinDIO - The number of the digital pin connected to the DIO pin of the module
  Pb_scoreboard(uint8_t pinClk, uint8_t pinDIO);
  
  //! Sets the brightness of the display.
  //!
  //! The setting takes effect when a command is given to change the data being
  //! displayed.
  //!
  //! @param brightness A number from 0 (lowes brightness) to 7 (highest brightness)
  void setBrightness(uint8_t brightness);
  
  //! Display arbitrary data on the module
  //!
  //! This function receives raw segment values as input and displays them. The segment data
  //! is given as a byte array, each byte corresponding to a single digit. Within each byte,
  //! bit 0 is segment A, bit 1 is segment B etc.
  //! The function may either set the entire display or any desirable part on its own. The first
  //! digit is given by the @ref pos argument with 0 being the leftmost digit. The @ref length
  //! argument is the number of digits to be set. Other digits are not affected.
  //!
  //! @param segments An array of size @ref length containing the raw segment values
  //! @param length The number of digits to be modified
  //! @param pos The position from which to start the modification (0 - leftmost, 3 - rightmost)
  void setSegments(const uint8_t segments[], uint8_t length = 4, uint8_t pos = 0);
  
  //! Displayes a decimal number
  //!
  //! Dispalyes the given argument as a decimal number
  //!
  //! @param num The number to be shown
  //! @param leading_zero When true, leading zeros are displayed. Otherwise unnecessary digits are
  //!        blank
  //! @param length The number of digits to set. The user must ensure that the number to be shown
  //!        fits to the number of digits requested (for example, if two digits are to be displayed,
  //!        the number must be between 0 to 99)
  //! @param pos The position least significant digit (0 - leftmost, 3 - rightmost)
  void showdisplay(int num, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);
  
  //! Translate a single digit into 7 segment code
  //!
  //! The method accepts a number between 0 - 15 and converts it to the
  //! code required to display the number on a 7 segment display.
  //! Numbers between 10-15 are converted to hexadecimal digits (A-F)
  //!
  //! @param digit A number between 0 to 15
  //! @return A code representing the 7 segment image of the digit (LSB - segment A;
  //!         bit 6 - segment G; bit 7 - always zero)
  uint8_t encodeDigit(uint8_t digit);

  // same as encodeDigit(), but brightens decimal point
  uint8_t encodeDigitp(uint8_t digit);  // Spice modification

  // Updates m_partition private variable
  // This is used by predisplay() and postdisplay()
  // m_partition marks the spot with a decimal point
  // 0 => decimal point, use 1, 2, 3, 4 for left to right positions
  void setpartition(uint8_t par); // Spice modification

  // Display number to the left of decimal point
  void predisplay(int num, bool leading_zero = false); // Spice modification

  // Display number to the right of decimal point  
  void postdisplay(int num, bool leading_zero = false);  // Spice modification

  void blankdisplay(); // Spice modification

  void blankpredisplay(); // Spice modification

  void blankpostdisplay(); // Spice modification

  
protected:
   void bitDelay();
   
   void start();
   
   void stop();
   
   bool writeByte(uint8_t b);
   
private:
	uint8_t m_pinClk;
	uint8_t m_pinDIO;
	uint8_t m_brightness;
	uint8_t m_partition;   // Spice modification
};

#define TM_DOT          0x80

#define TM_MINUS        0x40
#define TM_PLUS         0x44
#define TM_BLANK        0x00
#define TM_DEGREES      0x63
#define TM_UNDERSCORE   0x08
#define TM_EQUALS       0x48
#define TM_CHAR_ERR     0x49

uint8_t inline TM1637_map_char(const char ch)
{
    uint8_t rc = 0;

    switch (ch)
    {
        case '-': rc = TM_MINUS; break;
        case '+': rc = TM_PLUS; break;
        case ' ': rc = TM_BLANK; break;
        case '^': rc = TM_DEGREES; break;
        case '_': rc = TM_UNDERSCORE; break;
        case '=': rc = TM_EQUALS; break;
        default:
            break;
    }

    return rc;
}


// This is meant to be used with the Peak detector board to read an analog value.
// Meant to capture event size from Piezo-electric element.
// Accepts activity threshold and two wait times as arguments.
class Pb_peakpiezo
{
 public:
  Pb_peakpiezo(uint8_t APin, uint8_t rstPin, int thresh, int timeA, int timeB);
  void update();
  void reset();
  int read();
 private:
  uint8_t _APin, _rstPin;
  int _thresh, _timeA, _timeB;
  int _Aval, _Atemp;
  unsigned long _oldtA, _oldtB;
  uint8_t _state;
};



#endif

