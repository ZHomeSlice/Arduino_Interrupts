#ifndef Interrupts
#define  Interrupts

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#define LIBRARY_VERSION	1.1.1
#include <avr/pgmspace.h>

// Macros
#define runAfter(t)	for (static unsigned long _ATimer; (unsigned long)(millis() - _ATimer) >= (t); _ATimer = millis())
#define runEvery(t)	for (static unsigned long _ETimer; (unsigned long)(millis() -_ETimer) >= (t); _ETimer += (t))
#define DeBounce(t) for (static unsigned long DeBounceTimer; (unsigned long)(Time - DeBounceTimer) >= (t); DeBounceTimer = Time)


// Bionary Link
#define PinB0 1 // 1
#define PinB1 2 // 10
#define PinB2 4 // 100
#define PinB3 8 // 1000 etc.
#define PinB4 16
#define PinB5 32
#define PinB6 64
#define PinB7 128
#define PinB8 256
#define PinB9 512
#define PinB10 1024
#define PinB11 2048
#define PinB12 4096
#define PinB13 8192
#define PinB14 65536
#define PinB15 131072
#define PinB16 262144
#define PinB17 524288
#define PinB18 1048576
#define PinB19 2097152
#define PinB20 4194304
#define PinB21 8388608

#define PinBA0 65536
#define PinBA1 131072
#define PinBA2 262144
#define PinBA3 524288
#define PinBA4 1048576
#define PinBA5 2097152
#define PinBA6 4194304
#define PinBA7 8388608

// Bit to Pin#
#define Bit0 0
#define Bit1 1
#define Bit2 2
#define Bit3 3
#define Bit4 4
#define Bit5 5
#define Bit6 6
#define Bit7 7
#define Bit8 8
#define Bit9 9
#define Bit10 10
#define Bit11 11
#define Bit12 12
#define Bit13 13
#define Bit16 14
#define Bit17 15
#define Bit18 16
#define Bit19 17
#define Bit20 18
#define Bit21 19
#define Bit22 20
#define Bit23 21

//Pin# to Bit
#define Pin0 0
#define Pin1 1
#define Pin2 2
#define Pin3 3
#define Pin4 4
#define Pin5 5
#define Pin6 6
#define Pin7 7
#define Pin8 8
#define Pin9 9
#define Pin10 10
#define Pin11 11
#define Pin12 12
#define Pin13 13
#define Pin14 16
#define Pin15 17
#define Pin16 18
#define Pin17 19
#define Pin18 20
#define Pin19 21
#define Pin20 22
#define Pin21 23
#define PinA0 16
#define PinA1 17
#define PinA2 18
#define PinA3 19
#define PinA4 20
#define PinA5 21
#define PinA6 22
#define PinA7 23


class InterruptsClass {
public:
	//	 Interrupts();
	uint8_t  BitNumber[20] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 16, 17, 18, 19, 20, 21}; // Key = Pin Val = Position List of pins that could be used as ping inputs:
	static void nothing(void) {};
	static void nothing(uint32_t,uint32_t,uint32_t) {};

	typedef void (*voidFuncVoidPtr)(void);
	typedef void (*voidFunc3u32Ptr)(uint32_t,uint32_t,uint32_t);// Create a type to point to a function.
	voidFunc3u32Ptr  PinCB[20] = {nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing}; // Key = Pin Val = Position List of pins that could be used as ping inputs:
	InterruptsClass & AddPin(uint8_t pin, uint8_t InputType = INPUT);
	InterruptsClass & onInterrupt(void (*CB)(uint32_t,uint32_t,uint32_t));
	InterruptsClass & onPin(int Pin, int InputType, void (*CB)(uint32_t,uint32_t,uint32_t));
	void PinCallBack(uint32_t Time, uint32_t PinsChanged, uint32_t Pins);
	bool CheckPin(uint8_t Pin);
	InterruptsClass & Timer0Enable();
	InterruptsClass & onTimer0(void (*CB)(void));
	InterruptsClass & Timer1Enable();
	InterruptsClass & Timer2Enable();
	// *************************************************************
	volatile uint32_t EdgeTime[20]; // Time Storage
	// RC Remote
	volatile uint16_t RCRemote(uint8_t Pin, uint32_t cTime, uint32_t Pins,bool ValidRC = true);
	// Ping
	volatile int16_t Ping(uint8_t Pin, uint32_t cTime, uint32_t Pins);
	// Encoder
	volatile int8_t Encoder(uint8_t ClockPin, uint8_t DataPin, uint32_t Pins,bool HalfStep);
	// Switch
	volatile int8_t Switch (uint8_t Pin,uint16_t cTime,int16_t debounceTime,bool OnRise);
	// Sample Time
	volatile uint16_t SampleTime(uint8_t Pin, uint32_t cTime, uint32_t Pins,bool OnRise);
	volatile uint16_t DelayTimer(uint8_t Pin, uint32_t cTime, uint32_t Pins,bool OnRise);
	volatile int32_t Tachometer(uint8_t ClockPin, uint8_t DataPin, uint32_t cTime, uint32_t Pins);
};
#endif

#ifdef DEBUG
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define SERIALBEGIN(...)   Serial.begin(__VA_ARGS__)
#define DPRINT(...)        Serial.print(__VA_ARGS__)
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#define DPRINTF(...)       Serial.print(F(__VA_ARGS__))   //Printing text using the F macro
#define DPRINTLNF(...)     Serial.println(F(__VA_ARGS__)) //Printing text using the F macro
#define	DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print(F("\t"));Serial.print(F(Name));Serial.print(F(" ")); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}//StringSize,Name,Variable,Spaces,Percision
#define	DPRINTSF(...)      {char S[30]; Serial.print(dtostrf((float)__VA_ARGS__ , S))}//Variable,Spaces,Percision
#define	DPRINTLNSF(...)    {char S[30]; Serial.print(dtostrf((float)__VA_ARGS__ , S))}//Variable,Spaces,Percision
#define TOGGLEd13          PINB = 0x20                    //For the UNO only
#else
#define DPRINTSTIMER(t)    if(false)
#define SERIALBEGIN(...)   //blank line
#define DPRINT(...)        //blank line
#define DPRINTLN(...)      //blank line
#define DPRINTF(...)       //blank line
#define DPRINTLNF(...)     //blank line
#define	DPRINTSFN(...)     //blank line
#define	DPRINTSF(...)      //blank line
#define	DPRINTLNSF(...)    //blank line
#define TOGGLEd13          //blank line 
#endif
