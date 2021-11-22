#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Interrupts.h"
// All

static void nothing(void) {};
static void nothing(uint32_t,uint32_t,uint32_t) {};
typedef void (*voidFuncVoidPtr)(void);
typedef void (*voidFunc3u32Ptr)(uint32_t,uint32_t,uint32_t);// Create a type to point to a function.
typedef void (*voidFunc3u32Ptr)(uint32_t,uint32_t,uint32_t);// Create a type to point to a funciton.
volatile voidFunc3u32Ptr Int_CB = nothing;
volatile voidFuncVoidPtr TIMER0_CB = nothing;
volatile voidFuncVoidPtr TIMER2_CB = nothing;

volatile uint8_t  GroupStartPin[3]  = {0,8,14};
volatile uint8_t  GroupEndPin[3]    = {7,13,19};
union Mask{
	volatile uint32_t All;
	volatile uint8_t  Port[4];
};
Mask _Pins;

volatile uint32_t _PinMask; //volatile uint8_t  
volatile uint32_t _PCintLast; //volatile uint8_t  
volatile uint32_t Interupted;

/************ static functions common to all instances ***********************/
// port change Interrupt

void ISR_Exe(void);
ISR(PCINT2_vect) { //this ISR pins 0-7
	ISR_Exe();
}
ISR(PCINT1_vect) { //this ISR pinss A0~A5
	ISR_Exe();
}
ISR(PCINT0_vect) { //this ISR  pins 8-13 
	ISR_Exe();
}
void ISR_Exe(){
	_Pins.Port[0] = PIND; // 0-7
	_Pins.Port[1] = PINB; // 8-13
	_Pins.Port[2] = PINC; // A0-A6
	uint32_t _cTime = micros();    
	_Pins.Port[3] = 1;    // Placeholder
	uint32_t _Changed = _Pins.All ^ _PCintLast;// doing a ^ between the current interruption and the last one indicates wich pin changed
	uint32_t _Masked = _Changed & _PinMask;// Has one of the pins we need changed
	if(!_Masked)return;                    //No Data move on
	if (Int_CB) {
		Int_CB(_cTime, _Masked,_Pins.All); // Callback Function
	}
	Interupted++;
	_PCintLast = _Pins.All;          // we memorize the current state of all PINs in group
}
InterruptsClass & InterruptsClass::onTimer0(void (*CB)(void)){
	TIMER0_CB = CB;
	return *this; // return Pointer to this class
}
ISR(TIMER0_COMPA_vect){ 
	if (TIMER0_CB) {
		TIMER0_CB(); // Callback Function
	}
}
/*
ISR(TIMER0_OVF_vect)
{
	if (TIMER0_CB) {
		TIMER0_CB(); // Callback Function
	}
}
*/
ISR(TIMER2_COMPA_vect)    
{
	if (TIMER2_CB) {
		TIMER2_CB(); // Callback Function
	}
}


ISR(TIMER2_OVF_vect)  
{
	if (TIMER2_CB) {
		TIMER2_CB(); // Callback Function
	}
}

InterruptsClass & InterruptsClass::Timer0Enable(){
	OCR0A = 0xAF;
	TIMSK0 |= _BV(OCIE0A);
	return *this; // return Pointer to this class
}
InterruptsClass & InterruptsClass::Timer1Enable(){
	OCR1A = 0xAF;
	TIMSK1 |= _BV(OCIE1A);
	return *this; // return Pointer to this class
}
InterruptsClass & InterruptsClass::Timer2Enable(){
	OCR2A = 0xAF;
	TIMSK2 |= _BV(OCIE2A);
	return *this; // return Pointer to this class
}

/****************** end of static functions ******************************/
// Install Pin change interrupt for a pin, can be called multiple times

InterruptsClass & InterruptsClass::AddPin(uint8_t Pin, uint8_t InputType){
	pinMode(Pin, InputType);// enable interrupt for pin...
	if(bitRead(_PinMask, Pin)) return *this; // return Pointer to this class
	bitWrite(_PinMask, BitNumber[Pin], 1);
	*digitalPinToPCMSK(Pin) |= bit (digitalPinToPCMSKbit(Pin));  // enable pin
	PCIFR  |= bit (digitalPinToPCICRbit(Pin)); // clear any outstanding interrupt
	PCICR  |= bit (digitalPinToPCICRbit(Pin)); // enable interrupt for the group
	return *this; // return Pointer to this class
}

InterruptsClass & InterruptsClass::onInterrupt(void (*CB)(uint32_t,uint32_t,uint32_t)){
	Int_CB = CB;
	return *this; // return Pointer to this class
}

InterruptsClass & InterruptsClass::onPin(int Pin, int InputType, void (*CB)(uint32_t,uint32_t,uint32_t)){
	AddPin( Pin,  InputType);
	PinCB[Pin] = CB;
	return *this; // return Pointer to this class
}
void InterruptsClass::PinCallBack(uint32_t Time, uint32_t PinsChanged, uint32_t Pins){
	for(byte Pin = 0;Pin<20;Pin++){
		if(bitRead(PinsChanged, Pin)){
			if (PinCB[Pin]) {
				PinCB[Pin](Time,PinsChanged,Pins);
			}
		}
	}
}
bool InterruptsClass::CheckPin(uint8_t Pin){
//if(Pin >10)	    Serial.println(_Pins.All,BIN);
	return(bitRead(_Pins.All,BitNumber[Pin]));

	
}

volatile uint16_t InterruptsClass::RCRemote(uint8_t Pin, uint32_t cTime, uint32_t Pins,bool ValidRC){	

	if (CheckPin(Pin)) { 
		EdgeTime[Pin] = cTime; //Pulse went HIGH store the start time
		return (0);
	} else {                         // Pulse Went low calculate the duratoin
		uint16_t dTime = cTime - EdgeTime[Pin]; // Calculate the change in time
		if (ValidRC){
			if ((900 < dTime) && (dTime < 2100)){    // Time is within proper RC range 
				return(dTime);		 // only send proper PWM Pulse
			} else {
				return (0);
			}
		}
		return(dTime);		 // Lets send any duration up to 65535 micro seconds
	}
}

volatile int16_t InterruptsClass::Ping(uint8_t Pin, uint32_t cTime, uint32_t Pins){	
	return(RCRemote(Pin, cTime, Pins,false));
}

volatile int8_t InterruptsClass::Encoder(uint8_t ClockPin, uint8_t DataPin, uint32_t Pins,bool HalfStep){	
	uint8_t DataBit = CheckPin(DataPin);
	if (CheckPin(ClockPin)) {             //CLK Pulse went HIGH
		return(DataBit ? -1 : 1);  // Add this to your counter Variable
	} else if (HalfStep) { // half step code higher resolution
		return(DataBit ? 1 : -1); // Add this to your counter Variable
	}
	return(0);
}



volatile int8_t InterruptsClass::Switch (uint8_t Pin,uint16_t cTime,int16_t debounceTime,bool OnRise){
	if ((cTime - EdgeTime[Pin]) >= debounceTime ) {
		EdgeTime[Pin] = cTime;
		if (OnRise) { //on pin rise
			return(CheckPin(Pin) ? 1 : 0);
		} else{
			return(CheckPin(Pin) ? 0 : 1);
		}
	}
	return (0);
}

volatile uint16_t InterruptsClass::SampleTime(uint8_t Pin, uint32_t cTime, uint32_t Pins,bool OnRise){	
	if (CheckPin(Pin) == OnRise) { 
			uint16_t dTime = cTime - EdgeTime[Pin]; // Calculate the change in time
			EdgeTime[Pin] = cTime; //Pulse went HIGH store the start time
			return (dTime);
	} 
	return (0);
}

volatile uint16_t InterruptsClass::DelayTimer(uint8_t Pin, uint32_t cTime, uint32_t Pins,bool OnRise){	
	
	return (0);
}
volatile int32_t InterruptsClass::Tachometer(uint8_t ClockPin, uint8_t DataPin, uint32_t cTime, uint32_t Pins){ // returns MS per Pulse + value is Forward - Value is backwards
	if (CheckPin(ClockPin)) {             //CLK Pulse went HIGH
		int32_t dTime;
		dTime = cTime - EdgeTime[ClockPin]; 
		EdgeTime[ClockPin] = cTime;
		return(CheckPin(DataPin) ? dTime: (-1 * dTime)); // Sets the time between pulses positive or negative 
	}
	return(false);
}
