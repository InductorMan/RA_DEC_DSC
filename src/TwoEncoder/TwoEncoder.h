/* Based on:
 * Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 *
 * Version 1.2 - fix -2 bug in C-only code
 * Version 1.1 - expand to support boards with up to 60 interrupts
 * Version 1.0 - initial release
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
This version of the Encoder library is a special-purpose version for use on 
ESP8266 specifically, and for use with a pair of encoders producing index
pulses, with interrupts enabled on all six encoder pins. Experiments show 
that running two instances of the original Encoder library (whether with index
or not) on the ESP8266 results in missed edges. I haven't figured out what the
issue is, although it seems like a race condition between the two interrupts.
On inspection though the code appears to properly capture interrupts whether
they occured before or after the ISR entry. Regardless, preliminary 
experiments show that running the logic for both encoders in a single ISR 
seems to fix the missed-edge issue.

This is not a pretty library.
*/



#ifndef TwoEncoder_h_
#define TwoEncoder_h_


#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#elif defined(WIRING)
#include "Wiring.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include "utility/direct_pin_read.h"

#if defined(ENCODER_USE_INTERRUPTS) || !defined(ENCODER_DO_NOT_USE_INTERRUPTS)
#define ENCODER_USE_INTERRUPTS
#define ENCODER_ARGLIST_SIZE CORE_NUM_INTERRUPT
#include "utility/interrupt_pins.h"
#else
#define ENCODER_ARGLIST_SIZE 0
#endif

// Use ICACHE_RAM_ATTR for ISRs to prevent ESP8266 resets; all 
//ISRs must be in IRAM for reliable performance becuase flash memory reads
//do not have guaranteed execution time
#if defined(ESP8266)
#define ISR_ATTR ICACHE_RAM_ATTR
#else
#define ISR_ATTR
#endif

#define HAS_INDEX 1
#define NO_INDEX 0


typedef struct {
	volatile IO_REG_TYPE * pin1_register;
	volatile IO_REG_TYPE * pin2_register;
	volatile IO_REG_TYPE * pinI_register;
	IO_REG_TYPE            pin1_bitmask;
	IO_REG_TYPE            pin2_bitmask;
	IO_REG_TYPE            pinI_bitmask;
	uint8_t                state;
	int32_t                position;
	int32_t                errors;




	volatile IO_REG_TYPE * pin1B_register;
	volatile IO_REG_TYPE * pin2B_register;
	volatile IO_REG_TYPE * pinIB_register;
	IO_REG_TYPE            pin1B_bitmask;
	IO_REG_TYPE            pin2B_bitmask;
	IO_REG_TYPE            pinIB_bitmask;
	uint8_t                stateB;
	int32_t                positionB;
	int32_t                errorsB;

} Encoder_internal_state_t;

typedef void(*isr_t)(Encoder_internal_state_t *);

class TwoEncoder
{
public:

	TwoEncoder(uint8_t pin1, uint8_t pin2, uint8_t pinI, uint8_t pin1B, uint8_t pin2B, uint8_t pinIB) {

		uint8_t pin1_has_interrupt = 0;
		uint8_t pin1B_has_interrupt = 0;
		uint8_t pin2_has_interrupt = 0;
		uint8_t pin2B_has_interrupt = 0;

		missingIntPins = 0;
	 	missingIntPinsB = 0;


		#ifdef INPUT_PULLUP
		pinMode(pin1, INPUT_PULLUP);
		pinMode(pin2, INPUT_PULLUP);
		pinMode(pinI, INPUT_PULLUP);

		pinMode(pin1B, INPUT_PULLUP);
		pinMode(pin2B, INPUT_PULLUP);
		pinMode(pinIB, INPUT_PULLUP);
		#else
		pinMode(pin1, INPUT);
		digitalWrite(pin1, HIGH);
		pinMode(pin2, INPUT);
		digitalWrite(pin2, HIGH);
		pinMode(pinI, INPUT);
		digitalWrite(pinI, HIGH);

		pinMode(pin1B, INPUT);
		digitalWrite(pin1B, HIGH);
		pinMode(pin2B, INPUT);
		digitalWrite(pin2B, HIGH);
		pinMode(pinIB, INPUT);
		digitalWrite(pinIB, HIGH);
		#endif

		encoder.pin1_register = PIN_TO_BASEREG(pin1);
		encoder.pin1_bitmask = PIN_TO_BITMASK(pin1);
		encoder.pin2_register = PIN_TO_BASEREG(pin2);
		encoder.pin2_bitmask = PIN_TO_BITMASK(pin2);
		encoder.pinI_register = PIN_TO_BASEREG(pinI);
		encoder.pinI_bitmask = PIN_TO_BITMASK(pinI);
		encoder.position = 0;


		encoder.pin1B_register = PIN_TO_BASEREG(pin1B);
		encoder.pin1B_bitmask = PIN_TO_BITMASK(pin1B);
		encoder.pin2B_register = PIN_TO_BASEREG(pin2B);
		encoder.pin2B_bitmask = PIN_TO_BITMASK(pin2B);
		encoder.pinIB_register = PIN_TO_BASEREG(pinIB);
		encoder.pinIB_bitmask = PIN_TO_BITMASK(pinIB);
		encoder.positionB = 0;
		// allow time for a passive R-C filter to charge
		// through the pullup resistors, before reading
		// the initial state
		delayMicroseconds(2000);


		uint8_t s = 0;

		if (DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask)) s |= 1;
		if (DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask)) s |= 2;
		encoder.state = s;
		s = 0;
		if (DIRECT_PIN_READ(encoder.pin1B_register, encoder.pin1B_bitmask)) s |= 1;
		if (DIRECT_PIN_READ(encoder.pin2B_register, encoder.pin2B_bitmask)) s |= 2;
		encoder.stateB = s;



		pin1_has_interrupt = check_interrupt(pin1);
		pin1B_has_interrupt = check_interrupt(pin1B);

		pin2_has_interrupt = check_interrupt(pin2);
		pin2B_has_interrupt = check_interrupt(pin2B);

		



		if(pin1_has_interrupt && pin2_has_interrupt){
			isrInUseA = updateI;
		}
		else if(pin1_has_interrupt && !pin2_has_interrupt){
			isrInUseA = updateIPin1;
		}
		else if(!pin1_has_interrupt && pin2_has_interrupt){
			isrInUseA = updateIPin2;
		} 
		else {
			isrInUseA = updateI;
			missingIntPins = 1;
		}


		if(pin1B_has_interrupt && pin2B_has_interrupt){
			isrInUseB = updateI;
		}
		else if(pin1B_has_interrupt && !pin2B_has_interrupt){
			isrInUseB = updateIPin1;
		}
		else if(!pin1B_has_interrupt && pin2B_has_interrupt){
			isrInUseB = updateIPin2;
		} 
		else {
			isrInUseB = updateI;
			missingIntPinsB = 1;
		}

		attach_interrupt(pin1, &encoder, CHANGE, isrInUseA);
		attach_interrupt(pin2, &encoder, CHANGE, isrInUseA);
		attach_interrupt(pinI, &encoder, RISING, isrInUseA);

		attach_interrupt(pin1B, &encoder, CHANGE, isrInUseB);
		attach_interrupt(pin2B, &encoder, CHANGE, isrInUseB);
		attach_interrupt(pinIB, &encoder, RISING, isrInUseB);

		
	}

	TwoEncoder(uint8_t pin1, uint8_t pin2, uint8_t pin1B, uint8_t pin2B) {

		uint8_t pin1_has_interrupt = 0;
		uint8_t pin1B_has_interrupt = 0;
		uint8_t pin2_has_interrupt = 0;
		uint8_t pin2B_has_interrupt = 0;


		missingIntPins = 0;
	 	missingIntPinsB = 0;


		#ifdef INPUT_PULLUP
		pinMode(pin1, INPUT_PULLUP);
		pinMode(pin2, INPUT_PULLUP);

		pinMode(pin1B, INPUT_PULLUP);
		pinMode(pin2B, INPUT_PULLUP);
		#else
		pinMode(pin1, INPUT);
		digitalWrite(pin1, HIGH);
		pinMode(pin2, INPUT);
		digitalWrite(pin2, HIGH);

		pinMode(pin1B, INPUT);
		digitalWrite(pin1B, HIGH);
		pinMode(pin2B, INPUT);
		digitalWrite(pin2B, HIGH);
		#endif

		encoder.pin1_register = PIN_TO_BASEREG(pin1);
		encoder.pin1_bitmask = PIN_TO_BITMASK(pin1);
		encoder.pin2_register = PIN_TO_BASEREG(pin2);
		encoder.pin2_bitmask = PIN_TO_BITMASK(pin2);
		encoder.position = 0;


		encoder.pin1B_register = PIN_TO_BASEREG(pin1B);
		encoder.pin1B_bitmask = PIN_TO_BITMASK(pin1B);
		encoder.pin2B_register = PIN_TO_BASEREG(pin2B);
		encoder.pin2B_bitmask = PIN_TO_BITMASK(pin2B);
		encoder.positionB = 0;
		// allow time for a passive R-C filter to charge
		// through the pullup resistors, before reading
		// the initial state
		delayMicroseconds(2000);

		uint8_t s = 0;
		if (DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask)) s |= 1;
		if (DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask)) s |= 2;
		encoder.state = s;
		s = 0;
		if (DIRECT_PIN_READ(encoder.pin1B_register, encoder.pin1B_bitmask)) s |= 1;
		if (DIRECT_PIN_READ(encoder.pin2B_register, encoder.pin2B_bitmask)) s |= 2;
		encoder.stateB = s;

		pin1_has_interrupt = check_interrupt(pin1);
		pin1B_has_interrupt = check_interrupt(pin1B);
		pin2_has_interrupt = check_interrupt(pin2);
		pin2B_has_interrupt = check_interrupt(pin2B);




		if(pin1_has_interrupt && pin2_has_interrupt){
			isrInUseA = update;
		}
		else if(pin1_has_interrupt && !pin2_has_interrupt){
			isrInUseA = updatePin1;
		}
		else if(!pin1_has_interrupt && pin2_has_interrupt){
			isrInUseA = updatePin2;
		} 
		else {
			isrInUseA = update;
			missingIntPins = 1;
		}


		if(pin1B_has_interrupt && pin2B_has_interrupt){
			isrInUseB = update;
		}
		else if(pin1B_has_interrupt && !pin2B_has_interrupt){
			isrInUseB = updateIPin1;
		}
		else if(!pin1B_has_interrupt && pin2B_has_interrupt){
			isrInUseB = updateIPin2;
		} 
		else {
			isrInUseB = update;
			missingIntPinsB = 1;
		}

		attach_interrupt(pin1, &encoder, CHANGE, isrInUseA);
		attach_interrupt(pin2, &encoder, CHANGE, isrInUseA);

		attach_interrupt(pin1B, &encoder, CHANGE, isrInUseB);
		attach_interrupt(pin2B, &encoder, CHANGE, isrInUseB);

	}
	TwoEncoder(uint8_t pin1, uint8_t pin2, uint8_t pinI) {

		uint8_t pin1_has_interrupt = 0;
		uint8_t pin2_has_interrupt = 0;

		missingIntPins = 0;


		#ifdef INPUT_PULLUP
		pinMode(pin1, INPUT_PULLUP);
		pinMode(pin2, INPUT_PULLUP);
		pinMode(pinI, INPUT_PULLUP);

		#else
		pinMode(pin1, INPUT);
		digitalWrite(pin1, HIGH);
		pinMode(pin2, INPUT);
		digitalWrite(pin2, HIGH);
		pinMode(pinI, INPUT);
		digitalWrite(pinI, HIGH);
		#endif

		encoder.pin1_register = PIN_TO_BASEREG(pin1);
		encoder.pin1_bitmask = PIN_TO_BITMASK(pin1);
		encoder.pin2_register = PIN_TO_BASEREG(pin2);
		encoder.pin2_bitmask = PIN_TO_BITMASK(pin2);
		encoder.pinI_register = PIN_TO_BASEREG(pinI);
		encoder.pinI_bitmask = PIN_TO_BITMASK(pinI);
		encoder.position = 0;

		// allow time for a passive R-C filter to charge
		// through the pullup resistors, before reading
		// the initial state
		delayMicroseconds(2000);


		uint8_t s = 0;

		if (DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask)) s |= 1;
		if (DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask)) s |= 2;
		encoder.state = s;




		pin1_has_interrupt = check_interrupt(pin1);
		pin2_has_interrupt = check_interrupt(pin2);

		



		if(pin1_has_interrupt && pin2_has_interrupt){
			isrInUseA =  singleUpdateI;
		}
		else if(pin1_has_interrupt && !pin2_has_interrupt){
			isrInUseA = singleUpdateIPin1;
		}
		else if(!pin1_has_interrupt && pin2_has_interrupt){
			isrInUseA = singleUpdateIPin2;
		} 
		else {
			isrInUseA = singleUpdateI;
			missingIntPins = 1;
		}

		isrInUseB = isrInUseA;


		attach_interrupt(pin1, &encoder, CHANGE, isrInUseA);
		attach_interrupt(pin2, &encoder, CHANGE, isrInUseA);
		attach_interrupt(pinI, &encoder, RISING, isrInUseA);


		
	}

	TwoEncoder(uint8_t pin1, uint8_t pin2) {

		uint8_t pin1_has_interrupt = 0;
		uint8_t pin2_has_interrupt = 0;


		missingIntPins = 0;


		#ifdef INPUT_PULLUP
		pinMode(pin1, INPUT_PULLUP);
		pinMode(pin2, INPUT_PULLUP);

		#else
		pinMode(pin1, INPUT);
		digitalWrite(pin1, HIGH);
		pinMode(pin2, INPUT);
		digitalWrite(pin2, HIGH);

		#endif

		encoder.pin1_register = PIN_TO_BASEREG(pin1);
		encoder.pin1_bitmask = PIN_TO_BITMASK(pin1);
		encoder.pin2_register = PIN_TO_BASEREG(pin2);
		encoder.pin2_bitmask = PIN_TO_BITMASK(pin2);
		encoder.position = 0;

		// allow time for a passive R-C filter to charge
		// through the pullup resistors, before reading
		// the initial state
		delayMicroseconds(2000);

		uint8_t s = 0;
		if (DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask)) s |= 1;
		if (DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask)) s |= 2;
		encoder.state = s;


		pin1_has_interrupt = check_interrupt(pin1);
		pin2_has_interrupt = check_interrupt(pin2);




		if(pin1_has_interrupt && pin2_has_interrupt){
			isrInUseA = singleUpdate;
		}
		else if(pin1_has_interrupt && !pin2_has_interrupt){
			isrInUseA = singleUpdatePin1;
		}
		else if(!pin1_has_interrupt && pin2_has_interrupt){
			isrInUseA = singleUpdatePin2;
		} 
		else {
			isrInUseA = singleUpdate;
			missingIntPins = 1;
		}


		isrInUseB = isrInUseA;

		attach_interrupt(pin1, &encoder, CHANGE, isrInUseA);
		attach_interrupt(pin2, &encoder, CHANGE, isrInUseA);


	}

	inline int32_t read() { return readA();}
	inline int32_t readA() {
		if (missingIntPins) {
			noInterrupts();
			isrInUseA(&encoder);
			interrupts();
		}
		int32_t ret = encoder.position;
		return ret;
	}
	inline int32_t readB() {
		if (missingIntPinsB) {
			noInterrupts();
			isrInUseB(&encoder);
			interrupts();
		}
		int32_t ret = encoder.positionB;
		return ret;
	}
	inline int32_t readAndReset() { return readAndReset(); }
	inline int32_t readAndResetA() {
		if (missingIntPins) {
			noInterrupts();
			isrInUseA(&encoder);
			interrupts();
		}
		int32_t ret = encoder.position;
		encoder.position = 0;

		return ret;
	}
	inline int32_t readAndResetB() {
		if (missingIntPinsB) {
			noInterrupts();
			isrInUseA(&encoder);
			interrupts();
		}
		int32_t ret = encoder.positionB;
		encoder.positionB = 0;

		return ret;
	}
	inline void write(int32_t p) {
		noInterrupts();
		encoder.position = p;
		interrupts();
	}  
	inline int32_t readErrors() { return readErrorsA(); }
	inline int32_t readErrorsA() {
		return encoder.errors;
	}
	inline int32_t readErrorsB() {
		return encoder.errorsB;
	}

private:
	Encoder_internal_state_t encoder;

	uint8_t missingIntPins;
	uint8_t missingIntPinsB;

	isr_t isrInUseA;
	isr_t isrInUseB;



public:
	static Encoder_internal_state_t * interruptArgs[ENCODER_ARGLIST_SIZE];

	static isr_t isrList[ENCODER_ARGLIST_SIZE];


//                           _______         _______       
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2

//both pins with interrupts attached: 
		//	new	new	old	old
		//	pin2	pin1	pin2	pin1	Result
		//	----	----	----	----	------
		//	0	0	0	0	no movement
		//	0	0	0	1	+1
		//	0	0	1	0	-1
		//	0	0	1	1	error
		//	0	1	0	0	-1
		//	0	1	0	1	no movement
		//	0	1	1	0	error
		//	0	1	1	1	+1
		//	1	0	0	0	+1
		//	1	0	0	1	error
		//	1	0	1	0	no movement
		//	1	0	1	1	-1
		//	1	1	0	0	error
		//	1	1	0	1	-1
		//	1	1	1	0	+1
		//	1	1	1	1	no movement

//first pin with interrupt attached:
		//	new	new	old	old
		//	pin2	pin1	pin2	pin1	Result
		//	----	----	----	----	------
		//	0	0	0	0	no movement
		//	0	0	0	1	+1
		//	0	0	1	0	-1
		//	0	0	1	1	+2  (if only pin1 has interrupt attached) 
		//	0	1	0	0	-1
		//	0	1	0	1	no movement
		//	0	1	1	0	-2  (if only pin1 has interrupt attached)
		//	0	1	1	1	+1
		//	1	0	0	0	+1
		//	1	0	0	1	-2  (if only pin1 has interrupt attached)
		//	1	0	1	0	no movement
		//	1	0	1	1	-1
		//	1	1	0	0	+2  (if only pin1 has interrupt attached)
		//	1	1	0	1	-1
		//	1	1	1	0	+1
		//	1	1	1	1	no movement

//second pin with interrupt attached:
		//	new	new	old	old
		//	pin2	pin1	pin2	pin1	Result
		//	----	----	----	----	------
		//	0	0	0	0	no movement
		//	0	0	0	1	+1
		//	0	0	1	0	-1
		//	0	0	1	1	-2  (if only pin2 has interrupt attached) 
		//	0	1	0	0	-1
		//	0	1	0	1	no movement
		//	0	1	1	0	+2  (if only pin2 has interrupt attached)
		//	0	1	1	1	+1
		//	1	0	0	0	+1
		//	1	0	0	1	+2  (if only pin2 has interrupt attached)
		//	1	0	1	0	no movement
		//	1	0	1	1	-1
		//	1	1	0	0	-2  (if only pin2 has interrupt attached)
		//	1	1	0	1	-1
		//	1	1	1	0	+1
		//	1	1	1	1	no movement

public:
	// update() is not meant to be called from outside Encoder,
	// but it is public to allow static interrupt routines.
	// DO NOT call update() directly from sketches.

	//in order to preserve as much speed as possible, a multiplicity of
	//update functions are declared to deal with all the possible types
	//of encoder and pin configurations, so that conditional statements
	//don't need to be executed during the ISR. The types are named 
	// using a combination of labels from the following groups:
	//
	// update: two encoders
	// singleUpdate: only one encoder
	//
	// I: both encoders have an index pin
	// [no I]: neither encoder has an index pin
	//
	// [no Pin]
	// Pin1: only pin 1 has an interrupt attached
	// Pin2: only pin 2 has an interrupt attached
	// 
	// note that two encoders but only one equipped with index pin is not
	// a supported configuration.
	//
	// error generation is not supported if only one pin is interrupt equipped
	// since all possible state transitions are likely (non-interrupt-pin transitons
	// are possible when read() is called, since this invokes the update() routine
	// asynchronously from the pin transitions)


	static ISR_ATTR void update(Encoder_internal_state_t *arg) {


		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);

		uint8_t p1valB = DIRECT_PIN_READ(arg->pin1B_register, arg->pin1B_bitmask);
		uint8_t p2valB = DIRECT_PIN_READ(arg->pin2B_register, arg->pin2B_bitmask);


		uint8_t state = arg->state & 3;
		uint8_t stateB = arg->stateB & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				break;
			case 2: case 4: case 11: case 13:
				arg->position--;
				break;
			case 3: case 12: case 6: case 9:
				arg->errors++;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	



		

		if (p1valB) stateB |= 4;
		if (p2valB) stateB |= 8;
		arg->stateB = (stateB >> 2);

		switch (stateB) {
			case 1: case 7: case 8: case 14:
				arg->positionB++;
				break;
			case 2: case 4: case 11: case 13:
				arg->positionB--;
				break;
			case 3: case 12: case 6: case 9:
				arg->errorsB++;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	



		return;
	}


	static ISR_ATTR void updateI(Encoder_internal_state_t *arg) {

		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);
		uint8_t pIval = DIRECT_PIN_READ(arg->pinI_register, arg->pinI_bitmask);

		uint8_t p1valB = DIRECT_PIN_READ(arg->pin1B_register, arg->pin1B_bitmask);
		uint8_t p2valB = DIRECT_PIN_READ(arg->pin2B_register, arg->pin2B_bitmask);
		uint8_t pIvalB = DIRECT_PIN_READ(arg->pinIB_register, arg->pinIB_bitmask);


		uint8_t state = arg->state & 3;
		uint8_t stateB = arg->stateB & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				break;
			case 2: case 4: case 11: case 13:
				arg->position--;
				break;
			case 3: case 12: case 6: case 9:
				arg->errors++;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	


		if(pIval) {
			arg->position = 0;
		}

		

		if (p1valB) stateB |= 4;
		if (p2valB) stateB |= 8;
		arg->stateB = (stateB >> 2);

		switch (stateB) {
			case 1: case 7: case 8: case 14:
				arg->positionB++;
				break;
			case 2: case 4: case 11: case 13:
				arg->positionB--;
				break;
			case 3: case 12: case 6: case 9:
				arg->errorsB++;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	

		if(pIvalB) {
			arg->positionB = 0;
		}


		return;
	}

	static ISR_ATTR void updatePin1(Encoder_internal_state_t *arg) {

		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);

		uint8_t p1valB = DIRECT_PIN_READ(arg->pin1B_register, arg->pin1B_bitmask);
		uint8_t p2valB = DIRECT_PIN_READ(arg->pin2B_register, arg->pin2B_bitmask);


		uint8_t state = arg->state & 3;
		uint8_t stateB = arg->stateB & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				break;
			case 2: case 4: case 11: case 13:
				arg->position--;
				break;
			case 3: case 12:
				arg->position += 2;
				break;
			case 6: case 9:
				arg->position -= 2;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	



		

		if (p1valB) stateB |= 4;
		if (p2valB) stateB |= 8;
		arg->stateB = (stateB >> 2);

		switch (stateB) {
			case 1: case 7: case 8: case 14:
				arg->positionB++;
				break;
			case 2: case 4: case 11: case 13:
				arg->positionB--;
				break;
			case 3: case 12: 
				arg->positionB += 2;
				break;
			case 6: case 9:
				arg->positionB -= 2;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	



		return;
	}


	static ISR_ATTR void updateIPin1(Encoder_internal_state_t *arg) {

		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);
		uint8_t pIval = DIRECT_PIN_READ(arg->pinI_register, arg->pinI_bitmask);

		uint8_t p1valB = DIRECT_PIN_READ(arg->pin1B_register, arg->pin1B_bitmask);
		uint8_t p2valB = DIRECT_PIN_READ(arg->pin2B_register, arg->pin2B_bitmask);
		uint8_t pIvalB = DIRECT_PIN_READ(arg->pinIB_register, arg->pinIB_bitmask);

		uint8_t state = arg->state & 3;
		uint8_t stateB = arg->stateB & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				break;
			case 2: case 4: case 11: case 13:
				arg->position--;
				break;
			case 3: case 12:
				arg->position += 2;
				break;
			case 6: case 9:
				arg->position -= 2;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	

		if(pIval) {
			arg->position = 0;
		}

		

		if (p1valB) stateB |= 4;
		if (p2valB) stateB |= 8;
		arg->stateB = (stateB >> 2);

		switch (stateB) {
			case 1: case 7: case 8: case 14:
				arg->positionB++;
				break;
			case 2: case 4: case 11: case 13:
				arg->positionB--;
				break;
			case 3: case 12: 
				arg->positionB += 2;
				break;
			case 6: case 9:
				arg->positionB -= 2;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	


		if(pIvalB) {
			arg->positionB = 0;
		}

		return;
	}

		static ISR_ATTR void updatePin2(Encoder_internal_state_t *arg) {

		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);

		uint8_t p1valB = DIRECT_PIN_READ(arg->pin1B_register, arg->pin1B_bitmask);
		uint8_t p2valB = DIRECT_PIN_READ(arg->pin2B_register, arg->pin2B_bitmask);


		uint8_t state = arg->state & 3;
		uint8_t stateB = arg->stateB & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				break;
			case 2: case 4: case 11: case 13:
				arg->position--;
				break;
			case 3: case 12:
				arg->position -= 2;
				break;
			case 6: case 9:
				arg->position += 2;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	



		

		if (p1valB) stateB |= 4;
		if (p2valB) stateB |= 8;
		arg->stateB = (stateB >> 2);

		switch (stateB) {
			case 1: case 7: case 8: case 14:
				arg->positionB++;
				break;
			case 2: case 4: case 11: case 13:
				arg->positionB--;
				break;
			case 3: case 12: 
				arg->positionB -= 2;
				break;
			case 6: case 9:
				arg->positionB += 2;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	



		return;
	}


	static ISR_ATTR void updateIPin2(Encoder_internal_state_t *arg) {

		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);
		uint8_t pIval = DIRECT_PIN_READ(arg->pinI_register, arg->pinI_bitmask);

		uint8_t p1valB = DIRECT_PIN_READ(arg->pin1B_register, arg->pin1B_bitmask);
		uint8_t p2valB = DIRECT_PIN_READ(arg->pin2B_register, arg->pin2B_bitmask);
		uint8_t pIvalB = DIRECT_PIN_READ(arg->pinIB_register, arg->pinIB_bitmask);

		uint8_t state = arg->state & 3;
		uint8_t stateB = arg->stateB & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				break;
			case 2: case 4: case 11: case 13:
				arg->position--;
				break;
			case 3: case 12:
				arg->position -= 2;
				break;
			case 6: case 9:
				arg->position += 2;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	

		if(pIval) {
			arg->position = 0;
		}

		

		if (p1valB) stateB |= 4;
		if (p2valB) stateB |= 8;
		arg->stateB = (stateB >> 2);

		switch (stateB) {
			case 1: case 7: case 8: case 14:
				arg->positionB++;
				break;
			case 2: case 4: case 11: case 13:
				arg->positionB--;
				break;
			case 3: case 12: 
				arg->positionB -= 2;
				break;
			case 6: case 9:
				arg->positionB += 2;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	


		if(pIvalB) {
			arg->positionB = 0;
		}

		return;
	}



	static ISR_ATTR void singleUpdate(Encoder_internal_state_t *arg) {


		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);


		uint8_t state = arg->state & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				return;
			case 2: case 4: case 11: case 13:
				arg->position--;
				return;
			case 3: case 12: case 6: case 9:
				arg->errors++;
				return;
			case 0: case 5: case 10: case 15:
				return;

		}	



	
	}


	static ISR_ATTR void singleUpdateI(Encoder_internal_state_t *arg) {

		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);
		uint8_t pIval = DIRECT_PIN_READ(arg->pinI_register, arg->pinI_bitmask);


		uint8_t state = arg->state & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				break;
			case 2: case 4: case 11: case 13:
				arg->position--;
				break;
			case 3: case 12: case 6: case 9:
				arg->errors++;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	


		if(pIval) {
			arg->position = 0;
		}

		return;
	}

	static ISR_ATTR void singleUpdatePin1(Encoder_internal_state_t *arg) {

		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);


		uint8_t state = arg->state & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				return;
			case 2: case 4: case 11: case 13:
				arg->position--;
				return;
			case 3: case 12:
				arg->position += 2;
				return;
			case 6: case 9:
				arg->position -= 2;
				return;
			case 0: case 5: case 10: case 15:
				return;

		}	


	}


	static ISR_ATTR void singleUpdateIPin1(Encoder_internal_state_t *arg) {

		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);
		uint8_t pIval = DIRECT_PIN_READ(arg->pinI_register, arg->pinI_bitmask);

		uint8_t state = arg->state & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				break;
			case 2: case 4: case 11: case 13:
				arg->position--;
				break;
			case 3: case 12:
				arg->position += 2;
				break;
			case 6: case 9:
				arg->position -= 2;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	

		if(pIval) {
			arg->position = 0;
		}


		return;
	}

		static ISR_ATTR void singleUpdatePin2(Encoder_internal_state_t *arg) {

		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);


		uint8_t state = arg->state & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				return;
			case 2: case 4: case 11: case 13:
				arg->position--;
				return;
			case 3: case 12:
				arg->position -= 2;
				return;
			case 6: case 9:
				arg->position += 2;
				return;
			case 0: case 5: case 10: case 15:
				return;

		}	


	}


	static ISR_ATTR void singleUpdateIPin2(Encoder_internal_state_t *arg) {

		uint8_t p1val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask);
		uint8_t p2val = DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask);
		uint8_t pIval = DIRECT_PIN_READ(arg->pinI_register, arg->pinI_bitmask);

		uint8_t state = arg->state & 3;

		if (p1val) state |= 4;
		if (p2val) state |= 8;
		arg->state = (state >> 2);

		switch (state) {
			case 1: case 7: case 8: case 14:
				arg->position++;
				break;
			case 2: case 4: case 11: case 13:
				arg->position--;
				break;
			case 3: case 12:
				arg->position -= 2;
				break;
			case 6: case 9:
				arg->position += 2;
				break;
			case 0: case 5: case 10: case 15:
				break;

		}	

		if(pIval) {
			arg->position = 0;
		}

		
		return;
	}

private:

	//one channel
	//one channel, <2 interrupts
	//one channel, index
	//one channel, index, <2 interrupts
	//two channel
	//two channel, <2 interrupts
	//two channel, index
	//two channel, index, <2 interrupts



	// this giant function is an unfortunate consequence of Arduino's
	// attachInterrupt function not supporting any way to pass a pointer
	// or other context to the attached function.
	//
	// It also unforunately duplicates the exact machinery used by at least some
	// platforms to handle interrupts. For instance the ESP8266 core uses a static
	// global array of function pointers paired with argument pointers, and could
	// actually do this exact thing if the argument pointers were exposed to the
	// user. We end up having to pass it a pointer to a wrapper function that
	// then calls a member of our function pointer list supplied with the paired
	// argument from the args list. When the "core_esp8266_wiring_digital.c" 
	// interrupt_handler executes, it first
	// looks up the wrapper function pointer in its own interruptHanlders[] array,
	// checks to see if the interruptHandlers[i] entity has an argument pointer (which it
	// doesn't because attachInterruptArgs() isn't available externally), calls it,
	// and then we have to call our actual ISR function from within the wrapper.
	// This incurs an additional nested function call
	// per interrupt beyond what could theoretically be achieved. 



	static uint8_t attach_interrupt(uint8_t pin, Encoder_internal_state_t *state, uint8_t mode, void(*func)(Encoder_internal_state_t *)) {
			switch (pin) {
			#ifdef CORE_INT0_PIN
				case CORE_INT0_PIN:
					isrList[0] = func;
					interruptArgs[0] = state;
					attachInterrupt(0, isr0, mode);
					break;
			#endif
			#ifdef CORE_INT1_PIN
				case CORE_INT1_PIN:
					isrList[1] = func;
					interruptArgs[1] = state;
					attachInterrupt(1, isr1, mode);
					break;
			#endif
			#ifdef CORE_INT2_PIN
				case CORE_INT2_PIN:
					isrList[2] = func;
					interruptArgs[2] = state;
					attachInterrupt(2, isr2, mode);
					break;
			#endif
			#ifdef CORE_INT3_PIN
				case CORE_INT3_PIN:
					isrList[3] = func;
					interruptArgs[3] = state;
					attachInterrupt(3, isr3, mode);
					break;
			#endif
			#ifdef CORE_INT4_PIN
				case CORE_INT4_PIN:
					isrList[4] = func;
					interruptArgs[4] = state;
					attachInterrupt(4, isr4, mode);
					break;
			#endif
			#ifdef CORE_INT5_PIN
				case CORE_INT5_PIN:
					isrList[5] = func;
					interruptArgs[5] = state;
					attachInterrupt(5, isr5, mode);
					break;
			#endif
			#ifdef CORE_INT6_PIN
				case CORE_INT6_PIN:
					isrList[6] = func;
					interruptArgs[6] = state;
					attachInterrupt(6, isr6, mode);
					break;
			#endif
			#ifdef CORE_INT7_PIN
				case CORE_INT7_PIN:
					isrList[7] = func;
					interruptArgs[7] = state;
					attachInterrupt(7, isr7, mode);
					break;
			#endif
			#ifdef CORE_INT8_PIN
				case CORE_INT8_PIN:
					isrList[8] = func;
					interruptArgs[8] = state;
					attachInterrupt(8, isr8, mode);
					break;
			#endif
			#ifdef CORE_INT9_PIN
				case CORE_INT9_PIN:
					isrList[9] = func;
					interruptArgs[9] = state;
					attachInterrupt(9, isr9, mode);
					break;
			#endif
			#ifdef CORE_INT10_PIN
				case CORE_INT10_PIN:
					isrList[10] = func;
					interruptArgs[10] = state;
					attachInterrupt(10, isr10, mode);
					break;
			#endif
			#ifdef CORE_INT11_PIN
				case CORE_INT11_PIN:
					isrList[11] = func;
					interruptArgs[11] = state;
					attachInterrupt(11, isr11, mode);
					break;
			#endif
			#ifdef CORE_INT12_PIN
				case CORE_INT12_PIN:
					isrList[12] = func;
					interruptArgs[12] = state;
					attachInterrupt(12, isr12, mode);
					break;
			#endif
			#ifdef CORE_INT13_PIN
				case CORE_INT13_PIN:
					isrList[13] = func;
					interruptArgs[13] = state;
					attachInterrupt(13, isr13, mode);
					break;
			#endif
			#ifdef CORE_INT14_PIN
				case CORE_INT14_PIN:
					isrList[14] = func;
					interruptArgs[14] = state;
					attachInterrupt(14, isr14, mode);
					break;
			#endif
			#ifdef CORE_INT15_PIN
				case CORE_INT15_PIN:
					isrList[15] = func;
					interruptArgs[15] = state;
					attachInterrupt(15, isr15, mode);
					break;
			#endif
			#ifdef CORE_INT16_PIN
				case CORE_INT16_PIN:
					isrList[16] = func;
					interruptArgs[16] = state;
					attachInterrupt(16, isr16, mode);
					break;
			#endif
			#ifdef CORE_INT17_PIN
				case CORE_INT17_PIN:
					isrList[17] = func;
					interruptArgs[17] = state;
					attachInterrupt(17, isr17, mode);
					break;
			#endif
			#ifdef CORE_INT18_PIN
				case CORE_INT18_PIN:
					isrList[18] = func;
					interruptArgs[18] = state;
					attachInterrupt(18, isr18, mode);
					break;
			#endif
			#ifdef CORE_INT19_PIN
				case CORE_INT19_PIN:
					isrList[19] = func;
					interruptArgs[19] = state;
					attachInterrupt(19, isr19, mode);
					break;
			#endif
			#ifdef CORE_INT20_PIN
				case CORE_INT20_PIN:
					isrList[20] = func;
					interruptArgs[20] = state;
					attachInterrupt(20, isr20, mode);
					break;
			#endif
			#ifdef CORE_INT21_PIN
				case CORE_INT21_PIN:
					isrList[21] = func;
					interruptArgs[21] = state;
					attachInterrupt(21, isr21, mode);
					break;
			#endif
			#ifdef CORE_INT22_PIN
				case CORE_INT22_PIN:
					isrList[22] = func;
					interruptArgs[22] = state;
					attachInterrupt(22, isr22, mode);
					break;
			#endif
			#ifdef CORE_INT23_PIN
				case CORE_INT23_PIN:
					isrList[23] = func;
					interruptArgs[23] = state;
					attachInterrupt(23, isr23, mode);
					break;
			#endif
			#ifdef CORE_INT24_PIN
				case CORE_INT24_PIN:
					isrList[24] = func;
					interruptArgs[24] = state;
					attachInterrupt(24, isr24, mode);
					break;
			#endif
			#ifdef CORE_INT25_PIN
				case CORE_INT25_PIN:
					isrList[25] = func;
					interruptArgs[25] = state;
					attachInterrupt(25, isr25, mode);
					break;
			#endif
			#ifdef CORE_INT26_PIN
				case CORE_INT26_PIN:
					isrList[26] = func;
					interruptArgs[26] = state;
					attachInterrupt(26, isr26, mode);
					break;
			#endif
			#ifdef CORE_INT27_PIN
				case CORE_INT27_PIN:
					isrList[27] = func;
					interruptArgs[27] = state;
					attachInterrupt(27, isr27, mode);
					break;
			#endif
			#ifdef CORE_INT28_PIN
				case CORE_INT28_PIN:
					isrList[28] = func;
					interruptArgs[28] = state;
					attachInterrupt(28, isr28, mode);
					break;
			#endif
			#ifdef CORE_INT29_PIN
				case CORE_INT29_PIN:
					isrList[29] = func;
					interruptArgs[29] = state;
					attachInterrupt(29, isr29, mode);
					break;
			#endif

			#ifdef CORE_INT30_PIN
				case CORE_INT30_PIN:
					isrList[30] = func;
					interruptArgs[30] = state;
					attachInterrupt(30, isr30, mode);
					break;
			#endif
			#ifdef CORE_INT31_PIN
				case CORE_INT31_PIN:
					isrList[31] = func;
					interruptArgs[31] = state;
					attachInterrupt(31, isr31, mode);
					break;
			#endif
			#ifdef CORE_INT32_PIN
				case CORE_INT32_PIN:
					isrList[32] = func;
					interruptArgs[32] = state;
					attachInterrupt(32, isr32, mode);
					break;
			#endif
			#ifdef CORE_INT33_PIN
				case CORE_INT33_PIN:
					isrList[33] = func;
					interruptArgs[33] = state;
					attachInterrupt(33, isr33, mode);
					break;
			#endif
			#ifdef CORE_INT34_PIN
				case CORE_INT34_PIN:
					isrList[34] = func;
					interruptArgs[34] = state;
					attachInterrupt(34, isr34, mode);
					break;
			#endif
			#ifdef CORE_INT35_PIN
				case CORE_INT35_PIN:
					isrList[35] = func;
					interruptArgs[35] = state;
					attachInterrupt(35, isr35, mode);
					break;
			#endif
			#ifdef CORE_INT36_PIN
				case CORE_INT36_PIN:
					isrList[36] = func;
					interruptArgs[36] = state;
					attachInterrupt(36, isr36, mode);
					break;
			#endif
			#ifdef CORE_INT37_PIN
				case CORE_INT37_PIN:
					isrList[37] = func;
					interruptArgs[37] = state;
					attachInterrupt(37, isr37, mode);
					break;
			#endif
			#ifdef CORE_INT38_PIN
				case CORE_INT38_PIN:
					isrList[38] = func;
					interruptArgs[38] = state;
					attachInterrupt(38, isr38, mode);
					break;
			#endif
			#ifdef CORE_INT39_PIN
				case CORE_INT39_PIN:
					isrList[39] = func;
					interruptArgs[39] = state;
					attachInterrupt(39, isr39, mode);
					break;
			#endif
			#ifdef CORE_INT40_PIN
				case CORE_INT40_PIN:
					isrList[40] = func;
					interruptArgs[40] = state;
					attachInterrupt(40, isr40, mode);
					break;
			#endif
			#ifdef CORE_INT41_PIN
				case CORE_INT41_PIN:
					isrList[41] = func;
					interruptArgs[41] = state;
					attachInterrupt(41, isr41, mode);
					break;
			#endif
			#ifdef CORE_INT42_PIN
				case CORE_INT42_PIN:
					isrList[42] = func;
					interruptArgs[42] = state;
					attachInterrupt(42, isr42, mode);
					break;
			#endif
			#ifdef CORE_INT43_PIN
				case CORE_INT43_PIN:
					isrList[43] = func;
					interruptArgs[43] = state;
					attachInterrupt(43, isr43, mode);
					break;
			#endif
			#ifdef CORE_INT44_PIN
				case CORE_INT44_PIN:
					isrList[44] = func;
					interruptArgs[44] = state;
					attachInterrupt(44, isr44, mode);
					break;
			#endif
			#ifdef CORE_INT45_PIN
				case CORE_INT45_PIN:
					isrList[45] = func;
					interruptArgs[45] = state;
					attachInterrupt(45, isr45, mode);
					break;
			#endif
			#ifdef CORE_INT46_PIN
				case CORE_INT46_PIN:
					isrList[46] = func;
					interruptArgs[46] = state;
					attachInterrupt(46, isr46, mode);
					break;
			#endif
			#ifdef CORE_INT47_PIN
				case CORE_INT47_PIN:
					isrList[47] = func;
					interruptArgs[47] = state;
					attachInterrupt(47, isr47, mode);
					break;
			#endif
			#ifdef CORE_INT48_PIN
				case CORE_INT48_PIN:
					isrList[48] = func;
					interruptArgs[48] = state;
					attachInterrupt(48, isr48, mode);
					break;
			#endif
			#ifdef CORE_INT49_PIN
				case CORE_INT49_PIN:
					isrList[49] = func;
					interruptArgs[49] = state;
					attachInterrupt(49, isr49, mode);
					break;
			#endif
			#ifdef CORE_INT50_PIN
				case CORE_INT50_PIN:
					isrList[50] = func;
					interruptArgs[50] = state;
					attachInterrupt(50, isr50, mode);
					break;
			#endif
			#ifdef CORE_INT51_PIN
				case CORE_INT51_PIN:
					isrList[51] = func;
					interruptArgs[51] = state;
					attachInterrupt(51, isr51, mode);
					break;
			#endif
			#ifdef CORE_INT52_PIN
				case CORE_INT52_PIN:
					isrList[52] = func;
					interruptArgs[52] = state;
					attachInterrupt(52, isr52, mode);
					break;
			#endif
			#ifdef CORE_INT53_PIN
				case CORE_INT53_PIN:
					isrList[53] = func;
					interruptArgs[53] = state;
					attachInterrupt(53, isr53, mode);
					break;
			#endif
			#ifdef CORE_INT54_PIN
				case CORE_INT54_PIN:
					isrList[54] = func;
					interruptArgs[54] = state;
					attachInterrupt(54, isr54, mode);
					break;
			#endif
			#ifdef CORE_INT55_PIN
				case CORE_INT55_PIN:
					isrList[55] = func;
					interruptArgs[55] = state;
					attachInterrupt(55, isr55, mode);
					break;
			#endif
			#ifdef CORE_INT56_PIN
				case CORE_INT56_PIN:
					isrList[56] = func;
					interruptArgs[56] = state;
					attachInterrupt(56, isr56, mode);
					break;
			#endif
			#ifdef CORE_INT57_PIN
				case CORE_INT57_PIN:
					isrList[57] = func;
					interruptArgs[57] = state;
					attachInterrupt(57, isr57, mode);
					break;
			#endif
			#ifdef CORE_INT58_PIN
				case CORE_INT58_PIN:
					isrList[58] = func;
					interruptArgs[58] = state;
					attachInterrupt(58, isr58, mode);
					break;
			#endif
			#ifdef CORE_INT59_PIN
				case CORE_INT59_PIN:
					isrList[59] = func;
					interruptArgs[59] = state;
					attachInterrupt(59, isr59, mode);
					break;
			#endif
				default:
					return 0;
			}
			return 1;
		}


#if defined(ENCODER_USE_INTERRUPTS)
	#ifdef CORE_INT0_PIN
	static ISR_ATTR void isr0(void) { isrList[0](interruptArgs[0]); }
	#endif
	#ifdef CORE_INT1_PIN
	static ISR_ATTR void isr1(void) { isrList[1](interruptArgs[1]); }
	#endif
	#ifdef CORE_INT2_PIN
	static ISR_ATTR void isr2(void) { isrList[2](interruptArgs[2]); }
	#endif
	#ifdef CORE_INT3_PIN
	static ISR_ATTR void isr3(void) { isrList[3](interruptArgs[3]); }
	#endif
	#ifdef CORE_INT4_PIN
	static ISR_ATTR void isr4(void) { isrList[4](interruptArgs[4]); }
	#endif
	#ifdef CORE_INT5_PIN
	static ISR_ATTR void isr5(void) { isrList[5](interruptArgs[5]); }
	#endif
	#ifdef CORE_INT6_PIN
	static ISR_ATTR void isr6(void) { isrList[6](interruptArgs[6]); }
	#endif
	#ifdef CORE_INT7_PIN
	static ISR_ATTR void isr7(void) { isrList[7](interruptArgs[7]); }
	#endif
	#ifdef CORE_INT8_PIN
	static ISR_ATTR void isr8(void) { isrList[8](interruptArgs[8]); }
	#endif
	#ifdef CORE_INT9_PIN
	static ISR_ATTR void isr9(void) { isrList[9](interruptArgs[9]); }
	#endif
	#ifdef CORE_INT10_PIN
	static ISR_ATTR void isr10(void) { isrList[10](interruptArgs[10]); }
	#endif
	#ifdef CORE_INT11_PIN
	static ISR_ATTR void isr11(void) { isrList[11](interruptArgs[11]); }
	#endif
	#ifdef CORE_INT12_PIN
	static ISR_ATTR void isr12(void) { isrList[12](interruptArgs[12]); }
	#endif
	#ifdef CORE_INT13_PIN
	static ISR_ATTR void isr13(void) { isrList[13](interruptArgs[13]); }
	#endif
	#ifdef CORE_INT14_PIN
	static ISR_ATTR void isr14(void) { isrList[14](interruptArgs[14]); }
	#endif
	#ifdef CORE_INT15_PIN
	static ISR_ATTR void isr15(void) { isrList[15](interruptArgs[15]); }
	#endif
	#ifdef CORE_INT16_PIN
	static ISR_ATTR void isr16(void) { isrList[16](interruptArgs[16]); }
	#endif
	#ifdef CORE_INT17_PIN
	static ISR_ATTR void isr17(void) { isrList[17](interruptArgs[17]); }
	#endif
	#ifdef CORE_INT18_PIN
	static ISR_ATTR void isr18(void) { isrList[18](interruptArgs[18]); }
	#endif
	#ifdef CORE_INT19_PIN
	static ISR_ATTR void isr19(void) { isrList[19](interruptArgs[19]); }
	#endif
	#ifdef CORE_INT20_PIN
	static ISR_ATTR void isr20(void) { isrList[20](interruptArgs[20]); }
	#endif
	#ifdef CORE_INT21_PIN
	static ISR_ATTR void isr21(void) { isrList[21](interruptArgs[21]); }
	#endif
	#ifdef CORE_INT22_PIN
	static ISR_ATTR void isr22(void) { isrList[22](interruptArgs[22]); }
	#endif
	#ifdef CORE_INT23_PIN
	static ISR_ATTR void isr23(void) { isrList[23](interruptArgs[23]); }
	#endif
	#ifdef CORE_INT24_PIN
	static ISR_ATTR void isr24(void) { isrList[24](interruptArgs[24]); }
	#endif
	#ifdef CORE_INT25_PIN
	static ISR_ATTR void isr25(void) { isrList[25](interruptArgs[25]); }
	#endif
	#ifdef CORE_INT26_PIN
	static ISR_ATTR void isr26(void) { isrList[26](interruptArgs[26]); }
	#endif
	#ifdef CORE_INT27_PIN
	static ISR_ATTR void isr27(void) { isrList[27](interruptArgs[27]); }
	#endif
	#ifdef CORE_INT28_PIN
	static ISR_ATTR void isr28(void) { isrList[28](interruptArgs[28]); }
	#endif
	#ifdef CORE_INT29_PIN
	static ISR_ATTR void isr29(void) { isrList[29](interruptArgs[29]); }
	#endif
	#ifdef CORE_INT30_PIN
	static ISR_ATTR void isr30(void) { isrList[30](interruptArgs[30]); }
	#endif
	#ifdef CORE_INT31_PIN
	static ISR_ATTR void isr31(void) { isrList[31](interruptArgs[31]); }
	#endif
	#ifdef CORE_INT32_PIN
	static ISR_ATTR void isr32(void) { isrList[32](interruptArgs[32]); }
	#endif
	#ifdef CORE_INT33_PIN
	static ISR_ATTR void isr33(void) { isrList[33](interruptArgs[33]); }
	#endif
	#ifdef CORE_INT34_PIN
	static ISR_ATTR void isr34(void) { isrList[34](interruptArgs[34]); }
	#endif
	#ifdef CORE_INT35_PIN
	static ISR_ATTR void isr35(void) { isrList[35](interruptArgs[35]); }
	#endif
	#ifdef CORE_INT36_PIN
	static ISR_ATTR void isr36(void) { isrList[36](interruptArgs[36]); }
	#endif
	#ifdef CORE_INT37_PIN
	static ISR_ATTR void isr37(void) { isrList[37](interruptArgs[37]); }
	#endif
	#ifdef CORE_INT38_PIN
	static ISR_ATTR void isr38(void) { isrList[38](interruptArgs[38]); }
	#endif
	#ifdef CORE_INT39_PIN
	static ISR_ATTR void isr39(void) { isrList[39](interruptArgs[39]); }
	#endif
	#ifdef CORE_INT40_PIN
	static ISR_ATTR void isr40(void) { isrList[40](interruptArgs[40]); }
	#endif
	#ifdef CORE_INT41_PIN
	static ISR_ATTR void isr41(void) { isrList[41](interruptArgs[41]); }
	#endif
	#ifdef CORE_INT42_PIN
	static ISR_ATTR void isr42(void) { isrList[42](interruptArgs[42]); }
	#endif
	#ifdef CORE_INT43_PIN
	static ISR_ATTR void isr43(void) { isrList[43](interruptArgs[43]); }
	#endif
	#ifdef CORE_INT44_PIN
	static ISR_ATTR void isr44(void) { isrList[44](interruptArgs[44]); }
	#endif
	#ifdef CORE_INT45_PIN
	static ISR_ATTR void isr45(void) { isrList[45](interruptArgs[45]); }
	#endif
	#ifdef CORE_INT46_PIN
	static ISR_ATTR void isr46(void) { isrList[46](interruptArgs[46]); }
	#endif
	#ifdef CORE_INT47_PIN
	static ISR_ATTR void isr47(void) { isrList[47](interruptArgs[47]); }
	#endif
	#ifdef CORE_INT48_PIN
	static ISR_ATTR void isr48(void) { isrList[48](interruptArgs[48]); }
	#endif
	#ifdef CORE_INT49_PIN
	static ISR_ATTR void isr49(void) { isrList[49](interruptArgs[49]); }
	#endif
	#ifdef CORE_INT50_PIN
	static ISR_ATTR void isr50(void) { isrList[50](interruptArgs[50]); }
	#endif
	#ifdef CORE_INT51_PIN
	static ISR_ATTR void isr51(void) { isrList[51](interruptArgs[51]); }
	#endif
	#ifdef CORE_INT52_PIN
	static ISR_ATTR void isr52(void) { isrList[52](interruptArgs[52]); }
	#endif
	#ifdef CORE_INT53_PIN
	static ISR_ATTR void isr53(void) { isrList[53](interruptArgs[53]); }
	#endif
	#ifdef CORE_INT54_PIN
	static ISR_ATTR void isr54(void) { isrList[54](interruptArgs[54]); }
	#endif
	#ifdef CORE_INT55_PIN
	static ISR_ATTR void isr55(void) { isrList[55](interruptArgs[55]); }
	#endif
	#ifdef CORE_INT56_PIN
	static ISR_ATTR void isr56(void) { isrList[56](interruptArgs[56]); }
	#endif
	#ifdef CORE_INT57_PIN
	static ISR_ATTR void isr57(void) { isrList[57](interruptArgs[57]); }
	#endif
	#ifdef CORE_INT58_PIN
	static ISR_ATTR void isr58(void) { isrList[58](interruptArgs[58]); }
	#endif
	#ifdef CORE_INT59_PIN
	static ISR_ATTR void isr59(void) { isrList[59](interruptArgs[59]); }
	#endif

#endif


		static uint8_t check_interrupt(uint8_t pin) {
			switch (pin) {
			#ifdef CORE_INT0_PIN
				case CORE_INT0_PIN:
					break;
			#endif
			#ifdef CORE_INT1_PIN
				case CORE_INT1_PIN:
					break;
			#endif
			#ifdef CORE_INT2_PIN
				case CORE_INT2_PIN:
					break;
			#endif
			#ifdef CORE_INT3_PIN
				case CORE_INT3_PIN:
					break;
			#endif
			#ifdef CORE_INT4_PIN
				case CORE_INT4_PIN:
					break;
			#endif
			#ifdef CORE_INT5_PIN
				case CORE_INT5_PIN:
					break;
			#endif
			#ifdef CORE_INT6_PIN
				case CORE_INT6_PIN:
					break;
			#endif
			#ifdef CORE_INT7_PIN
				case CORE_INT7_PIN:
					break;
			#endif
			#ifdef CORE_INT8_PIN
				case CORE_INT8_PIN:
					break;
			#endif
			#ifdef CORE_INT9_PIN
				case CORE_INT9_PIN:
					break;
			#endif
			#ifdef CORE_INT10_PIN
				case CORE_INT10_PIN:
					break;
			#endif
			#ifdef CORE_INT11_PIN
				case CORE_INT11_PIN:
					break;
			#endif
			#ifdef CORE_INT12_PIN
				case CORE_INT12_PIN:
					break;
			#endif
			#ifdef CORE_INT13_PIN
				case CORE_INT13_PIN:
					break;
			#endif
			#ifdef CORE_INT14_PIN
				case CORE_INT14_PIN:
					break;
			#endif
			#ifdef CORE_INT15_PIN
				case CORE_INT15_PIN:
					break;
			#endif
			#ifdef CORE_INT16_PIN
				case CORE_INT16_PIN:
					break;
			#endif
			#ifdef CORE_INT17_PIN
				case CORE_INT17_PIN:
					break;
			#endif
			#ifdef CORE_INT18_PIN
				case CORE_INT18_PIN:
					break;
			#endif
			#ifdef CORE_INT19_PIN
				case CORE_INT19_PIN:
					break;
			#endif
			#ifdef CORE_INT20_PIN
				case CORE_INT20_PIN:
					break;
			#endif
			#ifdef CORE_INT21_PIN
				case CORE_INT21_PIN:
					break;
			#endif
			#ifdef CORE_INT22_PIN
				case CORE_INT22_PIN:
					break;
			#endif
			#ifdef CORE_INT23_PIN
				case CORE_INT23_PIN:
					break;
			#endif
			#ifdef CORE_INT24_PIN
				case CORE_INT24_PIN:
					break;
			#endif
			#ifdef CORE_INT25_PIN
				case CORE_INT25_PIN:
			#endif
			#ifdef CORE_INT26_PIN
				case CORE_INT26_PIN:
					break;
			#endif
			#ifdef CORE_INT27_PIN
				case CORE_INT27_PIN:
					break;
			#endif
			#ifdef CORE_INT28_PIN
				case CORE_INT28_PIN:
					break;
			#endif
			#ifdef CORE_INT29_PIN
				case CORE_INT29_PIN:
					break;
			#endif

			#ifdef CORE_INT30_PIN
				case CORE_INT30_PIN:
					break;
			#endif
			#ifdef CORE_INT31_PIN
				case CORE_INT31_PIN:
					break;
			#endif
			#ifdef CORE_INT32_PIN
				case CORE_INT32_PIN:
					break;
			#endif
			#ifdef CORE_INT33_PIN
				case CORE_INT33_PIN:
					break;
			#endif
			#ifdef CORE_INT34_PIN
				case CORE_INT34_PIN:
					break;
			#endif
			#ifdef CORE_INT35_PIN
				case CORE_INT35_PIN:
					break;
			#endif
			#ifdef CORE_INT36_PIN
				case CORE_INT36_PIN:
					break;
			#endif
			#ifdef CORE_INT37_PIN
				case CORE_INT37_PIN:
					break;
			#endif
			#ifdef CORE_INT38_PIN
				case CORE_INT38_PIN:
					break;
			#endif
			#ifdef CORE_INT39_PIN
				case CORE_INT39_PIN:
					break;
			#endif
			#ifdef CORE_INT40_PIN
				case CORE_INT40_PIN:
					break;
			#endif
			#ifdef CORE_INT41_PIN
				case CORE_INT41_PIN:
					break;
			#endif
			#ifdef CORE_INT42_PIN
				case CORE_INT42_PIN:
					break;
			#endif
			#ifdef CORE_INT43_PIN
				case CORE_INT43_PIN:
					break;
			#endif
			#ifdef CORE_INT44_PIN
				case CORE_INT44_PIN:
					break;
			#endif
			#ifdef CORE_INT45_PIN
				case CORE_INT45_PIN:
					break;
			#endif
			#ifdef CORE_INT46_PIN
				case CORE_INT46_PIN:
					break;
			#endif
			#ifdef CORE_INT47_PIN
				case CORE_INT47_PIN:
					break;
			#endif
			#ifdef CORE_INT48_PIN
				case CORE_INT48_PIN:
					break;
			#endif
			#ifdef CORE_INT49_PIN
				case CORE_INT49_PIN:
					break;
			#endif
			#ifdef CORE_INT50_PIN
				case CORE_INT50_PIN:
					break;
			#endif
			#ifdef CORE_INT51_PIN
				case CORE_INT51_PIN:
					break;
			#endif
			#ifdef CORE_INT52_PIN
				case CORE_INT52_PIN:
					break;
			#endif
			#ifdef CORE_INT53_PIN
				case CORE_INT53_PIN:
					break;
			#endif
			#ifdef CORE_INT54_PIN
				case CORE_INT54_PIN:
					break;
			#endif
			#ifdef CORE_INT55_PIN
				case CORE_INT55_PIN:
					break;
			#endif
			#ifdef CORE_INT56_PIN
				case CORE_INT56_PIN:
					break;
			#endif
			#ifdef CORE_INT57_PIN
				case CORE_INT57_PIN:
					break;
			#endif
			#ifdef CORE_INT58_PIN
				case CORE_INT58_PIN:
					break;
			#endif
			#ifdef CORE_INT59_PIN
				case CORE_INT59_PIN:
					break;
			#endif
				default:
					return 0;
			}
			return 1;
		}



};



#endif
