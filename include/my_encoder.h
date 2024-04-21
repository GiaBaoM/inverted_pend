#include <Arduino.h>
#define NUMBER_OF_ENCODERS 3
#define INTERRUPT_MODE RISING

class Encoder {
public:
Encoder(uint8_t enc,uint8_t pinA, uint8_t pinB, uint8_t mode ) : pinA(pinA), pinB(pinB), pos(0) {
	pinMode(pinA, INPUT_PULLUP);
	pinMode(pinB, INPUT_PULLUP);
	instances[getIntNum(pinA)] = this;
	instances[getIntNum(pinB)] = this;
	switch(enc)
	{
		case 0:
			if (mode == CHANGE)
			{
				attachInterrupt(getIntNum(pinA),irsPinAEn0, mode);
				attachInterrupt(getIntNum(pinB),irsPinBEn0, mode);
			} else if (mode == RISING || mode == FALLING)
			{
				attachInterrupt(getIntNum(pinA),irsPinAEn0, mode);
			}
			instances[0] = this;
			break;
		case 1:
			if (mode == CHANGE)
			{
				attachInterrupt(getIntNum(pinA),irsPinAEn1, mode);
				attachInterrupt(getIntNum(pinB),irsPinBEn1, mode);
			} else if (mode == RISING || mode == FALLING)
			{
				attachInterrupt(getIntNum(pinA),irsPinAEn1, mode);
			}
			instances[1] = this;
			break;
		case 2:
			if (mode == CHANGE)
			{
				attachInterrupt(getIntNum(pinA),irsPinAEn2, mode);
				attachInterrupt(getIntNum(pinB),irsPinBEn2, mode);
			} else if (mode == RISING || mode == FALLING)
			{
				attachInterrupt(getIntNum(pinA),irsPinAEn2, mode);
			}
			instances[2] = this;
			break;
	}
}

int32_t read() {
	return pos;
}

void write(int32_t newPos) {
    pos = newPos;
}
private:
    uint8_t pinA;
	uint8_t pinB;
	int32_t pos;
    static Encoder* instances[NUMBER_OF_ENCODERS];
	void debounce(int del) {
		for (int k=0;k<del;k++) {
			/* can't use delay in the ISR so need to waste some time
			perfoming operations, this uses roughly 0.1ms on uno  */
			k = k +0.0 +0.0 -0.0 +3.0 -3.0;
		}
	}
	static void irsPinAEn0(){
		//if (Encoder::instances [0] != NULL)
      Encoder::instances [0]-> irsPinA();
	};
	static void irsPinBEn0(){
		//if (Encoder::instances [0] != NULL)
      Encoder::instances [0]-> irsPinB();
	};
	static void irsPinAEn1(){
		//if (Encoder::instances [1] != NULL)
      Encoder::instances [1]-> irsPinA();
	};
	static void irsPinBEn1(){
		//if (Encoder::instances [1] != NULL)
      Encoder::instances [1]-> irsPinB();
	};
	static void irsPinAEn2(){
		//if (Encoder::instances [2] != NULL)
      Encoder::instances [2]-> irsPinA();
	};
	static void irsPinBEn2(){
		//if (Encoder::instances [2] != NULL)
      Encoder::instances [2]-> irsPinB();
	};
	void irsPinA(){
		#if INTERRUPT_MODE == CHANGE
		/* read pin B right away                                   */
		int drB = digitalRead(pinB);
		/* possibly wait before reading pin A, then read it        */
		//debounce(2);
		int drA = digitalRead(pinA);
		/* this updates the counter                                */
		if (drA == HIGH)
		{ /* low->high on A? */
			if (drB == LOW) pos++; /* going clockwise: increment         */
			else pos--; /* going counterclockwise: decrement  */
		}
		else
		{ 
			if (drB == HIGH) pos++;
			else pos--;
		} 
		#endif
		#if INTERRUPT_MODE == RISING
		int drB = digitalRead(pinB);
		if (drB == HIGH) pos--;
		else pos++;
		#endif
		#if INTERRUPT_MODE == FALLING
		int drB = digitalRead(pinB);
		if (drB == HIGH) pos++;
		else pos--;
		#endif
	};
	void irsPinB(){
		#if INTERRUPT_MODE == CHANGE
		/* read pin A right away                                   */
		int drA = digitalRead(pinA);
		/* possibly wait before reading pin B, then read it        */
		//debounce(2);
		int drB = digitalRead(pinB);
		/* this updates the counter                                */
		if (drB == HIGH)
		{ /* low->high on B? */
			if (drA == HIGH) pos++; /* going clockwise: increment         */
			else  pos--; /* going counterclockwise: decrement  */
		}
		else
		{ /* must be high to low on B */
			if (drA == LOW) pos++; /* going clockwise: increment         */
			else  pos--; /* going counterclockwise: decrement  */
		} /* end counter update */
		#endif

		#if INTERRUPT_MODE == RISING
		int drA = digitalRead(pinA);
		if (drA == HIGH) pos++; /* going clockwise: increment         */
		else  pos--; /* going counterclockwise: decrement  */
		#endif

		#if INTERRUPT_MODE == FALLING
		int drA = digitalRead(pinA);
		if (drA == HIGH) pos--; /* going clockwise: increment         */
		else  pos++; /* going counterclockwise: decrement  */
		#endif
	};
    int getIntNum(int pin) {
        // Implement this method based on your microcontroller's pin to interrupt number mapping
		{
			/* returns the interrupt number for a given interrupt pin 
			see http://arduino.cc/it/Reference/AttachInterrupt        */
			switch (pin)
			{
			case 2:
				return 0;
			case 3:
				return 1;
			case 21:
				return 2;
			case 20:
				return 3;
			case 19:
				return 4;
			case 18:
				return 5;
			default:
				return -1;
			}
		}
	}

};