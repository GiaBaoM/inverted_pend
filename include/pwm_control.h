#include <Arduino.h>
/*
This pins are together, can not modify frequency individually,same timmer):

pin 13, 4
pin 12, 11
pin 10, 9
pin 5, 3, 2
pin 8, 7, 6

Divisor 	        Frequency

1 	 	62500 Hz
2 	 	7812.5 Hz
3 	 	976.5625 Hz  <--DEFAULT Diecimila bootloader
4 	 	244.140625 Hz
5 	 	61.03515625 Hz  

For pins 2 to 13 EXCEPT 13,4: Divisor Frequency

1 	 31372.55 Hz
2	 3921.16  Hz
3	 490.20    Hz   <--DEFAULT Diecimila bootloader
4	 122.55    Hz
5	 30.610    Hz	
*/
#if  defined(__AVR_ATmega2560__)
void setPwmFrequencyMEGA2560(int pin, int divisor) {
  byte mode;
      switch(divisor) {
      case 1: mode = 0x01; break;
      case 2: mode = 0x02; break;
      case 3: mode = 0x03; break;
      case 4: mode = 0x04; break;
      case 5: mode = 0x05; break;
      case 6: mode = 0x06; break;
      case 7: mode = 0x07; break;
      default: return;
      }
      
        switch(pin) {	  
      case 2:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 3:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 4:  TCCR0B = TCCR0B  & 0b11111000 | mode; break;
      case 5:  TCCR3B = TCCR3B  & 0b11111000 | mode; break;
      case 6:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 7:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 8:  TCCR4B = TCCR4B  & 0b11111000 | mode; break;
      case 9:  TCCR2B = TCCR0B  & 0b11111000 | mode; break;
      case 10: TCCR2B = TCCR2B  & 0b11111000 | mode; break;
      case 11: TCCR1B = TCCR1B  & 0b11111000 | mode; break;  
      case 12: TCCR1B = TCCR1B  & 0b11111000 | mode; break;  
      case 13: TCCR0B = TCCR0B  & 0b11111000 | mode; break;
      default: return;
    }

}
	
  #endif