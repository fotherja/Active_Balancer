#include "Support.h"
#include "Arduino.h"

void Init_Timers_Pins (void)
{
  // Configure Timers:
  TCCR0A = 0b10100011;
  TCCR0B = 0b00000001;  
  OCR0A = 0;
  OCR0B = 0; 

  TCCR1A = 0b10100001;
  TCCR1B = 0b00001001;
  OCR1A = 0;
  OCR1B = 0;

  TCCR2A = 0b10100011;
  TCCR2B = 0b00000001;
  TIMSK2 = 0b00000001;
  OCR2A = 0;
  OCR2B = 0;

  // Set pin directions
  pinMode(ENABLE_CH1, OUTPUT);
  pinMode(ENABLE_CH2, OUTPUT);
  pinMode(ENABLE_CH3, OUTPUT);
  pinMode(ENABLE_CH4, OUTPUT);
  pinMode(ENABLE_CH5, OUTPUT);
  pinMode(ENABLE_CH6, OUTPUT);

  pinMode(OUT_CH1, OUTPUT);
  pinMode(OUT_CH2, OUTPUT);
  pinMode(OUT_CH3, OUTPUT);
  pinMode(OUT_CH4, OUTPUT);
  pinMode(OUT_CH5, OUTPUT);
  pinMode(OUT_CH6, OUTPUT);

  pinMode(SLEEP, OUTPUT);  
}

