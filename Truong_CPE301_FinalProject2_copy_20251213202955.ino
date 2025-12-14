
void setup() {
  DDRB = 0b11110000;
  /* designates as outputs
  bit 7: RED LED
  bit 6: YELLOW LED
  bit 5: GREEN LED
  bit 4: BLUE LED
  */
  PORTB = 0b01000000; //starting in idle
  DDRG & 0b11011111; //sets pin 4 (stop button)
  PORTG & 0b11011111;

  //setup for interrupts
  SREG = 0b10000000; //globally enables interrupts
  attachInterrupt(digitalPinToInterrupt(2), START, FALLING);

  Serial.begin(9600);
}

void loop() {

}

//switches to start state
void START() {
  PORTB = 0b00100000;
}