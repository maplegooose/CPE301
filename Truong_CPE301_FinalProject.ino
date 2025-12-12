void setup() {
  DDRB = DDRB | 0b11110000;
  /*
  bit 7: RED LED
  bit 6: YELLOW LED
  bit 5: GREEN LED
  bit 4: BLUE LED
  */
}

void loop() {
  PORTB = PORTB | 0b11110000;
  delay(100);
  PORTB = PORTB & 0b00001111;
  delay(100);
}
