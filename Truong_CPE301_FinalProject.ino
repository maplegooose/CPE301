#include "DHT.h" 

//creating objects
DHT dht(2, DHT11);

void setup() {
  DDRB = 0b11110000;
  /* designates as outputs
  bit 7: RED LED
  bit 6: YELLOW LED
  bit 5: GREEN LED
  bit 4: BLUE LED
  */
  DDRD = DDRD & 0b11111100;
  PORTD = DDRD & 0B11111100;
  /* designates as inputs
  bit 5: stop button
  bit 4: start button
  */
  DDRE = DDRE | 0b00010000; 
  PORTE = PORTE | 0b00010000;
  /* designates as input
  bit 4: DHT sensor
  */

  //setup for interrupts
  SREG = 0b10000000; //globally enables interrupts
  EICRA = 0b00001010; //triggers flag on falling edge for INT1:0
  EIMSK = 0b00000011; //enables interrupts for INT1:0

  PORTB = 0b01000000; //starting in idle

  dht.begin();
  Serial.begin(9600);
}

void loop() {
  int state = PORTB;

  if(state != 0b01000000){
    float temp = dht.readTemperature();
    Serial.println(temp);
  }
  else{
    Serial.println("idle");
  }
  delay(2000);
}

//switches to disabled state
ISR(INT0_vect){
  PORTB = 0b01000000;
}

//switches to idle state
ISR(INT1_vect){
  PORTB = 0b00100000;
}
