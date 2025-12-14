//libraries
#include "DHT.h" 

//creating objects
DHT dht(9, DHT11);

void setup() {
  DDRB = 0b11110000;
  /* designates as outputs
  bit 7: RED LED
  bit 6: YELLOW LED
  bit 5: GREEN LED
  bit 4: BLUE LED
  */
  PORTB = 0b01000000; //starting in idle

  DDRG & 0b11011111; //calls pin 4 an input (stop button)
  PORTG & 0b11011111;

  //setup for interrupts
  SREG = 0b10000000; //globally enables interrupts
  attachInterrupt(digitalPinToInterrupt(2), START, FALLING); //setting up START ISR

  dht.begin(); //starting the dht sensor
  Serial.begin(9600);
}

void loop() {
  while(PORTB == 0b00100000){
    //Serial.println("State: idle");

    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();

    Serial.print("temp: ");
    Serial.println(temp);
    Serial.print("humidity ");
    Serial.println(humidity);

    //pressing stop button returns state to disabled
    int stopState = PING;
    if(PING == 0){
      PORTB = PORTB << 1;
    }
  }
  Serial.println("State: disabled");
  //outside of while loop, disabled state
}

//switches to start state
void START() {
  PORTB = 0b00100000;
}
