
//libraries
#include "DHT.h" 
#include "LiquidCrystal.h"

//creating objects
DHT dht(9, DHT11);
LiquidCrystal lcd(50, 51, 5, 6, 7, 8);

//declaring vars
unsigned long prevMillis = 0;
const long interval = 60000;

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
  lcd.begin(16, 2); //starting LCD
  lcd.print("Hellow, world");
  Serial.begin(9600);
}

void loop() {
  int state = PORTB & 0b11110000;
  while(state == 0b00100000){
    Serial.println("State: idle");

    //only updates lcd every minute
    unsigned long currentMillis = millis();
    Serial.print("ms: ");
    Serial.println(currentMillis);

    if ((currentMillis - prevMillis) >= interval){

      prevMillis = currentMillis;
      float temp = dht.readTemperature();
      float humidity = dht.readHumidity();

      lcd.setCursor(0,0);
      lcd.print("Temp: ");
      lcd.print(temp);
      lcd.print(0xB0);
      lcd.print(" C");
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      lcd.print("%");
    }

    //pressing stop button returns state to disabled
    int stopState = PING;
    if(PING == 0){
      PORTB = PORTB << 1;
      state = PORTB & 0b11110000;
    }
    delay(100);
  }
  Serial.println("State: disabled");
  lcd.clear();
  //outside of while loop, disabled state
  delay(100);
}

//switches to start state
void START() {
  PORTB = 0b00100000;
}
