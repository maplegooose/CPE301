
//libraries
#include "DHT.h" 
#include "LiquidCrystal.h"

//defining address since ADC register takes up two bytes
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

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

  adc_init(); //initializing analog input

  dht.begin(); //starting the dht sensor
  lcd.begin(16, 2); //starting LCD
  lcd.print("Hellow, world");
  Serial.begin(9600);
}

void loop() {
  int state = PORTB & 0b11110000;
  while(state == 0b00100000){
    Serial.println("State: idle");

    //grabing data from DHT
    float temp = dht.readTemperature(true);
    float humidity = dht.readHumidity();
    float water = adc_read(0);

    Serial.print("water level: ");
    Serial.println(water);

    //only updates lcd every minute
    unsigned long currentMillis = millis();

    if ((currentMillis - prevMillis) >= interval){

      prevMillis = currentMillis;

      lcd.setCursor(0,0);
      lcd.print("Temp: ");
      lcd.print(temp);
      lcd.print(" F");
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

void adc_init() //initializes ADC for water sensor
{
  ADCSRA |= (1 << 7);  
  ADCSRA &= ~(1 << 5); 
  ADCSRA &= ~(1 << 3); 
  ADCSRA |= (1 << 2);  
  ADCSRA |= (1 << 1);
  ADCSRA |= (1 << 0);

  ADCSRB &= ~(1 << 3); 
  ADCSRB &= 0xF8;

  ADMUX &= ~(1 << 7);  
  ADMUX |= (1 << 6);   
  ADMUX &= ~(1 << 5);  
  ADMUX &= 0xE0;
}

unsigned int adc_read(unsigned char adc_channel_num) //work with channel 0
{
  ADMUX &= 0xE0;
  ADCSRB &= ~(1 << 3);

  if(adc_channel_num > 7){
    adc_channel_num = adc_channel_num -8;
      ADCSRB |= (1 << 3);
  }

  ADMUX |= adc_channel_num;
  ADCSRA |= (1 << 6);

  while((ADCSRA & (1 <<6)) != 0);
  
  return *my_ADC_DATA;
}

