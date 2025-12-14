
//libraries
#include "DHT.h" 
#include "LiquidCrystal.h"

//defining address since ADC register takes up two bytes
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//creating objects
DHT dht(22, DHT11);
LiquidCrystal lcd(50, 51, 5, 6, 7, 8);

//declaring vars
unsigned long prevMillis = 0;
const long interval = 60000;
const float tempThreshold = 69.00;
const int waterThreshold = -1;
int state = PORTB & 0b11110000;

void setup() {
  DDRB = 0b11110000;
  /* designates as outputs
  bit 7: RED LED
  bit 6: YELLOW LED
  bit 5: GREEN LED
  bit 4: BLUE LED
  */
  PORTB = 0b01000000; //starting in idle

  DDRE & 0b11011111; //calls pin 3 an input (reset button)
  PORTE & 0b11011111;

  DDRG & 0b11011111; //calls pin 4 an input (stop button)
  PORTG & 0b11011111;

  //setup for interrupts
  SREG = 0b10000000; //globally enables interrupts
  attachInterrupt(digitalPinToInterrupt(2), START, FALLING); //setting up START ISR

  adc_init(); //initializing analog input

  dht.begin(); //starting the dht sensor
  lcd.begin(16, 2); //starting LCD
  Serial.begin(9600);
}

void loop() {
  int state = PORTB & 0b11110000;
  //while loop is now "active" state
  //outside of while loop, it is in disabled state
  while(state != 0b01000000){
    state = PORTB & 0b11110000;

    //grabing data from DHT
    float temp = dht.readTemperature(true);
    float humidity = dht.readHumidity();
    float water = adc_read(0);

    Serial.println(temp);

    if (water < waterThreshold){ //if met, activates error state
      PORTB = PORTB & 0b00001111;
      PORTB = PORTB | 0b10000000;
      state = PORTB & 0b11110000;
      lcd.clear();
    } 
    
    while(state == 0b10000000){ //error state code (can't break out unless conditions met)

      lcd.setCursor(0, 0);
      lcd.write("ERROR: LOW WATER");

      analogWrite(9, 0);

      int resetState = !(PINE & 0b00100000);
      float water = adc_read(0);

      //checks if conditions for reset to idle has been met
      if((water > waterThreshold) && (resetState)){
        PORTB = PORTB & 0b00001111;
        PORTB = PORTB | 0b01000000;
        state = PORTB & 0b11110000;
        Serial.print("debug 1");
      }

      state = checkStop(state);
      state = PORTB & 0b11110000;

    }
    
    if ((temp > tempThreshold) && (water > waterThreshold)){ //if met, activates running state
      PORTB = PORTB & 0b00001111;
      PORTB = PORTB | 0b00010000;
      state = PORTB & 0b11110000;
    }

    if(state == 0b00010000){ //running state code
      analogWrite(9, 120);
    } else{
      analogWrite(9, 0);
    }

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
    state = checkStop(state);
    delay(100);
  }
  //outside of while loop, disabled state
  delay(100);
}

//ISR switches to idle state
void START() {
  if((PORTB & 0b11110000) != 0b10000000){ //ensures that start ISR cannot be used in error state
    PORTB = PORTB & 0b00001111;
    PORTB = PORTB | 0b00100000;
  }
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

int checkStop(int state) {
  int stopState = PING;
  if(PING == 0){
    PORTB = PORTB & 0b00001111;
    PORTB = PORTB | 0b01000000;
    return state = PORTB & 0b11110000;
  }
}
