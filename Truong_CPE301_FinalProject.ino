#include "DHT.h" 
#include "TinyStepper.h"
#include "LiquidCrystal.h"

volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//creating objects
LiquidCrystal lcd(50, 51, 31, 30, 29, 28);
DHT dht(2, DHT11);
TinyStepper vent(4096, 25, 24, 23, 22);

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
  DDRH = DDRH & 0b11100111;
  PORTH = PORTH & 0b11100111;
  /* inputs
  bit 4: left button for vent
  bit 3: right button for vent
  */

  //setup for interrupts
  SREG = 0b10000000; //globally enables interrupts
  EICRA = 0b00001010; //triggers flag on falling edge for INT1:0
  EIMSK = 0b00000011; //enables interrupts for INT1:0

  PORTB = 0b01000000; //starting in idle

  adc_init();

  dht.begin();
  vent.Enable();
  lcd.begin(16, 2);
  Serial.begin(9600); //REMOVE
}

void loop() {
  int state = PORTB;

  if(state != 0b01000000){
    //printing data from dht onto LCD
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
    
    lcd.print("temp: ");
    lcd.print(temp);
    lcd.print(" C");
    lcd.setCursor(0, 1);
    lcd.print("humidity: ");
    lcd.print(humidity);
    lcd.print("%");

    //checks for button to move the vent
    int moveState1 = PINH;
    int moveState2 = PINH;

    if((moveState1 & 0b00010000) && 0b00010000){
      vent.Move(-15);
    }else if((moveState2 & 0b00001000) && 0b00001000){
      vent.Move(15);
    }

    analogWrite(9, 120); //generates low  base voltage for transistor to turn on fan motor

    int waterLevel = adc_read(0);
    Serial.println(waterLevel);

  }
  else{
    lcd.print("disabled"); //turns off transistor for fan motor
    analogWrite(9, 0); //turns off base voltage for transistor to turn off fan motor
  }
  delay(100);
  lcd.clear();
}

//switches to disabled state
ISR(INT0_vect){
  PORTB = 0b01000000;
}

//switches to idle state
ISR(INT1_vect){
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
