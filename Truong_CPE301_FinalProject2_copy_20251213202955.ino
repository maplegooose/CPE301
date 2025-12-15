
//libraries
#include "DHT.h" 
#include "LiquidCrystal.h"
#include "TinyStepper.h"
#include "RTClib.h"

//defining address since ADC register takes up two bytes
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
#define RDA 0x80;
#define TBE 0x20  

//defining states
#define STATE_DISABLED 0b01000000
#define STATE_IDLE     0b00100000
#define STATE_RUNNING  0b00010000
#define STATE_ERROR    0b10000000

//defining a way to get the state
#define GET_STATE() (PORTB & 0b11110000)

//creating objects
DHT dht(22, DHT11);
LiquidCrystal lcd(50, 51, 5, 6, 7, 8);
TinyStepper vent(4096, 38, 39, 40, 41);
RTC_DS1307 rtc;

//declaring vars
unsigned long prevMillis = 0;
const long interval = 60000;
const float tempThreshold = 74;
const int waterThreshold = 20;
bool firstStartState = true;
const float t_clock = (6.25e-8);

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

  DDRL & 0b11101011; //pin 2: left button, pin 4: right button
  PORTL & 0b11101011;

  //setup for interrupts
  SREG = 0b10000000; //globally enables interrupts
  attachInterrupt(digitalPinToInterrupt(2), START, FALLING); //setting up START ISR

  adc_init(); //initializing analog input
  U0init(9600); //initailizing serial port

  dht.begin(); //starting the dht sensor
  lcd.begin(16, 2); //starting LCD
  rtc.begin();

  rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //readjusts the timeframe to when sketch was compiled
}

void loop() {
  int state = PORTB & 0b11110000;
  //while loop is now "active" state
  //outside of while loop, it is in disabled state
  while(GET_STATE() != 0b01000000){
    if ((firstStartState == true)){
      print("State changed to idle at ");
      printTimeStamp();
      firstStartState = false;
    }

    //grabing data from DHT
    float temp = dht.readTemperature(true);
    float humidity = dht.readHumidity();
    float water = adc_read(0);

    if (water < waterThreshold){ //if met, activates error state
      setState(STATE_ERROR);
      lcd.clear();
    } 
    
    while(GET_STATE() == STATE_ERROR){ //error state code (can't break out unless conditions met)

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.write("ERROR: LOW WATER");

      analogWrite(9, 0);

      int resetState = !(PINE & 0b00100000);
      float water = adc_read(0);

      //checks if conditions for reset to idle has been met
      if((water > waterThreshold) && (resetState)){
        setState(STATE_IDLE);
        lcd.clear();
      }

      checkTurn();

      checkStop();

    }
    
    if ((temp > tempThreshold) && (water > waterThreshold)){ //if met, activates running state
      setState(STATE_RUNNING);
    }

    if(GET_STATE() == STATE_RUNNING){ //running state code
      analogWrite(9, 120);
    } else{
      analogWrite(9, 0);
    }

    checkTurn();

    //only updates lcd every minute
    unsigned long currentMillis = millis();

    if ((currentMillis - prevMillis) >= interval){

      prevMillis = currentMillis;

      lcd.clear();
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
    checkStop();
    setTimerDelay(100);
  }
  analogWrite(9, 0);
  //outside of while loop, disabled state
  setTimerDelay(100);
}

//ISR switches to idle state
void START() {
  if((PORTB & 0b11110000) != 0b10000000){ //ensures that start ISR cannot be used in error state
    PORTB = (PORTB & 0b00001111) | 0b00100000;
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

void checkStop() {
  int stopState = PING;
  if(stopState == 0){
    setState(STATE_DISABLED);
  }
}

void checkTurn() {
    int leftState = PINL & 0b00000100;
    int rightState = PINL & 0b00010000;
    if (leftState == 0){
      vent.Move(-15);
      print("Vent is moving left at ");
      printTimeStamp();
    }
    if (rightState == 0){
      vent.Move(15);
      print("Vent is moving right at ");
      printTimeStamp();
    }
}

void U0init(unsigned long U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  UCSR0A = 0x20;
  UCSR0B = 0x18;
  UCSR0C = 0x06;
  UBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  return UCSR0A & RDA;
}

unsigned char U0getchar()
{
  return UDR0;
}

void U0putchar(unsigned char U0pdata)
{
  while((UCSR0A & TBE)==0);
  UDR0 = U0pdata;
}

//making my own function for printing to serial monitor
void print(char msg []){
  int i = 0;
  while(msg[i] != '\0'){
    U0putchar(msg[i]);
    i++;
  }
}

void printTwoDigits(int num){
  if (num < 10){
    U0putchar('0');
    U0putchar('0' + num);
  } else if (num >= 10){
    U0putchar('0' + (num / 10));
    U0putchar('0' + (num % 10));
  }
}

void printFourDigits(int num){
  U0putchar('0' + (num / 1000));
  U0putchar('0' + ((num / 100) % 10));
  U0putchar('0' + ((num / 10) % 10));
  U0putchar('0' + (num % 10));
}

//printing timestamp using rtc module
void printTimeStamp(){
  DateTime time = rtc.now();
  int month = time.month();
  int year = time.year();
  int day = time.day();
  int hour = time.hour();
  int minute = time.minute();
  int second = time.second();

  printTwoDigits(month);
  print("/");
  printTwoDigits(day);
  print("/");
  printFourDigits(year);
  print(" ");
  printTwoDigits(hour);
  print(":");
  printTwoDigits(minute);
  print(":");
  printTwoDigits(second);
  print("\n");
}

//clean function to set states and print
void setState(int newState) {
  int currentState = GET_STATE();

  if (currentState != newState) {
    PORTB = (PORTB & 0b00001111) | newState;

    if (newState == STATE_RUNNING) {
      print("State changed to running at ");
      printTimeStamp();
    }
    else if (newState == STATE_IDLE) {
      print("State changed to idle at ");
      printTimeStamp();
      firstStartState = true;
    }
    else if (newState == STATE_DISABLED) {
      print("State changed to disabled at ");
      printTimeStamp();
      firstStartState = true;
    }
    else if (newState == STATE_ERROR) {
      print("State changed to error at ");
      printTimeStamp();
      firstStartState = true;
    }
  }
}

void setTimerDelay(int ms){

  unsigned int ticks = (ms/1000)/t_clock;

  TCCR1B &= 0x00;
  TCNT1 = 65536 - ticks;

  TCCR1A = 0x00;
  TCCR1B |= 0b00000010;
  while((TIFR1 & 0x01)==0);
  TCCR1B &= 0x00;
  TIFR1 |= 0b00000001;

}
