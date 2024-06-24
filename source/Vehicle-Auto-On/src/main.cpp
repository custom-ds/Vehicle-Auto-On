//include <avr/io.h>
#define F_CPU 3333333UL
//include <util/delay.h>

#include <Arduino.h>


//Outputs
#define LED_STATUS PIN_PA6
#define ANALOG_ENABLE PIN_PA3

//Digital Inputs
#define SWITCH_AUTO PIN_PA4
#define SWITCH_ON PIN_PA5

//Analog Inputs
#define ANALOG_BATTERY_LEVEL PIN_PA1
#define ANALOG_REFERENCE PIN_PA2


void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_STATUS, OUTPUT);
  pinMode(ANALOG_ENABLE, OUTPUT);


  //Configure pullup resistors on the switches
  // PORTA_PIN4CTRL |= (1<<3); //pullup enable on PORTA PIN4 via PIN control register;
  // PORTA_PIN5CTRL |= (1<<3); //pullup enable on PORTA PIN5 via PIN control register;

  pinMode(SWITCH_AUTO, INPUT_PULLUP);
  pinMode(SWITCH_ON, INPUT_PULLUP);

  //initialize analog inputs
  // pinMode(ANALOG_BATTERY_LEVEL, INPUT);
  // pinMode(ANALOG_REFERENCE, INPUT);

  Serial.begin(9600);
}

void loop()
{
  //turn on the analog enable pin
  digitalWrite(ANALOG_ENABLE, HIGH);
  delay(25);    //wait for it to stabilize
  int battLevel = analogRead(ANALOG_BATTERY_LEVEL);
  int refLevel = analogRead(ANALOG_REFERENCE);
  //turn off the analog enable pin
  digitalWrite(ANALOG_ENABLE, LOW);


  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_STATUS, HIGH);
  // wait for a second
  delay(100);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_STATUS, LOW);
   // wait for a second
  delay(100);
  Serial.write(refLevel);
}



/*
int main(void) {
  PORTB_DIRSET = (1<<2); //set PORTB PIN2 to output
  PORTB_OUTSET = (1<<2); // set output HIGH;
  
  PORTA_PIN5CTRL |= (1<<3); //pullup enable on PORTA PIN5 via PIN control register;


  //https://www.instructables.com/ATTiny-Port-Manipulation-Part-2-AnalogRead/

  ADMUX |= (1 << REFS0);   //sets reference voltage to internal 1.1V 
  ADMUX |= (1 << MUX0);    //combined with next line… 
  ADMUX |= (1 << MUX1);    //sets ADC3 as analog input channel.
  ADMUX |= (1 << ADLAR);   //left adjusts for 8-bit resolution

  ADCSRA |= (1 << ADEN);   //enables the ADC



// #include <avr/io.h>   //allows for register commands to be understood
// int analogData;       //declare analogData variable

// void setup() {
// ADMUX = 0b10100011;   //sets 1.1V IRV, sets ADC3 as input channel,
// 		      //and left adjusts
// ADCSRA = 0b10000011;  //turn on ADC, keep ADC single conversion mode,
//                       //and set division factor-8 for 125kHz ADC clock
// ADCSRA = 0b10000011;  //turn on the ADC, keep ADC single conversion mode
//                       //set division factor-8 for 125kHz ADC clock
// Serial.begin(9600);   //start Serial Interface
// }

// void loop() {
// ADCSRA |= (1 << ADSC);         //start conversion
// analogData = ADCH;             //store data in analogData variable
// Serial.print("analogData: ");  //print "analogData: "
// Serial.println(analogData);    //print data in analogData variable
// delay(1000);                   //delay 1 second
// }

  while (1){

    PORTB_OUTCLR = (1<<2); // set LOW (LED OFF);
    _delay_ms(250);
    PORTB_OUTSET = (1<<2); // set HIGH (LED ON);
    _delay_ms(250);
    // if(PORTA_IN & (1<<5)){ // if input is HIGH
    //   PORTB_OUTSET = (1<<2); // set HIGH (LED ON);
    // }
    // else{ //if input is low
    //   PORTB_OUTCLR = (1<<2); // set LOW (LED OFF);
    // }
  }

}
*/