#include <avr/io.h>
//define F_CPU 3333333UL
#define F_CPU 3333333UL  // Correct frequency for ATtiny404 internal oscillator
//include <util/delay.h>

#include <Arduino.h>
#include <Wire.h>
// #include <SoftwareSerial.h>  // Remove SoftwareSerial - we'll implement our own


//Outputs
#define LED_STATUS PIN_PA6
#define ANALOG_ENABLE PIN_PA3
#define RELAY PIN_PA7


//Digital Inputs
#define SWITCH_AUTO PIN_PA4
#define SWITCH_ON PIN_PA5

//Analog Inputs
#define ANALOG_BATTERY_LEVEL PIN_PA1
#define ANALOG_REFERENCE PIN_PA2

//Software Serial pins
#define SOFT_SERIAL_RX PIN_PB2
#define SOFT_SERIAL_TX PIN_PB3

//Constants
#define MAX_COUNDOWN 10

// LIS2HH12 accelerometer constants
#define LIS2HH12_I2C_ADDRESS 0x1E  // Default I2C address (can also be 0x1D depending on SA0 pin)
#define LIS2HH12_WHO_AM_I 0x0F
#define LIS2HH12_CTRL1 0x20
#define LIS2HH12_CTRL2 0x21
#define LIS2HH12_CTRL3 0x22
#define LIS2HH12_OUT_X_L 0x28
#define LIS2HH12_OUT_X_H 0x29
#define LIS2HH12_OUT_Y_L 0x2A
#define LIS2HH12_OUT_Y_H 0x2B
#define LIS2HH12_OUT_Z_L 0x2C
#define LIS2HH12_OUT_Z_H 0x2D

// Motion detection threshold (adjust based on sensitivity needs)
#define MOTION_THRESHOLD 100  // Raw acceleration difference threshold
#define MOTION_SAMPLES 3      // Number of samples to compare for motion detection


// Define the timer frequency (1 Hz)
const uint32_t timerFrequency = 1; // 1 Hz

// Global variables for motion detection
int16_t prevAccelX = 0, prevAccelY = 0, prevAccelZ = 0;
bool accelInitialized = false;

// Custom serial configuration
#define CUSTOM_BAUD_RATE 960    //works out to be 1200 baud. The bit-banging is very sensitive to timing errors
#define BIT_DELAY_US (1000000UL / CUSTOM_BAUD_RATE)  // Microseconds per bit

// Function prototypes
bool initLIS2HH12();
bool checkForMotion();
void writeLIS2HH12Register(uint8_t reg, uint8_t value);
uint8_t readLIS2HH12Register(uint8_t reg);
void customSerialInit();
void customSerialWrite(uint8_t data);
void customSerialPrint(const char* str);
void customSerialPrintln(const char* str);
int16_t readLIS2HH12Axis(uint8_t regL, uint8_t regH);


void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_STATUS, OUTPUT);
  pinMode(ANALOG_ENABLE, OUTPUT);
  pinMode(RELAY, OUTPUT);



  //Configure pullup resistors on the switches
  // PORTA_PIN4CTRL |= (1<<3); //pullup enable on PORTA PIN4 via PIN control register;
  // PORTA_PIN5CTRL |= (1<<3); //pullup enable on PORTA PIN5 via PIN control register;

  pinMode(SWITCH_AUTO, INPUT_PULLUP);
  pinMode(SWITCH_ON, INPUT_PULLUP);

  //initialize analog inputs
  // pinMode(ANALOG_BATTERY_LEVEL, INPUT);
  // pinMode(ANALOG_REFERENCE, INPUT);



  //Configure the timer interrupt
  // Set up Timer/Counter 0 (TC0) for CTC mode (Clear Timer on Compare Match)
  //TCA0.SINGLE.CTRLA = 0; // Set all bits to 0
  //TCA0.SINGLE.CTRLB = 0; // Set all bits to 0
  //TCA0.SINGLE.CNT = 0; // Initialize counter value to 0

  // // Set the prescaler to 64 (prescaler = 64)
  // TCA0.SINGLE.CTRLA |= (1 << TCA_SINGLE_CLKSEL_DIV64_gc);

  // // Calculate the compare match value for 1 Hz frequency
  // uint32_t compareMatchValue = F_CPU / (64UL * timerFrequency) - 1;

  // // Set the compare match value
  // TCA0.SINGLE.CMP0 = compareMatchValue;

  // // Enable the compare match interrupt
  // TCA0.SINGLE.INTCTRL |= (1 << TCA_SINGLE_CMP0EN_bp);

  // // Enable global interrupts
  // sei();

  // Initialize custom bit-banged serial on PB2 (TX) and PB3 (RX)
  // Use a lower baud rate for better reliability with ATtiny404 at 3.33MHz
  customSerialInit();  // Initialize custom serial at 1200 baud
  
  // Also keep hardware serial if needed (optional)
  //Serial.begin(19200);

  customSerialPrintln("Starting up...");
  //Serial.println("Starting up...");
    // Initialize I2C communication
  Wire.begin();
  
  customSerialPrintln("Initializing LIS2HH12...");
  //Serial.println("Initializing LIS2HH12...");
  // Initialize the LIS2HH12 accelerometer
  initLIS2HH12();
  customSerialPrintln("Init complete.");
}

void loop()
{
  int switchAuto, switchOn;
  int battLevel, refLevel;

  int autoCountdown;

  bool ledState = false;

  //Check the switch state
  switchAuto = digitalRead(SWITCH_AUTO);
  switchOn = digitalRead(SWITCH_ON);

  while (1)
  {
    digitalWrite(LED_STATUS, HIGH);
    delay(500);
    digitalWrite(LED_STATUS, LOW);

    customSerialPrintln("Looping");
    //Check for motion
    if (checkForMotion()) {
      customSerialPrintln("Motion");
      digitalWrite(RELAY, HIGH);
      digitalWrite(LED_STATUS, HIGH);
      ledState = true;
      
      
      delay(2000); // Keep relay on for 2 seconds after motion detected
      digitalWrite(RELAY, LOW);
      digitalWrite(LED_STATUS, LOW);      
    }
  }
  

  if (switchAuto == LOW)
  {
    //Auto mode


    //Read in the analog values
    digitalWrite(ANALOG_ENABLE, HIGH);    //enable the analog circuitry
    delay(25);    //wait for it to stabilize
    battLevel = analogRead(ANALOG_BATTERY_LEVEL);
    refLevel = analogRead(ANALOG_REFERENCE);
    //turn off the analog enable pin
    digitalWrite(ANALOG_ENABLE, LOW);

    if (battLevel > refLevel)
    {
      //We have positive voltage on the input sense - reset the autoCountdown
      digitalWrite(RELAY, HIGH);    //turn on the relay
      digitalWrite(LED_STATUS, HIGH);
      ledState = true;
      autoCountdown = 10;   //reset the timer to keep the relay on even after the battery voltage drops
    }
    else
    {
      //We have negative voltage on the input sense - decrement the autoCountdown
      if (true)//(autoCountdown > 0) 
      {
        autoCountdown--;
        //Blink the LED
        digitalWrite(LED_STATUS, ledState);
        ledState = !ledState;
        
      }
      else
      {
        //Turn off the relay
        digitalWrite(RELAY, LOW);
        digitalWrite(LED_STATUS, LOW);
        ledState = false;
        //autoCountdown = 0;
      }
    }    

  }
  else if (switchOn == LOW)
  {
    //Manual mode
    //Turn on the relay
    digitalWrite(RELAY, HIGH);

    digitalWrite(LED_STATUS, HIGH);
    ledState = true;
  }
  else
  {
    //Turn off the relay
    digitalWrite(RELAY, LOW);
    digitalWrite(LED_STATUS, LOW);
    ledState = false;
  }


  delay(1000);
  //softSerial.write(refLevel);
}


// ISR(TCA0_CMP0_vect) {
//     // This code will be executed every second
//     // You can add your desired actions here
//     // For example, toggle an LED, update a counter, etc.
//   static bool ledState = false;

//   // Toggle the LED state
//   digitalWrite(LED_STATUS, ledState);
//   ledState = !ledState;
  

// }

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

// Helper function to write to LIS2HH12 register
void writeLIS2HH12Register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LIS2HH12_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Helper function to read from LIS2HH12 register
uint8_t readLIS2HH12Register(uint8_t reg) {
  Wire.beginTransmission(LIS2HH12_I2C_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(LIS2HH12_I2C_ADDRESS, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// Helper function to read 16-bit accelerometer data
int16_t readLIS2HH12Axis(uint8_t regL, uint8_t regH) {
  uint8_t low = readLIS2HH12Register(regL);
  uint8_t high = readLIS2HH12Register(regH);
  return (int16_t)((high << 8) | low);
}

// Initialize the LIS2HH12 accelerometer
bool initLIS2HH12() {
  // Check WHO_AM_I register
  uint8_t whoAmI = readLIS2HH12Register(LIS2HH12_WHO_AM_I);
  if (whoAmI != 0x41) {  // Expected WHO_AM_I value for LIS2HH12
    customSerialPrintln("LIS2HH12 not found!");
    return false;
  }
  
  // Configure CTRL1 register: Enable X, Y, Z axes, set data rate to 10Hz
  writeLIS2HH12Register(LIS2HH12_CTRL1, 0x17); // 0001 0111: 10Hz, XYZ enabled
  
  // Configure CTRL2 register: +/- 2g full scale
  writeLIS2HH12Register(LIS2HH12_CTRL2, 0x00); // 0000 0000: +/- 2g
  
  // Configure CTRL3 register: Default settings
  writeLIS2HH12Register(LIS2HH12_CTRL3, 0x00);
  
  delay(100); // Allow time for configuration
  
  // Read initial values
  prevAccelX = readLIS2HH12Axis(LIS2HH12_OUT_X_L, LIS2HH12_OUT_X_H);
  prevAccelY = readLIS2HH12Axis(LIS2HH12_OUT_Y_L, LIS2HH12_OUT_Y_H);
  prevAccelZ = readLIS2HH12Axis(LIS2HH12_OUT_Z_L, LIS2HH12_OUT_Z_H);
  
  accelInitialized = true;
  customSerialPrintln("LIS2HH12 initialized successfully");
  return true;
}

bool checkForMotion() {
  //Check the LIS2HH12 accelerometer for motion
  
  if (!accelInitialized) {
    return false; // Accelerometer not initialized
  }
  
  // Read current acceleration values
  int16_t currentAccelX = readLIS2HH12Axis(LIS2HH12_OUT_X_L, LIS2HH12_OUT_X_H);
  int16_t currentAccelY = readLIS2HH12Axis(LIS2HH12_OUT_Y_L, LIS2HH12_OUT_Y_H);
  int16_t currentAccelZ = readLIS2HH12Axis(LIS2HH12_OUT_Z_L, LIS2HH12_OUT_Z_H);
  
  // Calculate the difference from previous readings
  int16_t deltaX = abs(currentAccelX - prevAccelX);
  int16_t deltaY = abs(currentAccelY - prevAccelY);
  int16_t deltaZ = abs(currentAccelZ - prevAccelZ);
  
  // Check if any axis exceeds the motion threshold
  bool motionDetected = (deltaX > MOTION_THRESHOLD) || 
                       (deltaY > MOTION_THRESHOLD) || 
                       (deltaZ > MOTION_THRESHOLD);
  
  // Update previous values for next comparison
  prevAccelX = currentAccelX;
  prevAccelY = currentAccelY;
  prevAccelZ = currentAccelZ;
  
  // Optional: Print debug information
  if (motionDetected) {

    
    customSerialPrint("Motion detected! Delta X: ");
    customSerialPrint(String(deltaX).c_str());
    customSerialPrint(", Y: ");
    customSerialPrint(String(deltaY).c_str());
    customSerialPrint(", Z: ");
    customSerialPrintln(String(deltaZ).c_str());
  }
  
  return motionDetected;
}

// Custom bit-banged serial implementation
void customSerialInit() {
  pinMode(SOFT_SERIAL_TX, OUTPUT);
  digitalWrite(SOFT_SERIAL_TX, HIGH);  // Idle state is HIGH
  delayMicroseconds(BIT_DELAY_US * 2); // Let line settle
}

void customSerialWrite(uint8_t data) {
  // Start bit (LOW)
  digitalWrite(SOFT_SERIAL_TX, LOW);
  delayMicroseconds(BIT_DELAY_US);
  
  // Data bits (LSB first)
  for (int i = 0; i < 8; i++) {
    digitalWrite(SOFT_SERIAL_TX, (data >> i) & 0x01);
    delayMicroseconds(BIT_DELAY_US);
  }
  
  // Stop bit (HIGH)
  digitalWrite(SOFT_SERIAL_TX, HIGH);
  delayMicroseconds(BIT_DELAY_US);
}

void customSerialPrint(const char* str) {
  while (*str) {
    customSerialWrite(*str++);
  }
}

void customSerialPrintln(const char* str) {
  customSerialPrint(str);
  customSerialWrite('\r');
  customSerialWrite('\n');
}