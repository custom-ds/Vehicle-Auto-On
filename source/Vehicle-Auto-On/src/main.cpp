#include <avr/io.h>
#include <Arduino.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
//include <avr/pgmspace.h>

#define F_CPU 3333333U


// Function prototypes
bool initLIS2HH12();
bool checkForMotion();
void writeLIS2HH12Register(uint8_t reg, uint8_t value);
uint8_t readLIS2HH12Register(uint8_t reg);

void customSerialInit();
void customSerialWrite(uint8_t data);
void customSerialPrint(const char* str);
void customSerialPrintln(const char* str);
void customSerialPrint(uint16_t num);


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
//define DEBUG_SERIAL
#define SOFT_SERIAL_TX PIN_PB3

//Constants
#define MAX_COUNDOWN 2400   //Half-second intervals to keep the relay on after losing battery voltage sense. Set to 20 minutes (2400 * 0.5s)
//define MAX_COUNDOWN 10   //Half-second intervals to keep the relay on after losing battery voltage sense --- IGNORE ---

// LIS2HH12 accelerometer constants
#define LIS2HH12_I2C_ADDRESS 0x1D  // Default I2C address (can also be 0x1D depending on SA0 pin) (0x1E is default, 0x1D if SA0 is grounded)
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

//Enum switch state
enum SwitchState {
  STATE_OFF,
  STATE_ON,
  STATE_AUTO
};


// Function prototypes
bool initLIS2HH12();
bool checkForMotion();
void writeLIS2HH12Register(uint8_t reg, uint8_t value);
uint8_t readLIS2HH12Register(uint8_t reg);
int16_t readLIS2HH12Axis(uint8_t regL, uint8_t regH);

void customSerialInit();
void customSerialWrite(uint8_t data);
void customSerialPrint(const char* str);
void customSerialPrint(uint16_t num);
void customSerialPrintln(const char* str);




ISR(WDT_vect) {
  // WDT interrupt service routine - does nothing, just wakes the MCU
}


void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_STATUS, OUTPUT);
  pinMode(ANALOG_ENABLE, OUTPUT);
  pinMode(RELAY, OUTPUT);



  //Configure pullup resistors on the switches
  pinMode(SWITCH_AUTO, INPUT_PULLUP);
  pinMode(SWITCH_ON, INPUT_PULLUP);

  //initialize analog inputs
  pinMode(ANALOG_BATTERY_LEVEL, INPUT);
  pinMode(ANALOG_REFERENCE, INPUT);





  // Initialize custom bit-banged serial on PB2 (TX) and PB3 (RX)
  // Use a lower baud rate for better reliability with ATtiny404 at 3.33MHz
  customSerialInit();  // Initialize custom serial at 1200 baud


  #ifdef DEBUG_SERIAL
  customSerialPrintln("Start..");
  #endif

  // Initialize the LIS2HH12 accelerometer
  Wire.begin();
  #ifdef DEBUG_SERIAL
  customSerialPrintln("Init");
  #endif

  initLIS2HH12();
  
  #ifdef DEBUG_SERIAL
  customSerialPrintln("done");
  #endif

  //Set up WDT for ATtiny404
  // cli(); // Disable interrupts during WDT setup
  // Configure WDT in interrupt mode (not reset mode)
  // CCP = CCP_IOREG_gc; // Enable protected register write
  // WDT.CTRLA = WDT_PERIOD_512CLK_gc; // ~512ms timeout, interrupt mode
  //sei(); // Re-enable interrupts

  set_sleep_mode(SLEEP_MODE_STANDBY); // Use STANDBY for WDT wake-up
}

void loop()
{
  SwitchState switchPosition;
  uint16_t battLevel, refLevel;

  static int autoCountdown;
  static bool ledState = false;
  static bool relayState = false;


  //Check the switch state
  pinMode(SWITCH_AUTO, INPUT_PULLUP);
  pinMode(SWITCH_ON, INPUT_PULLUP);  


  //Determine the switch position
  if (digitalRead(SWITCH_AUTO) == HIGH && digitalRead(SWITCH_ON) == HIGH) {
    switchPosition = STATE_OFF;
  } else if (digitalRead(SWITCH_ON) == LOW) {
    //Switch is in the manual on position
    switchPosition = STATE_ON;
  } else if (digitalRead(SWITCH_AUTO) == LOW) {
    //Switch is in auto position
    switchPosition = STATE_AUTO;
  }
  
  
  



  
  switch (switchPosition) {
    case STATE_OFF:
      // Turn off the relay and LED
      #ifdef DEBUG_SERIAL
      customSerialPrintln("OF");
      #endif

      digitalWrite(RELAY, LOW);
      relayState = false;
      digitalWrite(LED_STATUS, LOW);
      ledState = false;

      checkForMotion();  //just check for motion to keep the accelerometer active

      break;

    case STATE_ON:
      // Turn on the relay and LED
      #ifdef DEBUG_SERIAL
      customSerialPrintln("ON");
      #endif

      digitalWrite(RELAY, HIGH);
      relayState = true;
      digitalWrite(LED_STATUS, HIGH);
      ledState = true;
      break;

    case STATE_AUTO:
      // In auto mode, check for battery voltage, and motion if we're already switched on
      #ifdef DEBUG_SERIAL
      customSerialPrintln("AU");
      //customSerialPrintln("Read Analog");
      #endif


      digitalWrite(ANALOG_ENABLE, HIGH);    //enable the analog circuitry
      delay(25);    //wait for it to stabilize
      battLevel = analogRead(ANALOG_BATTERY_LEVEL);
      refLevel = analogRead(ANALOG_REFERENCE);
      //turn off the analog enable pin
      digitalWrite(ANALOG_ENABLE, LOW);     


      #ifdef DEBUG_SERIAL
      customSerialPrint("Bat ");
      customSerialPrint(battLevel);
      customSerialPrintln("");
      customSerialPrint("Ref  ");
      customSerialPrint(refLevel);
      customSerialPrintln("");
      #endif

      //Desensitize the reference level a bit to avoid noise
      refLevel = refLevel / 4;  //divide by 4 to reduce noise sensitivity
      refLevel = refLevel + 526; //311 is about 11.5V on the battery sense line

      #ifdef DEBUG_SERIAL
      customSerialPrint("CRef ");
      customSerialPrint(refLevel);
      customSerialPrintln("");
      #endif


      bool motion = checkForMotion();
      if (motion) {
        //Motion detected - keep the relay on
        #ifdef DEBUG_SERIAL
        customSerialPrintln("*");
        #endif

      } else {
        #ifdef DEBUG_SERIAL
        customSerialPrintln("-");
        #endif
      }


      //check the input voltage to see if the alternator is charging the battery
      if (battLevel > refLevel)
      {
        //We have positive voltage on the input sense - reset the autoCountdown
        #ifdef DEBUG_SERIAL
        customSerialPrintln("Chrg");
        #endif

        digitalWrite(RELAY, HIGH);    //turn on the relay
        relayState = true;
        digitalWrite(LED_STATUS, HIGH);
        ledState = true;
        autoCountdown = MAX_COUNDOWN;   //reset the timer to keep the relay on even after the battery voltage drops
      }
      else
      {
        //We have negative voltage on the input sense - decrement the autoCountdown
        if (autoCountdown > 0) 
        {
          #ifdef DEBUG_SERIAL
          customSerialPrintln("Coast");
          #endif
          
          autoCountdown--;
          //Blink the LED if we were previously on, but now in coast because the voltage has dropped
          ledState = !ledState;
          digitalWrite(LED_STATUS, ledState);
        }
        else
        {
          #ifdef DEBUG_SERIAL
          customSerialPrintln(">0");
          #endif

          //Turn off the relay
          digitalWrite(RELAY, LOW);
          relayState = false;
          digitalWrite(LED_STATUS, LOW);
          ledState = false;
        }
      }   
      break;
  }

  #ifdef DEBUG_SERIAL
  customSerialPrintln("");
  customSerialPrintln("");
  #endif




  delay(500);

  // __asm__ __volatile__ ("wdr");   //similar to wdt_reset() but doesn't mess with the WDT config causing a full reset at overflow
  // if (ledState || relayState) {
  //   //we have something turned on. just delay for 500ms
  //   delay(500);
  // } else {
  //   //Sleep for about 500ms in low power mode
  //   // Prepare for WDT wake-up sleep
  //   sleep_enable();
  //   sei(); // Ensure interrupts are enabled for WDT wake-up
  //   sleep_cpu(); // Sleep until WDT interrupt
  //   sleep_disable();
  //   cli(); // Disable interrupts after waking up - no need for these until we go back to sleep again
  // }
}




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
  customSerialPrint(whoAmI);

  if (whoAmI != 0x41) {  // Expected WHO_AM_I value for LIS2HH12
    #ifndef DEBUG_SERIAL
    customSerialPrintln("miss");
    #endif

    digitalWrite(RELAY, HIGH); //debug
    delay(250);
    digitalWrite(RELAY, LOW);  //debug
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
  digitalWrite(RELAY, HIGH); //debug
  delay(250);
  digitalWrite(RELAY, LOW);  //debug
  delay(250);
  digitalWrite(RELAY, HIGH); //debug
  delay(250);
  digitalWrite(RELAY, LOW);  //debug
  delay(250);  
  digitalWrite(RELAY, HIGH); //debug
  delay(250);
  digitalWrite(RELAY, LOW);  //debug
  delay(250);

  #ifdef DEBUG_SERIAL
  customSerialPrintln("...");
  #endif
  return true;
}

bool checkForMotion() {
  //Check the LIS2HH12 accelerometer for motion
  digitalWrite(LED_STATUS, HIGH); // Indicate we're checking
  delay(500);
  digitalWrite(LED_STATUS, LOW);
  delay(500);

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
  bool motionDetected = (deltaX > MOTION_THRESHOLD) || (deltaY > MOTION_THRESHOLD) || (deltaZ > MOTION_THRESHOLD);
  
  // Update previous values for next comparison
  prevAccelX = currentAccelX;
  prevAccelY = currentAccelY;
  prevAccelZ = currentAccelZ;
  
  // Optional: Print debug information
  if (motionDetected) {
    digitalWrite(LED_STATUS, HIGH); // Indicate motion detected
    delay(4000);
    digitalWrite(LED_STATUS, LOW);
    delay(500);
    #ifdef DEBUG_SERIAL
    customSerialPrint("X: ");
    customSerialPrint(String(deltaX).c_str());
    customSerialPrint(", Y: ");
    customSerialPrint(String(deltaY).c_str());
    customSerialPrint(", Z: ");
    customSerialPrintln(String(deltaZ).c_str());
    #endif
  }
  
  digitalWrite(LED_STATUS, HIGH); // Indicate we're checking
  delay(150);
  digitalWrite(LED_STATUS, LOW);
  delay(2000);

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

void customSerialPrint(uint16_t num) {
  char buffer[6]; // Enough for 5 digits + null terminator
  itoa(num, buffer, 10);
  customSerialPrint(buffer);
}

void customSerialPrintln(const char* str) {
  customSerialPrint(str);
  customSerialWrite('\r');
  customSerialWrite('\n');
}
