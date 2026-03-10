#include <avr/io.h>
#include <Arduino.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
//include <avr/pgmspace.h>

#define F_CPU 3333333U



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
#define DEBUG_SERIAL
#define SOFT_SERIAL_TX PIN_PB3

//Constants
#define MAX_COUNDOWN 2400   //Half-second intervals to keep the relay on after losing battery voltage sense. Set to 20 minutes (2400 * 0.5s)
//define MAX_COUNDOWN 10   //Half-second intervals to keep the relay on after losing battery voltage sense --- IGNORE ---
#define SWITCH_POSITION_DELAY 3

// How many cycles to wait between events. Each cycle is about 300mS

#define CHECK_AUTO_INTERVAL 9   //Number of loop cycles between auto-checks for charging/motion. 9 = 3.0 seconds
#define AUTO_SHUTDOWN_DELAY 1800 //Number of cycles to wait before auto-shutdown when not charging or in motion. 1800 = about 10 minutes. Set to 30 for debugging


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

enum LightState {
  LIGHT_OFF,
  LIGHT_ON,
  BLINK_SLOW,
  BLINK_FAST
};


// Function prototypes
bool isCharging();
bool isInMotion();

void setupRTC();
void goToSleep(bool lowPowerMode);

bool initLIS2HH12();
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


  // Initialize the LIS2HH12 accelerometer
  Wire.begin();
  #ifdef DEBUG_SERIAL
  customSerialPrintln("Init");
  #endif

  initLIS2HH12();
  
  #ifdef DEBUG_SERIAL
  customSerialPrintln("OK");
  #endif

  setupRTC();
}




void loop() {
  SwitchState nextPosition;

  //static vars to survive the sleep cycles
  static LightState ledMode;
  static uint16_t nextPositionDelay = 0;
  static uint16_t autoReaffirm = 0;
  static uint16_t autoKeepAliveCount = 0;
  static uint16_t ledCount = 0;

  static int autoCountdown;
  static bool lowPower = false;


  //Check the switch state
  pinMode(SWITCH_AUTO, INPUT_PULLUP);
  pinMode(SWITCH_ON, INPUT_PULLUP);  


  //Assume it's safe to go into low-power mode
  lowPower = true;


  //Determine the switch position
  if (digitalRead(SWITCH_AUTO) == HIGH && digitalRead(SWITCH_ON) == HIGH) {
    //both high, so it's in the off position
    if (nextPosition != STATE_OFF) {
      nextPosition = STATE_OFF;
      nextPositionDelay = SWITCH_POSITION_DELAY; //delay 5 cycles before changing state
    }
  } else if (digitalRead(SWITCH_ON) == LOW) {
    //Switch is in the manual on position
    if (nextPosition != STATE_ON) {
      nextPosition = STATE_ON;
      nextPositionDelay = SWITCH_POSITION_DELAY; //delay 5 cycles before changing state
    }
  } else if (digitalRead(SWITCH_AUTO) == LOW) {
    //Switch is in auto position
    if (nextPosition != STATE_AUTO) {
      nextPosition = STATE_AUTO;
      nextPositionDelay = SWITCH_POSITION_DELAY; //delay 5 cycles before changing state
    }
  }
  


  if (nextPositionDelay > 0) {
    nextPositionDelay--;
  } else {
    //it's time to make a move

    switch (nextPosition) {
    case STATE_OFF:
      // Turn off the relay and LED
      #ifdef DEBUG_SERIAL
      customSerialPrintln("0");
      #endif

      digitalWrite(RELAY, LOW);
      ledMode = LIGHT_OFF;
      ledCount = 0;
      lowPower = true;    // allow it to go into low power mode

      autoKeepAliveCount = 0;  //reset the auto keep-alive counter

      break;
    case STATE_ON:
      // Turn on the relay and LED
      #ifdef DEBUG_SERIAL
      customSerialPrintln("1");
      #endif

      digitalWrite(RELAY, HIGH);
      ledMode = LIGHT_ON;
      ledCount = 0;
      lowPower = false;   //stay awake when manually on

      autoKeepAliveCount = 0;  //reset the auto keep-alive counter

      break;
    case STATE_AUTO:
      // In auto mode, check for battery voltage, and motion if we're already switched on
      #ifdef DEBUG_SERIAL
      customSerialPrintln("A");
      #endif

      if (autoReaffirm > 0) {
        autoReaffirm--;
      } else {
        //it's time to reaffirm if the car is charging/moving
        if (isCharging()) {
          //the car battery is charging - stay awake
          autoReaffirm = CHECK_AUTO_INTERVAL;
          if (ledMode != BLINK_SLOW) {
            ledMode = BLINK_SLOW;
            ledCount = 0;
          }
          //reset the autoKeepAliveCount
          if (autoKeepAliveCount > 0) {
            //we were already on, so just extend the keep-alive
            autoKeepAliveCount = AUTO_SHUTDOWN_DELAY;
          } else {
            //we were off, so set the keep-alive to turn on the relay, but set it to a higher initial value to allow the alternator to stabilize
            autoKeepAliveCount = AUTO_SHUTDOWN_DELAY + 10;
          }

        } else {
          //maybe it's moving but the battery voltage is dropping - this happens on modern cars with smart alternators

          //but make sure that the Auto had previously been woken due to charging, before failing back to motion detection for keep-alive
          #ifdef DEBUG_SERIAL
          customSerialPrint(autoKeepAliveCount);
          customSerialPrintln("");
          #endif

          if (autoKeepAliveCount > 0) {
            #ifdef DEBUG_SERIAL
            customSerialPrintln("M");
            #endif

            if (isInMotion()) {
              //the car is moving - stay awake
              autoReaffirm = CHECK_AUTO_INTERVAL;
              if (ledMode != BLINK_FAST) {
                ledMode = BLINK_FAST;
                ledCount = 0;
              }

              //reset the autoKeepAliveCount
              autoKeepAliveCount = AUTO_SHUTDOWN_DELAY;
            } else {
              // Not charging and not in motion - start the shutdown process
              ledMode = LIGHT_OFF;
              ledCount = 0;
              autoReaffirm = 3;
            }
          }
        }
      }

      //Check the state of the relay based on the autoKeepAliveCount
      if (autoKeepAliveCount > 0) {
        autoKeepAliveCount--;   //decrement the keep-alive counter while it's positive
        // see if we're over the max countdown, which indicates that we're waiting a couple of seconds for the alternator to stabilize before energizing the relay
        if (autoKeepAliveCount <= AUTO_SHUTDOWN_DELAY) {
          //we're witin the normal operating range, so energize the relay
          digitalWrite(RELAY, HIGH);
        }
        
        lowPower = false;   //stay awake while the relay is on
      } else {
        digitalWrite(RELAY, LOW);
        lowPower = true;    //allow low power mode when the relay is off
      }

      break;
    }


    int8_t currentLEDState = digitalRead(LED_STATUS);
    //Check the LED Blink Status
    if (ledCount == 0) {
      //it's time to cycle the LED
      switch (ledMode) {
      case LIGHT_OFF:
        digitalWrite(LED_STATUS, LOW);
        ledCount = 0xFFFF; //stay off for quite a while
        break;
      case LIGHT_ON:
        digitalWrite(LED_STATUS, HIGH);
        ledCount = 0xFFFF; //stay on for quite a while
        break;
      case BLINK_SLOW:
        digitalWrite(LED_STATUS, !currentLEDState);
        ledCount = 6; //slow blink
        break;
      case BLINK_FAST:
        digitalWrite(LED_STATUS, !currentLEDState);
        ledCount = 2; //fast blink
        break;
      }

    } else {
      ledCount--;
    }

  }



  customSerialPrint(lowPower);
  customSerialPrintln("");
  goToSleep(lowPower);
}

ISR(RTC_CNT_vect) {
  RTC.INTFLAGS = RTC_OVF_bm; // clear flag
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm; // clear PIT interrupt flag
}



void setupRTC() {
  while (RTC.STATUS > 0) {}               // wait for sync
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;      // 32.768 kHz internal ULP oscillator
  RTC.PITINTCTRL = RTC_PI_bm;             // enable PIT interrupt
  //RTC.PITCTRLA   = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;        // start PIT   // About 1 second
  //RTC.PITCTRLA   = RTC_PERIOD_CYC16384_gc | RTC_PITEN_bm;        // start PIT   // About 500mS
  RTC.PITCTRLA   = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm;        // start PIT    // About 250mS
}


void goToSleep(bool lowPowerMode) {

  if (lowPowerMode) {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
  } else {
    delay(250);
  }

}


bool isCharging() {

  uint16_t battery, reference;

  //read the analog inputs
  digitalWrite(ANALOG_ENABLE, HIGH);    //enable the analog circuitry

  battery = analogRead(ANALOG_BATTERY_LEVEL);
  reference = analogRead(ANALOG_REFERENCE);

  //turn off the analog enable pin
  digitalWrite(ANALOG_ENABLE, LOW);

  //Convert to millivolts
  //battery = (float)(battery * 1);  //Show the raw value for debugging
  battery = (float)(battery * 18.32);// + 308;   //approximate battery voltage in mV 18.31
  if (digitalRead(RELAY) == HIGH) {
    //the relay is on, so the offset voltage is elevated due to voltage drop across resettable fuse
    battery += 568;
  } else {
    battery += 320;
  }
  
  reference = (reference * 2) + 12000; //approximate reference voltage in mV

  #ifdef DEBUG_SERIAL
  customSerialPrint("Bat ");
  customSerialPrint(battery);
  customSerialPrint(" / ");
  customSerialPrint(reference);
  customSerialPrintln("");
  return (battery > reference);
  #endif
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
  #ifdef DEBUG_SERIAL
  customSerialPrintln("a");
  #endif
  return true;
}

bool isInMotion() {
  //Check the LIS2HH12 accelerometer for motion

  if (!accelInitialized) {
    return false; // Accelerometer not initialized
  }

  
  // Read current acceleration values
  int16_t currentAccelX = readLIS2HH12Axis(LIS2HH12_OUT_X_L, LIS2HH12_OUT_X_H);
  int16_t currentAccelY = readLIS2HH12Axis(LIS2HH12_OUT_Y_L, LIS2HH12_OUT_Y_H);
  int16_t currentAccelZ = readLIS2HH12Axis(LIS2HH12_OUT_Z_L, LIS2HH12_OUT_Z_H);
  
  // Debug output
  #ifdef DEBUG_SERIAL
  customSerialPrint(currentAccelX);
  customSerialPrint(" ");
  customSerialPrint(currentAccelY);
  customSerialPrint(" ");
  customSerialPrint(currentAccelZ);
  customSerialPrintln("");
  #endif

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
