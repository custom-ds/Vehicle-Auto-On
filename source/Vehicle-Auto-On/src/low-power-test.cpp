#include <avr/io.h>
#include <Arduino.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/power.h>

//include <avr/pgmspace.h>

#define F_CPU 3333333U


// Function prototypes
void customSerialInit();
void customSerialWrite(uint8_t data);
void customSerialPrint(const char* str);
void customSerialPrintln(const char* str);
void customSerialPrint(uint16_t num);


//Outputs
#define LED_STATUS PIN_PA6
#define ANALOG_ENABLE PIN_PA3
#define RELAY PIN_PA7

//Software Serial pins
#define DEBUG_SERIAL
#define SOFT_SERIAL_TX PIN_PB3

//Constants
#define MAX_COUNDOWN 2400   //Half-second intervals to keep the relay on after losing battery voltage sense. Set to 20 minutes (2400 * 0.5s)
//define MAX_COUNDOWN 10   //Half-second intervals to keep the relay on after losing battery voltage sense --- IGNORE ---

// Define the timer frequency (1 Hz)
const uint32_t timerFrequency = 1; // 1 Hz

// Custom serial configuration
#define CUSTOM_BAUD_RATE 960    //works out to be 1200 baud. The bit-banging is very sensitive to timing errors
#define BIT_DELAY_US (1000000UL / CUSTOM_BAUD_RATE)  // Microseconds per bit

// Function prototypes
void customSerialInit();
void customSerialWrite(uint8_t data);
void customSerialPrint(const char* str);
void customSerialPrint(uint16_t num);
void customSerialPrintln(const char* str);

// ISR(WDT_vect) {
//   // WDT interrupt service routine - clears flag and wakes the MCU
//   // The interrupt flag is automatically cleared when entering the ISR
// }

ISR(RTC_CNT_vect) {
  RTC.INTFLAGS = RTC_OVF_bm; // clear flag
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm; // clear PIT interrupt flag
}



void setupRTC_PIT_halfsec() {
  while (RTC.STATUS > 0) {}               // wait for sync
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;      // 32.768 kHz internal ULP oscillator
  RTC.PITINTCTRL = RTC_PI_bm;             // enable PIT interrupt
//  RTC.PITCTRLA   = RTC_PERIOD_CYC16384_gc | RTC_PITEN_bm;        // start PIT
  RTC.PITCTRLA   = RTC_PERIOD_CYC32768_gc | RTC_PITEN_bm;        // start PIT
}

void setupWDT() {
  // Set watchdog period to ~512 ms
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_8KCLK_gc);
}

void setupRTC() {
  // Use 32.768 kHz oscillator
  customSerialPrintln("A");
  //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC32K_gc);
  //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSCULP32K_gc);
  customSerialPrintln("B");
  // Set RTC period (~500ms at 32.768kHz)
  RTC.PER = 16384; // 0.5s
  customSerialPrintln("C");
  // Enable interrupt
  RTC.INTCTRL = RTC_OVF_bm;
  customSerialPrintln("D");
  // Start RTC
  RTC.CTRLA = RTC_RTCEN_bm;
  customSerialPrintln("E");
}

void goToSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();
  sleep_cpu();
  sleep_disable();
}

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_STATUS, OUTPUT);
  pinMode(ANALOG_ENABLE, OUTPUT);
  pinMode(RELAY, OUTPUT);

  customSerialInit();  // Initialize custom serial at 1200 baud
  customSerialPrintln("Start..");

  //setupWDT();
  //setupRTC();
  setupRTC_PIT_halfsec();

  customSerialPrintln("Set up");
}

void loop()
{
  static bool ledState = false;
  digitalWrite(LED_STATUS, HIGH);
  customSerialPrintln("sleeping");
  delay(100);
  digitalWrite(LED_STATUS, LOW);

  for (int i = 0; i < 20; i++) {
    goToSleep();
  }
  customSerialPrintln("Woke up");


  digitalWrite(LED_STATUS, HIGH);
  customSerialPrintln("timing out");
  delay(5000);
  digitalWrite(LED_STATUS, LOW);
  goToSleep();
  delay(5000);
  
  
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
