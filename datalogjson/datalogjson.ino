#define md5HASH "d0da699d81d0c26184014d62b8bffbca"
#define md5TIME "2023-01-01-12-29-42"

// md5HASH used to know the version that is the basis for the running code.
// In many cases the defines down below, and sometimes some code, will be modified after checking out this file,
// so the hash of this file will differ from what is in git, but md5HASH will allow linking back to the original.
// At startup, the the md5HASH will be output on the LCD and the serial data feed.
// The values of the defines below will be output in the serial data feed.
// And there is a code change message in the defines below that can be set if the code was changed after checkout.
// With this structure in place, it should be possible to recreate a running system by looking at the startup data.

// Script to read A & D pins, timestamp the data, and send out the serial port in a json packet
// A serial logger (OpenLog) can be used to record the data, or a RPi can be used as a gateway to the internet
// This approach was taken because the system will be used with batter/solar power.
// The RPi can be shut down and just the core data logging can run when power needs to be conserved.
// A LCD shows the last 2 readings and the high and low for today and yesterday. A 20X4 LCD is needed to show that much data.
// Pins used: D0, D1 for serial port, D5, D6 for sample speed, I2C port (depends on Arduino flavor)
//            D7 push button as used in https://www.arduino.cc/en/tutorial/pushbutton, used to cut short long sleep times
//            LED_BUILTIN - on when processing data (not sleeping)
//            D8 default for raising, D9 default for reducing - setpoint control and/or duty cycle monitoring. Can be changed in defines below.
// It is possible the code can be used unmodified in many applications.
// But many applications will require some modification. Even so, having sample code to talk to all the components should speed things up.
// Requires libraries to drive the LCD, real time clock, etc.
// Try to complile and missing libraries will cause errors. Look at #include lines to see libraries used.

// It is a mix of code I wrote and example code I found
// Mashup by Walter Spurgiasz 2020
// Code inspired from:
// https://lastminuteengineers.com/ds3231-rtc-arduino-tutorial/
// https://www.makerguides.com/character-i2c-lcd-arduino-tutorial/
// Tested on Uno, Mega2560, Leonardo, Due, Adafruit Metro M4 Express AirLift (WiFi) - Lite, chipKIT uC32
// Adafruit Metro M4 Express AirLift (WiFi) - Lite
// https://learn.adafruit.com/adafruit-metro-m4-express-featuring-atsamd51/setup
// File->Preferences->Additional Boards Manager URLS (comma seperated) https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
// Tools->Board->Board Manager  Scroll down to the Adafruit and install
// chipKIT uC32 https://blog.digilentinc.com/how-to-program-your-chipkit-board-in-the-arduino-ide/
// File->Preferences->Additional Boards Manager URLS (comma seperated) https://github.com/chipKIT32/chipKIT-core/raw/master/package_chipkit_index.json
// Tools->Board->Board Manager  Scroll down to the chipKIT and install
// On 64b Ubuntu - sudo apt-get install libc6-i386 - More details at http://chipkit.net/wiki/index.php?title=ChipKIT_core -> 64-Bit Linux

// !! please read all the comments below if you plan on using the sepoint feature !!
// !! if doing refrigeration control, there can be many issues to be aware of !!
// !! SETPOINT_RESTART_DELAY and HYSTERESIS are trying to address the short cycle issue !!
// !! this page mentions how a delay can solve the short cycle issue http://www.refrigerationbasics.com/RBIII/controls5.htm !!
//#define SETPOINT_HIGH_LIMIT 68 // trips at this value + hysteresis, comment out to turn off reducing, will still monitor duty cycle, if commented out, coment out alt setpoint
#define SETPOINT_HIGH_HYSTERESIS 3 // hysteresis setup to go past setpoint (away from other setpoint) to reduce ringing when hysteresis and setpoint gap is small
#define SETPOINT_DC_HIGH_PIN 3 // A or D pin for reducing. Output if using setpoint control, or input if monitoring duty cycle.
#define DUTY_CYCLE_HIGH_SIGNAL 'A' // D = digital output if setpoint control or input if monitoring. A = analog input if monitoring.
#define DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD 125 // D input: 1 active high, 0 active low. A input: + or - #, 200 = active if above 200, -200 = active if below 200
#define DUTY_CYCLE_HIGH_DISABLE 0 // 1 = do not calculate or display duty cycle
//#define SETPOINT_LOW_LIMIT 67 // comment out to turn off raising, trips at this value - hysteresis
#define SETPOINT_LOW_HYSTERESIS 1
#define SETPOINT_DC_LOW_PIN 8 // A or D pin for raising. Output if using setpoint control, or input if monitoring duty cycle.
#define DUTY_CYCLE_LOW_SIGNAL 'D' // D = digital output if setpoint control or input if monitoring. A = analog input if monitoring.
#define DUTY_CYCLE_LOW_SIGNAL_THRESHOLD 0 // D input: 1 active high, 0 active low. A input: + or - #, 200 = active if above 200, -200 = active if below 200
#define DUTY_CYCLE_LOW_DISABLE 1 // 1 = do not calculate or display duty cycle
#define SETPOINT_RESTART_DELAY 5 // cycles to wait until turning on raising/reducing after last phase ended, confirm cycle time is correct to prevent short cycling
#define DUTY_CYCLE_FRAME_SAMPLES 200 // number of samples in a duty cycle calculation time frame, roll to next frame when full
#define DUTY_CYCLE_FRAME_ROLLOVER .2 // the amount of the next frame to seed with the current duty cycle
// alt setpoints, common use is setpoints to hold temp at about 70F for fermentation, alt setpoint to hold temp at 40F or below for refrigeration
#define SETPOINT_HIGH_LIMIT_ALT 36 // do not define alt setpoints if regular setpoints not set, trips at this value + hysteresis
#define SETPOINT_LOW_LIMIT_ALT 1 // trips at this value - hysteresis
#define SETPOINT_ALT_PIN 10 // D pin for switching to alt setpoints
#define VOLTAGE_DISPLAY_PIN 4 // A pin to display voltage if space not used by duty cycle. Main use is to show the voltage of the back up battery.
//#define CODE_CHANGE_MESSAGE '' // if code is changed, enter a change message to track what was changed
// !!! if adding a param here, also add to jsonConfig !!!

// pre declare and reused ram to keep usage down
float atemp[3]; // used for scaling TMP36 temperature sensor on A inputs, each float uses 4 bytes
char thour = 99; // save temp time
char tmin = 99;
char tsec = 99;
char tday = 99; // save day so know when to reset high and low
float thtemp = -999; // today high temperature set so any normal reading above
float tltemp = 999; // today low temperature set so any normal reading below
float yhtemp = -999; // yesterday high
float yltemp = 999; // yesterday low
float voltage_display = -1; // used for scaling voltage, -1 = not set
float temp_float; // temporary float
int temp_int; // temporary int
unsigned int delay_sec = 0; // to track the delay between samples with out RTC
uint32_t lastTimestamp = 0;
float elapsedTimestamp = 0; // float because divided by 36
unsigned char duty_cycle_reducing_count = 0; // number of samples signal was active
unsigned char duty_cycle_reducing = 0; // calculated duty cycle
unsigned char duty_cycle_raising_count = 0; // samples in active state
unsigned char duty_cycle_raising = 0;
unsigned char duty_cycle_total_count = 0; // total samples in a time frame, ratio with active count gets duty cycle
// seed with SETPOINT_RESTART_DELAY so restarted do not skip delay, set to SETPOINT_RESTART_DELAY at turn off, ticks down to 0 to reenable setpoint control
unsigned char setpoint_restart_delay = SETPOINT_RESTART_DELAY + 1; // +1 to make up for the --count that happens before the first setpoint check
byte bitwise_status = 0b01000000;
#define BWS_NO_RTC 0b10000000
#define BWS_NO_DATA 6 // suppress prev data until a read is made

#include "Wire.h"
#define SERIALP Serial // different Arduino have variuos serial ports, so control which one we use

// from http://forum.arduino.cc/index.php?topic=113656.0
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  // Code in here will only be compiled if an Arduino Uno (or older) is used.
  #define arduinoVariant "Uno"
  #define analogPins 4
  #define digitalPins 14
  #define ioref 5.0f // operating voltage on an Uno, float becaused coud be 3.3 and used to scale A/D converter
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
  // Code in here will only be compiled if an Arduino Leonardo is used.
  #define arduinoVariant "Leonardo"
  #define analogPins 6  // A0-A5, A6-A11 (on digital pins 4, 6, 8, 9, 10, & 12, will need to convert those to analog to go above 6)
  #define digitalPins 14  // On the Leonardo, I2C is on digital pins 2 and 3 so don't use those if you need I2C
  #define ioref 5.0f // operating voltage on an Leonardo, float becaused coud be 3.3 and used to scale A/D converter
  // Comment out the next 3 lines to output to the USB serial port (watch on Arduino Serial Monitor)
  #define USE_SERIAL1 1
  #undef SERIALP
  #define SERIALP Serial1
#endif
  
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // Code in here will only be compiled if an Arduino Mega is used.
  #define arduinoVariant "Mega"
  #define analogPins 8 // Mega has 16 but GenuLog shield supports 8
  #define digitalPins 14 // Mega has 54 but GenuLog shield supports 14
  #define ioref 5.0f
#endif

#if defined(__SAM3X8E__)
  // Code in here will only be compiled if an Arduino Due is used.
  #define arduinoVariant "Due"
  #define analogPins 8 // Due has 16 but GenuLog shield supports 8
  #define digitalPins 14 // Due has 54 but GenuLog shield supports 14
  #define ioref 3.3f
  #define Wire Wire1 // this is needed on Due
#endif

#if defined(_VARIANT_METRO_M4_WIFI_)
  #define arduinoVariant "Metro_M4_WiFi"
  #define analogPins NUM_ANALOG_INPUTS
  #define digitalPins NUM_DIGITAL_PINS
  #define ioref 3.3f
  #define USE_SERIAL1 1
  #undef SERIALP
  #define SERIALP Serial1
  #define USE_PIO_SERCOM 1
  //#include <Arduino.h>        // required before wiring_private.h - worked with out, some say it is required here?
  #include "wiring_private.h" // pinPeripheral() function - needed for Serial1
#endif

#if defined(_BOARD_NAME_) && defined(_DTWI0_BASE)
  // _BOARD_NAME_ "chipKIT uC32" they do not have a seperate define for each board so this may collide with other chipKIT boards
  // other option is to read _BOARD_NAME_ at run time and set the needed values
  #define arduinoVariant "chipKIT uC32"
  #define analogPins NUM_ANALOG_PINS
  #define digitalPins NUM_DIGITAL_PINS
  #define ioref 3.3f
#endif

#ifndef arduinoVariant
  #define arduinoVariant "Unknown"
#endif

#include "RTClib.h" // Adafruit  https://www.arduino.cc/reference/en/libraries/rtclib/
#include <hd44780.h> // Extensible hd44780 in library manager - https://github.com/duinoWitchery/hd44780 - worked on Due
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
// #include "SparkFun_External_EEPROM.h" // https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library

// Stated out with the common LiquidCrystal_I2C library, but ran into issues with it on the Due, commented out the old code, it is unmaintained/untested
// #include <LiquidCrystal_I2C.h> // https://www.arduinolibraries.info/libraries/liquid-crystal-i2-c - Frank de Brabander

RTC_DS3231 rtc; // DS3231_I2C_ADDRESS 0x68

#define LCD_COLS 20
#define LCD_ROWS 4
hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
// Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered)
// LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, LCD_COLS, LCD_ROWS); // Change to (0x27,16,2) for 16x2 LCD.

#if defined(LCD_COLS) && LCD_COLS == 20
  void clearLCDline(byte line) {
    lcd.setCursor(0, line);
    lcd.print("                    ");
  }
#else
  void clearLCDline(byte line) {
    lcd.setCursor(0, line);
    lcd.print("                ");
  }
#endif

//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// Convert normal decimal numbers to binary coded decimal
//byte decToBcd(byte val)
//{
//  return( (val/10*16) + (val%10) );
//}
// Convert binary coded decimal to normal decimal numbers
//byte bcdToDec(byte val)
//{
//  return( (val/16*10) + (val%16) );
//}

void jsonConfig() {
  // sends out the config defines
  SERIALP.print(F("{\"Config\": {")); // flash-memory based strings to save RAM
  SERIALP.print(F("\"SETPOINT_HIGH_LIMIT\": \""));
#if defined(SETPOINT_HIGH_LIMIT)
  SERIALP.print(SETPOINT_HIGH_LIMIT);
#else
  SERIALP.print(F("undef"));
#endif
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"SETPOINT_HIGH_HYSTERESIS\": \""));
  SERIALP.print(SETPOINT_HIGH_HYSTERESIS);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"SETPOINT_DC_HIGH_PIN\": \""));
  SERIALP.print(SETPOINT_DC_HIGH_PIN);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"DUTY_CYCLE_HIGH_SIGNAL\": \""));
  SERIALP.print(DUTY_CYCLE_HIGH_SIGNAL);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD\": \""));
  SERIALP.print(DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"DUTY_CYCLE_HIGH_DISABLE\": \""));
  SERIALP.print(DUTY_CYCLE_HIGH_DISABLE);
  SERIALP.print(F("\", "));
  
  SERIALP.print(F("\"SETPOINT_LOW_LIMIT\": \""));
#if defined(SETPOINT_LOW_LIMIT)
  SERIALP.print(SETPOINT_LOW_LIMIT);
#else
  SERIALP.print(F("undef"));
#endif
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"SETPOINT_LOW_HYSTERESIS\": \""));
  SERIALP.print(SETPOINT_LOW_HYSTERESIS);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"SETPOINT_DC_LOW_PIN\": \""));
  SERIALP.print(SETPOINT_DC_LOW_PIN);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"DUTY_CYCLE_LOW_SIGNAL\": \""));
  SERIALP.print(DUTY_CYCLE_LOW_SIGNAL);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"DUTY_CYCLE_LOW_SIGNAL_THRESHOLD\": \""));
  SERIALP.print(DUTY_CYCLE_LOW_SIGNAL_THRESHOLD);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"DUTY_CYCLE_LOW_DISABLE\": \""));
  SERIALP.print(DUTY_CYCLE_LOW_DISABLE);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"SETPOINT_RESTART_DELAY\": \""));
  SERIALP.print(SETPOINT_RESTART_DELAY);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"DUTY_CYCLE_FRAME_SAMPLES\": \""));
  SERIALP.print(DUTY_CYCLE_FRAME_SAMPLES);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"DUTY_CYCLE_FRAME_ROLLOVER\": \""));
  SERIALP.print(DUTY_CYCLE_FRAME_ROLLOVER);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"SETPOINT_HIGH_LIMIT_ALT\": \""));
  SERIALP.print(SETPOINT_HIGH_LIMIT_ALT);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"SETPOINT_LOW_LIMIT_ALT\": \""));
  SERIALP.print(SETPOINT_LOW_LIMIT_ALT);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"SETPOINT_ALT_PIN\": \""));  
  SERIALP.print(SETPOINT_ALT_PIN);
  SERIALP.print(F("\", "));
  SERIALP.print(F("\"CODE_CHANGE_MESSAGE\": \""));
#if defined(CODE_CHANGE_MESSAGE)
  SERIALP.print(CODE_CHANGE_MESSAGE);
#else
  SERIALP.print(F("undef"));
#endif
  SERIALP.print(F("\"")); // last item no trailing comma
  SERIALP.println(F("}}"));
}

void timeStamp(byte output) {
  DateTime now;
  if (!(bitwise_status & BWS_NO_RTC)) {
    now = rtc.now();
  }

  // timeStamp uses 2 ways to output data in the first 2 bits: Serial = 01 and LCD = 10
  // to save memory, directly output to the disired output
  // the next 2 bits have the format: whole timeStamp = 11, date only 10, time only 01

  if ((output & B00000011) == B0000001) {
    if ((bitwise_status & BWS_NO_RTC)) {
      SERIALP.print(F("No RTC"));
      return;
    }
    if ((output & B00001000) == B00001000) {
      SERIALP.print(now.year(), DEC);
      SERIALP.print("-");
      SERIALP.print(now.month(), DEC);
      SERIALP.print("-");
      SERIALP.print(now.day(), DEC);
    }
    if ((output & B00001100) == B00001100) {
      SERIALP.print("T");
    }
    if ((output & B00000100) == B00000100) {
      if (now.hour()<10) {
        SERIALP.print("0");
      }
      SERIALP.print(now.hour(), DEC);
      SERIALP.print(":");
      if (now.minute()<10) {
        SERIALP.print("0");
      }
      SERIALP.print(now.minute(), DEC);
      SERIALP.print(":");
      if (now.second()<10) {
        SERIALP.print("0");
      }
      SERIALP.print(now.second(), DEC);
    }
  } else {
    if ((bitwise_status & BWS_NO_RTC)) {
      lcd.print("No RTC");
      return;
    }
    if ((output & B00001000) == B00001000) {
      lcd.print(now.year(), DEC);
      lcd.print("-");
      lcd.print(now.month(), DEC);
      lcd.print("-");
      lcd.print(now.day(), DEC);
    }
    if ((output & B00001100) == B00001100) {
      lcd.print("T");
    }
    if ((output & B00000100) == B00000100) {
      if (now.hour()<10) {
        lcd.print("0");
      }
      lcd.print(now.hour(), DEC);
      lcd.print(":");
      if (now.minute()<10) {
        lcd.print("0");
      }
      lcd.print(now.minute(), DEC);
      lcd.print(":");
      if (now.second()<10) {
        lcd.print("0");
      }
      lcd.print(now.second(), DEC);
    }
  }
}

void jsonTimestamp() {
  // starts off the json line with a timestamp
  SERIALP.print("{\"Time\": \""); // {"Time": "2020-5-14T15:00:06"
  timeStamp(13);
  SERIALP.print("\"");
}

// setpoint control currently only implemented for digital pins
// hysteresis setup to go past setpoint (away from other setpoint) to reduce ringing when hysteresis is small
// DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD has multiple uses: analog input threshold, digital input and output active state
#if defined(SETPOINT_HIGH_LIMIT) && DUTY_CYCLE_HIGH_SIGNAL == 'D'
void check_setpoint_high(int high_limit) {
  if ((atemp[0] > high_limit + SETPOINT_HIGH_HYSTERESIS) && (setpoint_restart_delay == 0)) {
    digitalWrite(SETPOINT_DC_HIGH_PIN, DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD); // turn on reducing
  } else if (atemp[0] < high_limit) {
    if (digitalRead(SETPOINT_DC_HIGH_PIN) == DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD) {
      setpoint_restart_delay = SETPOINT_RESTART_DELAY;
    }
    digitalWrite(SETPOINT_DC_HIGH_PIN, DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD ^ 1); // turn off once hysteresis cleared
  }
}
#endif

#if defined(SETPOINT_LOW_LIMIT) && DUTY_CYCLE_LOW_SIGNAL == 'D'
void check_setpoint_low(int low_limit) {
  if ((atemp[0] < low_limit - SETPOINT_LOW_HYSTERESIS) && (setpoint_restart_delay == 0)) {
    digitalWrite(SETPOINT_DC_LOW_PIN, DUTY_CYCLE_LOW_SIGNAL_THRESHOLD); // turn on raising
  } else if (atemp[0] > low_limit) {
    if (digitalRead(SETPOINT_DC_LOW_PIN) == DUTY_CYCLE_LOW_SIGNAL_THRESHOLD) {
      setpoint_restart_delay = SETPOINT_RESTART_DELAY;
    }
    digitalWrite(SETPOINT_DC_LOW_PIN, DUTY_CYCLE_LOW_SIGNAL_THRESHOLD ^ 1); // turn off once hysteresis cleared
  }
}
#endif

void outputSerialNumber(byte output) { // output to serial = 1, to lcd = 2
  int count = 0;

  // code to read the serial # from AT24CS08
  Wire.beginTransmission(0x58); // base address of 128-bit (16 bytes) serial #
  Wire.write(0x80); // dummy write to set register pointer to 80h start of 128-bit (16 bytes) serial #
  if(count = Wire.endTransmission()) { // reuse count to save ram, 0 = success
    if (output == 1) {
      SERIALP.print(", \"SerialNumber\": \"Error: "); // "SerialNumber": "FFFFFFFFFFFFFFFF"
      SERIALP.print(count); // https://www.arduino.cc/en/Reference/WireEndTransmission for error codes
      SERIALP.print("\"");
    } else {
      lcd.print("SerialNumberError: ");
      lcd.print(count); // https://www.arduino.cc/en/Reference/WireEndTransmission for error codes
    }
    return;
  }
  
  Wire.beginTransmission(0x58);
  Wire.requestFrom(0x58, 16);
  if (output == 1) {
    SERIALP.print(", \"SerialNumber\": \""); // "SerialNumber": "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
  }
  while(Wire.available()) {    // slave may send less than requested
    unsigned char c = Wire.read();    // receive a byte as character
    count++;
    if (output == 1) {
      SERIALP.print(c < 16 ? "0" : ""); // so fixed length output
      SERIALP.print(c, HEX);
      // SERIALP.print(' '); // used to debug issues with print(c, HEX) on differnt types of Arduino
      // SERIALP.print(Wire.available());
      // SERIALP.print(' ');
    } else {
      lcd.print(c < 16 ? "0" : "");
      lcd.print(c, HEX);
      lcd.print(' ');
      if(count == 6) {
        lcd.setCursor(0, 2);
      } else if(count == 12) {
        lcd.setCursor(0, 3);
      }
    }
  }
  if (output == 1) {
    SERIALP.print("\"");
  }
}

#ifndef md5HASH
#endif
void outputMd5Hash(byte output) { // output to serial = 1, to lcd = 2
  static const char hash[] = md5HASH;
  int i;

  if (output == 1) {
    SERIALP.print(", \"md5Hash\": \""); // "md5Hash": "d2c8f43cdeda8221736a0445f4488adc" 32 chars
  }
  for (i = 0; i < strlen(hash); i++) {
    char c = hash[i];    // receive a byte as character
    if (output == 1) {
      SERIALP.print(c);
    } else {
      lcd.print(c);
      if(i == 15) {
        lcd.setCursor(0, 2);
      }
    }
  } // end for
  if (output == 1) {
    SERIALP.print("\"");
  }
}

void setup() {
  int status;

#if defined(USE_PIO_SERCOM)
  // Assign pins 0 & 1 SERCOM functionality on Metro M4
  pinPeripheral(0, PIO_SERCOM_ALT);
  pinPeripheral(1, PIO_SERCOM_ALT);
#endif

  // Open serial communications and wait for port to open:
#if defined(USE_SERIAL1)
  Serial1.begin(9600);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#else
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif
  // SERIALP.print("Test Serial first."); // use to test serial port before anything else jams things up

  Wire.begin(); // seems we do not need this, things seem to work the same if called or not called

  // for some reason if the rtc begin is done after the lcd init, the rtc begin fails
  if (! rtc.begin()) {
    SERIALP.println("{\"Error\": \"Could not find RTC\"}");
    bitwise_status = bitwise_status | BWS_NO_RTC;
  }

  // line below can be used to set a time module, someday add code so RTC can be set from reading the serial port
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // keep between rtc check and clock set check, uncomment, comple and run, immediately comment out, comple and run

  if (!(bitwise_status & BWS_NO_RTC) && rtc.lostPower()) {
    SERIALP.println("{\"Error\": \"RTC lost power, Time needs to be set,\"}");
    while (1);
  }

  jsonTimestamp(); // start up message time
  outputSerialNumber(1); // you can remove this if not sending data to cloud service
  outputMd5Hash(1); // show the check sum of the source code
  // output compile time / version info so can see in the field
  // Used info on http://forum.arduino.cc/index.php?topic=158014.0
  SERIALP.print(F(", \"Compiled\": \"" __DATE__ ", "));
  SERIALP.print(F( __TIME__ ", " __VERSION__ "\""));
  SERIALP.print(F(", \"arduinoVariant\": \"" arduinoVariant "\""));
  SERIALP.println("}");
  jsonConfig();

  // lcd.init(); // LiquidCrystal_I2C
  // lcd.backlight(); // lcd.noBacklight();
  status = lcd.begin(LCD_COLS, LCD_ROWS); // cols, rows
  if(status) { // non zero status means it was unsuccesful
    SERIALP.println(F("{\"Error\": \"LCD  begin command failed,\"}"));
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the onboard LED if possible
    // hd44780::fatalError(status); // does not return, uncomment if needed to troubleshoot LCD
  }

#if 1 // i2c_scanner can be handyfor troubleshooting - https://playground.arduino.cc/Main/I2cScanner/
  // device scan done one time so did not put in a function
  byte error, address;
  int nDevices;
 
  SERIALP.print(F("{\"I2C devices\": \""));
  lcd.setCursor(0, 0);
  lcd.print(F("I2C devices:"));
  lcd.setCursor(0, 1);
 
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // i2c_scanner uses the return value of Write.endTransmisstion to see if a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) { // 0: success
      if (nDevices>0) {
        SERIALP.print(", ");
        lcd.print(" ");
      }
      SERIALP.print("0x");
      if (address<16) {
        SERIALP.print("0");
        lcd.print("0");
      }
      SERIALP.print(address,HEX);
      lcd.print(address,HEX);
      nDevices++;
    } else if (error==4) { // 4: data byte transfer timeout
      if (nDevices>0) {
        SERIALP.print(", ");
        lcd.print(" ");
      }
      SERIALP.print(F("Error 4 at 0x"));
      lcd.print("Error@ ");
      if (address<16) {
        SERIALP.print("0");
        lcd.print("0");
      }
      SERIALP.print(address,HEX);
      lcd.print(address,HEX);
      nDevices++; // count all devices that send a known response
    } else if (error!=2) { // error = 2 if no device, to see other return values
      if (address>1) {
        SERIALP.print(", ");
      }
      SERIALP.print(F("Unknown response "));
      SERIALP.print(error);
      SERIALP.print(F(" at 0x"));
      if (address<16) {
        SERIALP.print("0");
      }
      SERIALP.print(address,HEX);
    }
    if (nDevices==6) {
      lcd.setCursor(0, 2);
    }
    if (nDevices==12) {
      lcd.setCursor(0, 3);
    }
    delay(10); // small delay to see each transmission on a scope
  }
  lcd.setCursor(15, 3);
  lcd.print("#:");
  lcd.print(nDevices);
  SERIALP.print(F("\", \"Number I2C devices\": "));
  SERIALP.print(nDevices);
  SERIALP.println("}");
  delay(3000);
#endif // i2c_scanner

  lcd.setCursor(0, 0); // Set the cursor on the first column and first row. X, Y

#if 0
  // The code below attempts put OpenLog in command mode, send a init command, display the return text on the lcd, and reset back to recording mode
  // The code works putting OpenLog in command mode and sending a command, but quickly runs into issues due to the lcd overflowing.
  // This code can be a start for reading from OpenLog, but needs more work.
  // The code is here to test that the RX hardware is working (read from OpenLog, OpenLog is not jamming the serial port during programming)
  // On an Uno the Arduino IDE Serial Monitor shows the commands sent to the OpenLog, the lcd shows the output of the OpenLog. Other Arduinos may behave differently.
  lcd.print("OpenLog: ");
  SERIALP.println(""); // move to a clean line
  SERIALP.print("\x1a\x1a\x1a");
  // Wait for OpenLog to return waiting for a command
  status = 100; // reuse status to save ram
  while(status > 0) {
    if(Serial.available())
      if(Serial.read() == '>') break;
    delay(20);
    status--;
  }
  lcd.setCursor(0, 1);
  if(status == 0) {
    lcd.print("Missing/No response");
  } else {
    int line = 0;
    SERIALP.println("init"); // regular println works with OpenLog v2.51 and above, can also try an ls command
    //delay(100);
    for (status = 0; status < 1000; status++) { // reuse status to save ram
      if (Serial.available() > 0) {
        char rec_char;

        rec_char = Serial.read();
        if (rec_char == 9) { // Tab
          lcd.print(' ');
        } else if (rec_char == 13) { // Return
          ;
        } else if (rec_char != 10) { // 10 = New Line
          lcd.print((char) rec_char);
        } else {
          line++;
          if(line == LCD_ROWS) { // reset to top line
            line = 0;
            // status = 1000; // stop displaying serial data
          }
          clearLCDline(line);
          lcd.setCursor(0, line);
          delay(50);
        }
        // SERIALP.print(rec_char); useful for troubleshooting, but will echo the text back to OpenLog causing unknown command errors
      } else {
        delay(10);
        if (Serial.available() == 0) {
          break; // if we still don't have any data, exit
        }
      }
    }
    SERIALP.println(""); // make sure reset has clean line
    delay(5000); // let data hang on the lcd
    SERIALP.println("reset"); // reset OpenLog to record data
  }
  delay(1000);
  SERIALP.println(""); // restart with a newline
  while(Serial.available()) {Serial.read();} // clear the serial buffer
#endif // OpenLog RX

  //ExternalEEPROM eeMem;
  //if (eeMem.begin(0b1010000) == false) { // , Wire1 0b1011(A2 A1 A0): a AT24CS08 I2C EEPROM w/ 128-bit (16 bytes) serial #
  //  SERIALP.println("No memory detected. Freezing.");
  //  while (1)
  //    ;
  //}
  //SERIALP.println("Memory detected!");
  //SERIALP.print("Mem size in bytes: ");
  //SERIALP.println(eeMem.length());
  //byte byteValue = 200;
  //const uint8_t myChars[2] = "L";
  //eeMem.write(128, myChars, 1); // (location, data) dummy write
  //byteValue = eeMem.read(128);
  //SERIALP.print("I read: ");
  //SERIALP.println(byteValue);

  clearLCDline(0);
  clearLCDline(1);
#if defined(LCD_ROWS) && LCD_ROWS == 4
  clearLCDline(2);
  clearLCDline(3);
#endif

  pinMode(7, INPUT_PULLUP); // pushbutton on GenuLog shield
  lcd.print("Serial #: ");
  lcd.setCursor(0, 1);
  outputSerialNumber(2);
  for (char count = 0; count < 2;) {
    delay(1000); // 1 sec button scan
    if (digitalRead(7) != 0) { // use pushbutton to hold display
      count++;
    }
  }

  clearLCDline(0);
  clearLCDline(1);
#if defined(LCD_ROWS) && LCD_ROWS == 4
  clearLCDline(2);
  clearLCDline(3);
#endif

  lcd.print("md5 hash: ");
  lcd.setCursor(0, 1);
  outputMd5Hash(2);
  for (char count = 0; count < 2; ) {
    delay(1000); // 1 sec button scan
    if (digitalRead(7) != 0) { // use pushbutton to hold display
      count++;
    }
  }

  //pinMode(0, INPUT_PULLUP); // warning - may disturb serial port
  //pinMode(1, INPUT_PULLUP); // warning - may disturb serial port
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  // pinMode(7, INPUT_PULLUP); set above
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);  // LED

  // setup for setpoint
#if defined(SETPOINT_HIGH_LIMIT) && DUTY_CYCLE_HIGH_SIGNAL == 'D'
  digitalWrite(SETPOINT_DC_HIGH_PIN, DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD ^ 1);
  pinMode(SETPOINT_DC_HIGH_PIN, OUTPUT);
#endif
#if defined(SETPOINT_LOW_LIMIT) && DUTY_CYCLE_LOW_SIGNAL == 'D'
  digitalWrite(SETPOINT_DC_LOW_PIN, DUTY_CYCLE_LOW_SIGNAL_THRESHOLD ^ 1);
  pinMode(SETPOINT_DC_LOW_PIN, OUTPUT);
#endif

} // end setup()

void loop() {
  //SERIALP.print(" * ");
  //lcd.setCursor(0, 1);
  //lcd.print("Test");
  //SERIALP.println("");
  //return; // the above block tests writing to serial port and lcd with out involving other functions
  DateTime now;
  if (!(bitwise_status & BWS_NO_RTC)) {
    now = rtc.now(); // get time and date from RTC and save in variables
  }
  jsonTimestamp(); // will send no RTC message
  SERIALP.print(", ");

  // display prev data and time
  lcd.setCursor(0, 1);
  if (!(bitwise_status & BWS_NO_RTC)) {
    if (thour<10) {
      lcd.print("0");
    }
    lcd.print(thour, DEC);
    lcd.print(":");
    if (tmin<10) {
      lcd.print("0");
    }
    lcd.print(tmin, DEC);
    lcd.print(":");
    if (tsec<10) {
      lcd.print("0");
    }
    lcd.print(tsec, DEC);
  } else {
    lcd.print("Dly:");
    lcd.print(delay_sec);
  }
  lcd.print(" T");
  lcd.print(atemp[0], 0); // print Temperature 1 = one decimal point of precision, 0 = whole number

#if DUTY_CYCLE_LOW_DISABLE == 0
  lcd.print(" LDC");
  lcd.print(duty_cycle_raising); // show duty cycle
  lcd.print(' ');
#else 
  lcd.print(" V");
  lcd.print(voltage_display, 1); // show battery, or what is in voltage_display
#endif

  if (!(bitwise_status & BWS_NO_RTC)) {
    thour = now.hour(); // save data time
    tmin = now.minute();
    tsec = now.second();
  }

  for (int pin = 0; pin < analogPins; pin++) {
    SERIALP.print("\"A"); // A = analog
    SERIALP.print(pin);
    SERIALP.print("\": ");
    // In general, it is preferred to do signal processing downstream where issues are easier to fix
    // and the core data logging function is not impacted. But if an application does not use any
    // downstream processes, below is an example of calculating the temp from a TMP36.
    // Processing in this code allows the scaled values to be displayed on the connected LCD.
    // if (bitRead(bitwise_status, BWS_NO_DATA)) { // code to only pre-read on first read
    //  analogRead(pin); // dummy read to switch channel, recommended, needed on Due or first read is bad
    //  delayMicroseconds(100); // wait for s/h
    // }
    analogRead(pin); // dummy read to switch channel, recommended, needed on Due or first read is bad
    delayMicroseconds(100); // wait for s/h
    if (pin == 0 || pin == 1 || pin == 2) { // || pin == 1
      // convert reading from raw a/d to voltage and temp F or temp C, for 3.3v arduino ioref = 3.3
      // fill atemp from 0 so atemp[pin] does not have gaps that require a bigger array
      temp_int = atemp[pin] = analogRead(pin);
      SERIALP.print(temp_int); // output raw value
      temp_float = temp_int;
      temp_float /= 205;
      SERIALP.print(", \"A");
      SERIALP.print(pin);
      SERIALP.print("volt\": "); // output voltage
      SERIALP.print(temp_float);
      atemp[pin] *= ioref;
      SERIALP.print(", \"A");
      SERIALP.print(pin);
      SERIALP.print("tempF\": "); // update if using C
      atemp[pin] /= 1023.0;
      atemp[pin] = (atemp[pin] - 0.5) * 100; // converting to C, 10 mv per degree C with 500 mV offset
      // SERIALP.print(atemp[pin]); this line will output C
      atemp[pin] = (atemp[pin] * 9.0 / 5.0) + 32.0; // now convert to Fahrenheit
      SERIALP.print(atemp[pin]); // output tempF
    } else if (pin == 200) { // 5V FS no input voltage divider on 5V ioref, pin == 200 - disable
      temp_int = analogRead(pin);
      SERIALP.print(temp_int); // output raw value
      temp_float = temp_int;
      temp_float /= 205;
      SERIALP.print(", \"A");
      SERIALP.print(pin);
      SERIALP.print("volt\": ");
      SERIALP.print(temp_float);
    } else if (pin == 1 || pin == 2 || pin == 3 || pin == 4 || pin == 5 || pin == 6 || pin == 7) { // 40.75V FS input 71.5K/10K on 5V ioref, default if not already used
      temp_int = analogRead(pin);
      SERIALP.print(temp_int); // output raw value
      temp_float = temp_int<<2;
      temp_float /= 100;
      SERIALP.print(", \"A");
      SERIALP.print(pin);
      SERIALP.print("volt\": ");
      SERIALP.print(temp_float);
      if (pin == VOLTAGE_DISPLAY_PIN) {
        voltage_display = temp_float; // voltage to show in LCD if duty cycle calc turned off
      }
    } else {
      SERIALP.print(analogRead(pin)); // if no special treatment, send raw value
    }
    if (pin < analogPins - 1) { // do not add , on last item
      SERIALP.print(", ");
    }
  }

  if (!(bitwise_status & BWS_NO_RTC)) {
    SERIALP.print(F(", \"RTCTEMP\": "));
    SERIALP.print(rtc.getTemperature());
  }

  SERIALP.print(", ");
  // D0, D1 serial port so start at 2
  for (int pin = 2; pin < digitalPins; pin++) {
    SERIALP.print("\"D");
    SERIALP.print(pin);
    SERIALP.print("\": ");
    SERIALP.print(digitalRead(pin));
    if (pin < digitalPins - 1) {
      SERIALP.print(", ");
    }
  }

#if DUTY_CYCLE_LOW_DISABLE == 0
  SERIALP.print(F(", \"duty_cycle_raising\": "));
  SERIALP.print(duty_cycle_raising);
#endif
#if DUTY_CYCLE_HIGH_DISABLE == 0
  SERIALP.print(F(", \"duty_cycle_reducing\": "));
  SERIALP.print(duty_cycle_reducing);
#endif
#if DUTY_CYCLE_LOW_DISABLE == 0 || DUTY_CYCLE_HIGH_DISABLE == 0
  SERIALP.print(F(", \"duty_cycle_total_count\": "));
  SERIALP.print(duty_cycle_total_count);
#endif

  // if new day reset high and low trackers, do after data set
  if (!(bitwise_status & BWS_NO_RTC) && (tday != now.day())) {
    yhtemp = thtemp;
    yltemp = tltemp;
    thtemp = atemp[0];
    tltemp = atemp[0];
    tday = now.day();
    clearLCDline(0); // clear screen once a day
    clearLCDline(1);
    clearLCDline(2);
    clearLCDline(3);
  }

  lcd.setCursor(0, 0); // Set the cursor on the first column and first row. X, Y
  timeStamp(6);
  lcd.print(" T");
  lcd.print(atemp[0], 0); // print Temperature 1 = one decimal point of precision, 0 = whole number

#if DUTY_CYCLE_HIGH_DISABLE == 0
  lcd.print(" HDC");
  lcd.print(duty_cycle_reducing);
  lcd.print(' ');
#else
  lcd.print(" V");
  lcd.print(voltage_display, 1); // show battery, or what is in voltage_display
#endif

  // check if we have new high or lows
  if (atemp[0] > thtemp) {
    thtemp = atemp[0];
  }
  if (atemp[0] < tltemp) {
    tltemp = atemp[0];
  }

  lcd.setCursor(0, 2);
  if (!(bitwise_status & BWS_NO_RTC)) {
    lcd.print(now.year() - 2000, DEC);
    lcd.print("-");
    if (now.month()<10) {
      lcd.print("0");
    }
    lcd.print(now.month(), DEC);
    lcd.print("-");
    if (now.day()<10) {
      lcd.print("0");
    }
    lcd.print(now.day(), DEC);
  } else {
    lcd.print(F("No RTC  "));
  }

  lcd.print(" H");
  if (thtemp>99 || thtemp<0) {
    lcd.print(thtemp, 0);
  } else {
    lcd.print(thtemp, 1);
  }
  lcd.print(" L");
  if (tltemp>99 || tltemp<0) {
    lcd.print(tltemp, 0);
  } else {
    lcd.print(tltemp, 1);
  }

  lcd.setCursor(0, 3);
  if (yhtemp == -999 && yltemp == 999) {
    lcd.print("Yesterday: No Data");
  } else {
    lcd.print("Yeday H");
    lcd.print(yhtemp, 1);
    lcd.print(" L");
    lcd.print(yltemp, 1);
  }

  if (lastTimestamp == 0) {
    lastTimestamp = now.unixtime(); // so first loop does not create a huge elapsedTimestamp
  }
  if (!(bitwise_status & BWS_NO_RTC)) {
    elapsedTimestamp = now.unixtime() - lastTimestamp; // 1642623145
  } else {
    elapsedTimestamp = 0;
  }

// use bottom right coner for monitoring setpoint_restart_delay or serial rx
#if defined(SETPOINT_HIGH_LIMIT) || defined(SETPOINT_LOW_LIMIT)
  lcd.setCursor(19, 3); // bottom right coner
  if (setpoint_restart_delay > 0) {
    setpoint_restart_delay--;
    lcd.print(setpoint_restart_delay);
  } else {
    lcd.print(' ');
  }
#else
  if (Serial.available() > 0) {
    // for testing serial rx, or as an example of how to rx and display serial data
    lcd.setCursor(19, 3); // bottom right coner
    char rec_char;
    rec_char = Serial.read();
    if (rec_char != 10) {
      lcd.print((char) rec_char);
    }
  }
#endif

#if defined(SETPOINT_HIGH_LIMIT)
  if (digitalRead(SETPOINT_ALT_PIN) != 0) {
    check_setpoint_high(SETPOINT_HIGH_LIMIT);
  }
#endif

#if defined(SETPOINT_LOW_LIMIT)
  if (digitalRead(SETPOINT_ALT_PIN) != 0) {
    check_setpoint_low(SETPOINT_LOW_LIMIT);
  }
#endif

#if defined(SETPOINT_HIGH_LIMIT_ALT) && defined(SETPOINT_HIGH_LIMIT) // non alt must be used to trun alt setpoint
  if (digitalRead(SETPOINT_ALT_PIN) == 0) {
    check_setpoint_high(SETPOINT_HIGH_LIMIT_ALT);
  }
#endif

#if defined(SETPOINT_LOW_LIMIT_ALT) && defined(SETPOINT_LOW_LIMIT) // non alt must be used to trun alt setpoint
  if (digitalRead(SETPOINT_ALT_PIN) == 0) {
    check_setpoint_low(SETPOINT_LOW_LIMIT_ALT);
  }
#endif

#if DUTY_CYCLE_LOW_DISABLE == 0 || DUTY_CYCLE_HIGH_DISABLE == 0
  duty_cycle_total_count++;
#endif

#if DUTY_CYCLE_LOW_DISABLE == 0
#if DUTY_CYCLE_LOW_SIGNAL == 'D'
  if (digitalRead(SETPOINT_DC_LOW_PIN) == DUTY_CYCLE_LOW_SIGNAL_THRESHOLD) {
    duty_cycle_raising_count++;
  }
#else
  if ((DUTY_CYCLE_LOW_SIGNAL_THRESHOLD >= 0) && (analogRead(SETPOINT_DC_LOW_PIN) >= DUTY_CYCLE_LOW_SIGNAL_THRESHOLD)) {
    duty_cycle_raising_count++;
  } else if ((DUTY_CYCLE_LOW_SIGNAL_THRESHOLD < 0) && (analogRead(SETPOINT_DC_LOW_PIN) <= DUTY_CYCLE_LOW_SIGNAL_THRESHOLD)) {
    duty_cycle_raising_count++;
  }
#endif
  temp_int = duty_cycle_raising_count;
  temp_int *= 100;
  duty_cycle_raising = temp_int / duty_cycle_total_count;
#endif

#if DUTY_CYCLE_HIGH_DISABLE == 0
#if DUTY_CYCLE_HIGH_SIGNAL == 'D'
  if (digitalRead(SETPOINT_DC_HIGH_PIN) == DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD) {
    duty_cycle_reducing_count++;
  }
#else
  if ((DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD >= 0) && (analogRead(SETPOINT_DC_HIGH_PIN) >= DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD)) {
    duty_cycle_reducing_count++;
  } else if ((DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD < 0) && (analogRead(SETPOINT_DC_HIGH_PIN) <= DUTY_CYCLE_HIGH_SIGNAL_THRESHOLD)) {
    duty_cycle_reducing_count++;
  }
#endif
  temp_int = duty_cycle_reducing_count;
  temp_int *= 100;
  duty_cycle_reducing = temp_int / duty_cycle_total_count;
#endif

#if DUTY_CYCLE_LOW_DISABLE == 0 || DUTY_CYCLE_HIGH_DISABLE == 0
#if DUTY_CYCLE_HIGH_DISABLE == 0
  SERIALP.print(F(", \"duty_cycle_reducing_count\": ")); // useful for troubleshooting
  SERIALP.print(duty_cycle_reducing_count);
#endif
#if DUTY_CYCLE_LOW_DISABLE == 0
  SERIALP.print(F(", \"duty_cycle_raising_count\": ")); // useful for troubleshooting
  SERIALP.print(duty_cycle_raising_count);
#endif
  if (duty_cycle_total_count >= DUTY_CYCLE_FRAME_SAMPLES) { // rollover so old samples fade
    // SERIALP.print(", \"duty_cycle_frame_rollover\": "); // messes up csv files, so only use if needed for troubleshooting
    // SERIALP.print(DUTY_CYCLE_FRAME_SAMPLES);
    duty_cycle_total_count = DUTY_CYCLE_FRAME_ROLLOVER * DUTY_CYCLE_FRAME_SAMPLES;
#if DUTY_CYCLE_LOW_DISABLE == 0
    duty_cycle_raising_count = DUTY_CYCLE_FRAME_ROLLOVER * duty_cycle_raising_count;
#endif
#if DUTY_CYCLE_HIGH_DISABLE == 0
    duty_cycle_reducing_count = DUTY_CYCLE_FRAME_ROLLOVER * duty_cycle_reducing_count;
#endif
  }
#endif

  if (!(bitwise_status & BWS_NO_RTC)) {
    lastTimestamp = now.unixtime();
  }

  outputSerialNumber(1); // last so not using space early in the line, you can remove this if not sending data to cloud service
  SERIALP.println("}");

  bitClear(bitwise_status, BWS_NO_DATA); // clear BWS_NO_DATA now that we have data

  digitalWrite(LED_BUILTIN, LOW); // start of waiting for next processing cycle
  delay_sec = 0;
  // Control sample speed with D5, D6. Default to 10 min.
  if ((digitalRead(6) == 0) &&  (digitalRead(5) == 0)) {
    delay(1000); // 1 sec
    delay_sec = 1;
  } else if ((digitalRead(6) == 1) &&  (digitalRead(5) == 0)) {
    delay(10000); // 10 sec
    delay_sec = 10;
  } else if ((digitalRead(6) == 0) &&  (digitalRead(5) == 1)) {
    for (int count = 0; count < 6; count++) { // 1 min
      delay(10000); // 10 sec button scan
      delay_sec += 10;
      if (digitalRead(7) == 0) { // use pushbutton to cut short long sleep times
        break;
      }
    }
  } else if ((digitalRead(6) == 1) &&  (digitalRead(5) == 1)) {
    for (int count = 0; count < 60; count++) { // 10 min
      delay(10000); // 10 sec button scan
      delay_sec += 10;
      if (digitalRead(7) == 0) { // use pushbutton to cut short long sleep times
        break;
      }
    }
  }
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on while processing
}
