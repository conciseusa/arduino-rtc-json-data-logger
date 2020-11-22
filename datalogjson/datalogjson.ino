
// Script to read A & D pins, timestamp the data, and send out the serial port in a json packet
// A serial logger (OpenLog) can be used to record the data, or a RPi can be used as a gateway to the internet
// This approach was taken because the system will be used with batter/solar power.
// The RPi can be shut down and just the core data logging can run when power needs to be conserved.
// A LCD shows the last 2 readings and the high and low for today and yesterday. A 20X4 LCD is needed to show that much data.
// Pins used: D0, D1 for serial port, D2, D3 for sample speed, I2C port (depends on Arduino flavor)
//            D7 push button as used in https://www.arduino.cc/en/tutorial/pushbutton, LED_BUILTIN
// It is possible the code can be used unmodified in many applications.
// But many applications will require some modification. Even so, having sample code to talk to all the components should speed things up.
// It is a mix of code I wrote and example code I found
// Mashup by Walter Spurgiasz 2020
// Code inspired from:
// https://lastminuteengineers.com/ds3231-rtc-arduino-tutorial/
// https://www.makerguides.com/character-i2c-lcd-arduino-tutorial/
// Tested on Uno, Mega2560, Due, Adafruit Metro M4 Express AirLift (WiFi) - Lite, chipKIT uC32
// Adafruit Metro M4 Express AirLift (WiFi) - Lite
// https://learn.adafruit.com/adafruit-metro-m4-express-featuring-atsamd51/setup
// File->Preferences->Additional Boards Manager URLS (comma seperated) https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
// Tools->Board->Board Manager  Scroll down to the Adafruit and install
// chipKIT uC32 https://blog.digilentinc.com/how-to-program-your-chipkit-board-in-the-arduino-ide/
// File->Preferences->Additional Boards Manager URLS (comma seperated) https://github.com/chipKIT32/chipKIT-core/raw/master/package_chipkit_index.json
// Tools->Board->Board Manager  Scroll down to the chipKIT and install
// On 64b Ubuntu - sudo apt-get install libc6-i386 - More details at http://chipkit.net/wiki/index.php?title=ChipKIT_core -> 64-Bit Linux

float temp; // used for scaling TMP36 temperature sensor
char thour = 99; // save temp time
char tmin = 99;
char tsec = 99;
char tday = 99; // save day so know when to reset high and low
float thtemp = -999; // set so any normal reading above
float tltemp = 999; // set so any normal reading below
float yhtemp = -999;
float yltemp = 999;

#include "Wire.h"
#define SERIALP Serial // different Arduino have variuos serial ports, so control which one we use

// from http://forum.arduino.cc/index.php?topic=113656.0
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  // Code in here will only be compiled if an Arduino Uno (or older) is used.
  #define analogPins 4
  #define digitalPins 14
  #define ioref 5.0f // operating voltage on an Uno, float becaused coud be 3.3 and used to scale A/D converter
#endif
  
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  // Code in here will only be compiled if an Arduino Mega is used.
  #define analogPins 16
  #define digitalPins 54
  #define ioref 5.0f
#endif

#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
  // Code in here will only be compiled if an Arduino Leonardo is used.
#endif

#if defined(__SAM3X8E__)
  // Code in here will only be compiled if an Arduino Due is used.
  #define analogPins 16
  #define digitalPins 54
  #define ioref 3.3f
  #define Wire Wire1 // this is needed on Due
#endif

#if defined(_VARIANT_METRO_M4_WIFI_)
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
  #define analogPins NUM_ANALOG_PINS
  #define digitalPins NUM_DIGITAL_PINS
  #define ioref 3.3f
#endif

#include "RTClib.h"
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

void timeStamp(byte output) {
  DateTime now = rtc.now();

  // timeStamp uses 2 ways to output data in the first 2 bits: Serial = 01 and LCD = 10
  // to save memory, directly output to the disired output
  // the next 2 bits have the format: whole timeStamp = 11, date only 10, time only 01

  if ((output & B00000011) == B0000001) {
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
  SERIALP.print("\", ");
}

void outputSerialNumber(byte output) { // output to serial = 1, to lcd = 2
  int count = 0;

  // code to read the serial # from AT24CS08
  Wire.beginTransmission(0x58); // base address of 128-bit (16 bytes) serial #
  Wire.write(0x80); // dummy write to set register pointer to 80h start of 128-bit (16 bytes) serial #
  if(count = Wire.endTransmission()) { // reuse count to save ram, 0 = success
    if (output == 1) {
      SERIALP.print("\"SerialNumber\": \"Error: "); // "SerialNumber": "FFFFFFFFFFFFFFFF"
      SERIALP.print(count); // https://www.arduino.cc/en/Reference/WireEndTransmission for error codes
      SERIALP.print("\", ");
    } else {
      lcd.print("SerialNumberError: ");
      lcd.print(count); // https://www.arduino.cc/en/Reference/WireEndTransmission for error codes
    }
    return;
  }
  
  Wire.beginTransmission(0x58);
  Wire.requestFrom(0x58, 16);
  if (output == 1) {
    SERIALP.print("\"SerialNumber\": \""); // "SerialNumber": "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF"
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
    SERIALP.print("\", ");
  }
}

void setup() {
  int status;

  #if defined(USE_PIO_SERCOM)
    // Assign pins 0 & 1 SERCOM functionality on Metro M4
    pinPeripheral(0, PIO_SERCOM_ALT);
    pinPeripheral(1, PIO_SERCOM_ALT);
  #endif

  Wire.begin(); // seems we do not need this, things seem to work the same if called or not called

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


  #if 1 // i2c_scanner can be handyfor troubleshooting - https://playground.arduino.cc/Main/I2cScanner/
  byte error, address;
  int nDevices;
 
  SERIALP.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    // i2c_scanner uses the return value of Write.endTransmisstion to see if a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0) {
      SERIALP.print("I2C device found at address 0x");
      if (address<16)
        SERIALP.print("0");
      SERIALP.print(address,HEX);
      SERIALP.println("  !");
 
      nDevices++;
    } else if (error==4) {
      SERIALP.print("Unknown error at address 0x");
      if (address<16)
        SERIALP.print("0");
      SERIALP.println(address,HEX);
    }    
  }
  if (nDevices == 0) 
    SERIALP.println("No I2C devices found\n");
  else
    SERIALP.println("Scan done\n");
  #endif // i2c_scanner

  // for some reason if the rtc begin is done after the lcd init, the rtc begin fails
  while (! rtc.begin()) {
    SERIALP.println("{\"Error\": \"Could not find RTC\"}");
    delay(5000);
  }

  // line below can be used to set a time module, someday add code so RTC can be set from reading the serial port
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // keep between rtc check and clock set check, uncomment, comple and run, immediately comment out, comple and run

  if (rtc.lostPower()) {
    SERIALP.println("{\"Error\": \"RTC lost power, Time needs to be set,\"}");
    while (1);
  }

  SERIALP.println("");
  jsonTimestamp(); // start up message time
  outputSerialNumber(1); // you can remove this if not sending data to cloud service
  // output compile time / version info so can see in the field
  // Used info on http://forum.arduino.cc/index.php?topic=158014.0
  SERIALP.print("\"Compiled\": \"" __DATE__ ", " __TIME__ ", " __VERSION__ "\"");
  SERIALP.println("}");

  // lcd.init(); // LiquidCrystal_I2C
  // lcd.backlight(); // lcd.noBacklight();
  status = lcd.begin(LCD_COLS, LCD_ROWS); // cols, rows
  if(status) { // non zero status means it was unsuccesful
    SERIALP.println("{\"Error\": \"LCD  begin command failed,\"}");
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the onboard LED if possible
    // hd44780::fatalError(status); // does not return, uncomment if needed to troubleshoot LCD
  }
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

  lcd.print("Serial #: ");
  lcd.setCursor(0, 1);
  outputSerialNumber(2);

  delay(5000);

  //pinMode(0, INPUT_PULLUP); // warning - may disturb serial port
  //pinMode(1, INPUT_PULLUP); // warning - may disturb serial port
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

} // end setup()

void loop() {
  //SERIALP.print(" * ");
  //lcd.setCursor(0, 1);
  //lcd.print("Test");
  //SERIALP.println("");
  //return; // the above block tests writing to serial port and lcd with out involving other functions

  DateTime now = rtc.now();
  jsonTimestamp();
  outputSerialNumber(1); // you can remove this if not sending data to cloud service

  // display prev data and time
  lcd.setCursor(0, 1);
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
  lcd.print(" Temp ");
  lcd.print(temp, 1);

  thour = now.hour(); // save data time
  tmin = now.minute();
  tsec = now.second();

  for (int pin = 0; pin < analogPins; pin++) {
    SERIALP.print("\"A"); // A = analog
    SERIALP.print(pin);
    SERIALP.print("\": ");
    if (pin == 0) { // for TMP36
      // converting that reading to voltage, for 3.3v arduino use 3.3
      temp = analogRead(pin); // dummy read to switch channel
      delayMicroseconds(100); // wait for s/h
      temp = analogRead(pin) * ioref;
      temp /= 1024.0;
      temp = (temp - 0.5) * 100; // converting to C, 10 mv per degree C wit 500 mV offset
      temp = (temp * 9.0 / 5.0) + 32.0; // now convert to Fahrenheit
      SERIALP.print(temp);
    } else {
      SERIALP.print(analogRead(pin));
    }
    if (pin < analogPins - 1) {
      SERIALP.print(", ");
    }
  }

  if (1) {
    SERIALP.print(", \"RTCTEMP\": ");
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
  
  SERIALP.println("}");

  // if new day reset high and low trackers, do after data set
  if (tday != now.day()) {
    yhtemp = thtemp;
    yltemp = tltemp;
    thtemp = temp;
    tltemp = temp;
    tday = now.day();
    clearLCDline(0); // clear screen once a day
    clearLCDline(1);
    clearLCDline(2);
    clearLCDline(3);
  }

  lcd.setCursor(0, 0); // Set the cursor on the first column and first row. X, Y
  timeStamp(6);
  lcd.print(" Temp ");
  lcd.print(temp, 1); // print temp with one decimal point of precision

  // check if we have new high or lows
  if (temp > thtemp) {
    thtemp = temp;
  }
  if (temp < tltemp) {
    tltemp = temp;
  }

  lcd.setCursor(0, 2);
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
  lcd.print(" H");
  if (thtemp>99) {
    lcd.print(thtemp, 0);
  } else {
    lcd.print(thtemp, 1);
  }
  lcd.print(" L");
  if (tltemp>99) {
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

  if (Serial.available() > 0) {
    // for testing serial rx, or as an example of how to rx and display serial data
    lcd.setCursor(19, 3); // bottom right coner
    char rec_char;
    rec_char = Serial.read();
    if (rec_char != 10) {
      lcd.print((char) rec_char);
    }
  }

  // Control sample speed with D2, D3
  if ((digitalRead(3) == 1) &&  (digitalRead(2) == 1)) {
    delay(1000); // 1 sec
  } else if ((digitalRead(3) == 1) &&  (digitalRead(2) == 0)) {
    delay(10000); // 10 sec
  } else if ((digitalRead(3) == 0) &&  (digitalRead(2) == 1)) {
    delay(60000); // 1 min
  } else if ((digitalRead(3) == 0) &&  (digitalRead(2) == 0)) {
    delay(600000); // 10 min
  }
}
