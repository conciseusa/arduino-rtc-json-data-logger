
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

#include "RTClib.h"
#include <hd44780.h> // main hd44780 header https://github.com/duinoWitchery/hd44780 - worked on Due
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header
#include "SparkFun_External_EEPROM.h" // https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library

// Stated out with the common LiquidCrystal_I2C, but ran into issues with it on the Due, commented out the old code, but it is unmaintained/untested
// #include <LiquidCrystal_I2C.h> // https://www.arduinolibraries.info/libraries/liquid-crystal-i2-c - Frank de Brabander

RTC_DS3231 rtc; // DS3231_I2C_ADDRESS 0x68
ExternalEEPROM eeMem;

#define LCD_WIDTH 20
hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip
// Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered)
// LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, LCD_WIDTH, 4); // Change to (0x27,16,2) for 16x2 LCD.

#if defined(LCD_WIDTH) && LCD_WIDTH == 20
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
      Serial.print(now.year(), DEC);
      Serial.print("-");
      Serial.print(now.month(), DEC);
      Serial.print("-");
      Serial.print(now.day(), DEC);
    }
    if ((output & B00001100) == B00001100) {
      Serial.print("T");
    }
    if ((output & B00000100) == B00000100) {
      if (now.hour()<10) {
        Serial.print("0");
      }
      Serial.print(now.hour(), DEC);
      Serial.print(":");
      if (now.minute()<10) {
        Serial.print("0");
      }
      Serial.print(now.minute(), DEC);
      Serial.print(":");
      if (now.second()<10) {
        Serial.print("0");
      }
      Serial.print(now.second(), DEC);
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
  Serial.print("{\"Time\": \""); // {"Time": "2020-5-14T15:00:06"
  timeStamp(13);
  Serial.print("\", ");
}

void outputSerialNumber(byte output) { // output to serial = 1, to lcd = 2
  int count = 0;

  // code to read the serial # from AT24CS08
  Wire.beginTransmission(0x58); // base address of 128-bit (16 bytes) serial #
  Wire.write(0x80); // dummy write to set register pointer to 80h start of 128-bit (16 bytes) serial #
  Wire.endTransmission();
  Wire.beginTransmission(0x58);
  Wire.requestFrom(0x58, 16);
  if (output == 1) {
    Serial.print("\"SerialNumber\": \""); // "SerialNumber": "FFFFFFFFFFFFFFFF"
  }
  while(Wire.available()) {    // slave may send less than requested
    char c = Wire.read();    // receive a byte as character
    count++;
    if (output == 1) {
      Serial.print(c < 16 ? "0" : ""); // so fixed length output
      Serial.print(c, HEX);
      //Serial.print(' ');
    } else {
      lcd.print(c < 16 ? "0" : "");
      lcd.print(c, HEX);
      lcd.print(' ');
      if(count == 6) {
        lcd.setCursor(0, 2);
      }
      else if(count == 12) {
        lcd.setCursor(0, 3);
      }
    }
  }
  if (output == 1) {
    Serial.print("\", ");
  }
}

void setup() {
  int status;
  //Wire.begin(); // seems we do not need this

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // for some reason if the rtc begin is done after the lcd init, the rtc begin fails
  while (! rtc.begin()) {
    Serial.println("{\"Error\": \"Could not find RTC\"}");
    delay(1000); // 1 sec
  }

  // line below can be used to set a time module, someday add code so RTC can be set from reading the serial port
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // keep between rtc check and clock set check

  if (rtc.lostPower()) {
    Serial.println("{\"Error\": \"RTC lost power, Time needs to be set,\"}");
    while (1);
  }

  Serial.println("");
  jsonTimestamp(); // start up message time
  outputSerialNumber(1); // you can remove this if not sending data to cloud service
  // output compile time / version info so can see in the field
  // Used info on http://forum.arduino.cc/index.php?topic=158014.0
  Serial.print("\"Compiled\": \"" __DATE__ ", " __TIME__ ", " __VERSION__ "\"");
  Serial.println("}");

  lcd.init();
  lcd.backlight(); // lcd.noBacklight();
  status = lcd.begin(LCD_WIDTH, 4);
  if(status) { // non zero status means it was unsuccesful
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so blink error code using the onboard LED if possible
    hd44780::fatalError(status); // does not return
  }
  lcd.setCursor(0, 0); // Set the cursor on the first column and first row. X, Y

  #if 0
  // The code below attempts put OpenLog in command mode, send a init command, display the return text on the lcd, and reset back to recording mode
  // The code works putting OpenLog in command mode and sending a command, but quickly runs into issues due to the lcd overflowing.
  // This code can be a start for reading from OpenLog, but needs more work.
  // The code is here to test that the RX hardware is working (read from OpenLog, OpenLog is not jamming the serial port during programming) 
  lcd.print("OpenLog: ");
  Serial.println(""); // move to a clean line
  Serial.print("\x1a\x1a\x1a");
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
    Serial.println("init"); // regular println works with OpenLog v2.51 and above
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
          if(line == 4) { // reset to top line
            line = 0;
            // status = 1000; // stop displaying serial data
          }
          clearLCDline(line);
          lcd.setCursor(0, line);
          delay(50);
        }
        // Serial.print(rec_char); useful for troubleshooting, but will echo the text back to OpenLog causing unknown command errors
      } else {
        delay(10);
        if (Serial.available() == 0) {
          break; // if we still don't have any data, exit
        }
      }
    }
    Serial.println(""); // make sure reset has clean line
    delay(5000); // let data hang on the lcd
    Serial.println("reset"); // reset OpenLog to record data
  }
  delay(1000);
  Serial.println(""); // restart with a newline
  while(Serial.available()) {Serial.read();} // clear the serial buffer
  #endif // OpenLog RX

  //if (eeMem.begin(0b1010000) == false) { // , Wire1 0b1011(A2 A1 A0): a AT24CS08 I2C EEPROM w/ 128-bit (16 bytes) serial #
  //  Serial.println("No memory detected. Freezing.");
  //  while (1)
  //    ;
  //}
  //Serial.println("Memory detected!");
  //Serial.print("Mem size in bytes: ");
  //Serial.println(eeMem.length());
  //byte byteValue = 200;
  //const uint8_t myChars[2] = "L";
  //eeMem.write(128, myChars, 1); // (location, data) dummy write
  //byteValue = eeMem.read(128);
  //Serial.print("I read: ");
  //Serial.println(byteValue);

  //Wire.beginTransmission(0x58);
  //Wire.send(21000 >> 8);  // send MSB of the address
  //Wire.send(21000 & 0xFF); // send LSB of the address
  //Wire.endTransmission();

  lcd.print("Serial #: ");
  lcd.setCursor(0, 1);
  outputSerialNumber(2);

  delay(5000);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

} // end setup()

void loop() {
  //Serial.print(" * ");
  //lcd.setCursor(0, 1);
  //lcd.print("Test");

  //delay(1000); // 1 sec
  //return;

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
    Serial.print("\"A"); // A = analog
    Serial.print(pin);
    Serial.print("\": ");
    if (pin == 0) { // for TMP36
      // converting that reading to voltage, for 3.3v arduino use 3.3
      temp = analogRead(pin); // dummy read to switch channel
      delayMicroseconds(100); // wait for s/h
      temp = analogRead(pin) * ioref;
      temp /= 1024.0;
      temp = (temp - 0.5) * 100; // converting to C, 10 mv per degree C wit 500 mV offset
      temp = (temp * 9.0 / 5.0) + 32.0; // now convert to Fahrenheit
      Serial.print(temp);
    } else {
      Serial.print(analogRead(pin));
    }
    if (pin < analogPins - 1) {
      Serial.print(", ");
    }
  }

  if (1) {
    Serial.print(", \"RTCTEMP\": ");
    Serial.print(rtc.getTemperature());
  }

  Serial.print(", ");
  // D0, D1 serial port so start at 2
  for (int pin = 2; pin < digitalPins; pin++) {
    Serial.print("\"D");
    Serial.print(pin);
    Serial.print("\": ");
    Serial.print(digitalRead(pin));
    if (pin < digitalPins - 1) {
      Serial.print(", ");
    }
  }
  
  Serial.println("}");

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
