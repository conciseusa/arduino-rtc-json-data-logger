
// Script to read A & D pins, timestamp the data, and send out the serial port in a json packet
// A serial logger (OpenLog) can be used to record the data, or a RPi can be used as a gateway to the internet
// A LCD shows the last 2 readings and the high and low for today and yesterday
// It is a mix of code I wrote and example code I found
// Mashup by Walter Spurgiasz 2020
// Code inspired from:
// https://lastminuteengineers.com/ds3231-rtc-arduino-tutorial/
// https://www.makerguides.com/character-i2c-lcd-arduino-tutorial/

#include "Wire.h"
#include "RTClib.h"
#include <LiquidCrystal_I2C.h> // https://www.arduinolibraries.info/libraries/liquid-crystal-i2-c - Frank de Brabander

RTC_DS3231 rtc; // DS3231_I2C_ADDRESS 0x68
// Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered)
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4); // Change to (0x27,16,2) for 16x2 LCD.
int analogPins = 4; // pins on an Uno
int digitalPins = 14; // pins on an Uno
float temp;
char thour = 99; // save temp time
char tmin = 99;
char tsec = 99;
char tday = 99; // save day so know when to reset high and low
float thtemp = -999; // set so any normal reading above
float tltemp = 999; // set so any normal reading below
float yhtemp = -999;
float yltemp = 999;

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

void jsonTimestamp()
{
  Serial.print("{\"Time\": \""); // "Time": "2020-5-14T15:00:06"
  timeStamp(13);
  Serial.print("\", ");
}

void setup() {
  //Wire.begin(); // seems we do not need this

  // from http://forum.arduino.cc/index.php?topic=113656.0
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
    // Code in here will only be compiled if an Arduino Uno (or older) is used.
  #endif
  
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // Code in here will only be compiled if an Arduino Mega is used.
    analogPins = 16;
    digitalPins = 54;
  #endif

  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
    // Code in here will only be compiled if an Arduino Leonardo is used.
  #endif 

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println(""); // restart with a newline
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // line below can be used to set a time module
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // keep between rtc check and clock set check

  if (rtc.lostPower()) {
    Serial.println("{\"Error\": \"RTC lost power, time needs to be set,\"}");
    while (1);
  }

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  jsonTimestamp(); // start up message time
  // output compile time / version info so can see in the field
  // Used info on http://forum.arduino.cc/index.php?topic=158014.0
  Serial.print("\"Compiled\": \""__DATE__ ", " __TIME__ ", " __VERSION__"\"");
  Serial.println("}");

  lcd.init();
  lcd.backlight(); // lcd.noBacklight();

} // end setup()

void loop() {
  jsonTimestamp();

  // display prev data and time
  lcd.setCursor(0, 1);
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
  lcd.print(temp);

  DateTime now = rtc.now();
  thour = now.hour(); // save temp time
  tmin = now.minute();
  tsec = now.second();
  tday = now.day();

  for (int pin = 0; pin < analogPins; pin++) {
    Serial.print("\"A"); // A = analog
    Serial.print(pin);
    Serial.print("\": ");
    if (pin == 0) { // for TMP36
      // converting that reading to voltage, for 3.3v arduino use 3.3
      temp = analogRead(pin) * 5.0;
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

  Serial.print(", ");
  // D0, D1 srial port so start at 2
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

  lcd.setCursor(0, 0); // Set the cursor on the first column and first row. X, Y
  timeStamp(6);
  lcd.print(" Temp ");
  lcd.print(temp);

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
  lcd.print(" H");
  lcd.print(thtemp);
  lcd.print(" L");
  lcd.print(tltemp);

  // if new day reset high and low trackers
  if (tday != now.day()) {
    yhtemp = thtemp;
    yltemp = tltemp;
    thtemp = temp;
    tltemp = temp;
  }

  lcd.setCursor(0, 3);
  if (yhtemp == -999 && yltemp == 999) {
    lcd.print("Yesterday: No Data");
  } else {
    lcd.print("Yeday H");
    lcd.print(yhtemp);
    lcd.print(" L");
    lcd.print(yltemp);
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
