
// Script to read A & D pins, timestamp the date, and send out the serial port
// It is a mix of code I wrote and example code I found
// Mashup by Walter Spurgiasz 2016


/*
 The SD card code below was not robust for me so I used a serial logger instead (OpenLog)
 You may have beter luck...

 SD card datalogger
 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe
 This example code is in the public domain.
 */

#include <SPI.h>
//#include <SD.h>
#include "Wire.h"
#define DS3231_I2C_ADDRESS 0x68

// RTC code inspired from:
// http://tronixstuff.com/2014/12/01/tutorial-using-ds1307-and-ds3231-real-time-clock-modules-with-arduino/
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
}
void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}
void readDS3231time(byte *second, byte *minute, byte *hour, byte *dayOfWeek, byte *dayOfMonth, byte *month, byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  // request seven bytes of data from DS3231 starting from register 00h
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

void jsonTimestamp()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;

  Serial.print("{");
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  Serial.print("\"Time\": \"20");
  Serial.print(year, DEC);
  Serial.print("-");
  Serial.print(month, DEC);
  Serial.print("-");
  Serial.print(dayOfMonth, DEC);
  Serial.print("T");
  Serial.print(hour, DEC);
  Serial.print(":");
  if (minute<10)
  {
    Serial.print("0");
  }
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second<10)
  {
    Serial.print("0");
  }
  Serial.print(second, DEC);
  Serial.print("\", ");
}

//const int chipSelect = 4;

void setup() {
  Wire.begin();
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  // Uncomment setDS3231time to set the RTC. Recomment to let clock run.
  // DS3231 seconds, minutes, hours, day(1=Sunday, 7=Saturday), date, month, year
  //setDS3231time(30,13,22,3,23,8,16);

  // output version info so can see in the field
  // Used info on http://forum.arduino.cc/index.php?topic=158014.0
  jsonTimestamp();
  Serial.print("\"Compiled:\": \""__DATE__ ", " __TIME__ ", " __VERSION__"\"");
  Serial.println("}");

  // The SD card code below was not robust for so I used a serial logger instead (OpenLog)
  // You may have beter luck...
  //Serial.print("Initializing SD card...");

  //pinMode(10, OUTPUT); // change this to 53 on a mega  // don't follow this!!
  //digitalWrite(10, HIGH); // Add this line

  // see if the card is present and can be initialized:
  //if (!SD.begin(chipSelect)) {
  //  Serial.println("Card failed, or not present");
    // don't do anything more:
  //  return;
  //}
  //Serial.println("card initialized.");
}

void loop() {
  // make a string for assembling the data to log:
  //String dataString = "";
  int analogPins = 4;
  int digitalPins = 14;

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

  jsonTimestamp();

  for (int pin = 0; pin < analogPins; pin++) {
    Serial.print("\"A");
    Serial.print(pin);
    Serial.print("\": ");
    if (pin == 0) { // for TMP36
      // converting that reading to voltage, for 3.3v arduino use 3.3
      float temp = analogRead(pin) * 5.0;
      temp /= 1024.0;
      temp = (temp - 0.5) * 100; // converting to C, 10 mv per degree C wit 500 mV offset
      temp = (temp * 9.0 / 5.0) + 32.0; // now convert to Fahrenheit
      Serial.print(temp);
    } else {
      Serial.print(analogRead(pin));
      //int sensor = analogRead(pin);
      //dataString += String(sensor);
    }
    if (pin < analogPins - 1) {
      Serial.print(", ");
      //dataString += ",";
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
  // print to the serial port too:
  //Serial.println(dataString);

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  //File dataFile = SD.open("datalog1.txt", FILE_WRITE);

  // if the file is available, write to it:
  //if (dataFile) {
  //  dataFile.println(dataString);
  //  dataFile.close();
  //}
  // if the file isn't open, pop up an error:
  //else {
  //  Serial.println("error opening datalog.txt");
  //}

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
