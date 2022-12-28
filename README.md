# arduino-rtc-json-data-logger

This is the code/sketch to load on the board a
[GenuLog Data Acquisition, Signal Processing Shield](https://www.tindie.com/products/conciseusa/genulog-data-acquisition-signal-processing-shield/)
has been mounted to.

Even if you don't have the shield, you can add a RTC and an LCD to an Arduino and make you own data logger.

`git clone https://github.com/conciseusa/arduino-rtc-json-data-logger.git`

On Linux, if you get the<br>
avrdude: ser_open(): can't open device "/dev/ttyACM0": Permission denied<br>
error in the ArduinoIDE. Adding your user to the dialout group will typically fix the issue.<br>
`sudo adduser YourUserName dialout`<br>
logout and login

To confirm your serial port is in the dialout group<br>
`ls -l /dev/ttyACM*`<br>
To display the groups you are a member of<br>
`groups`<br>
You may need to adjust ttyACM if your system uses a differnt serial port name. It will typically only be present when the Arduino is connnected.<br>
