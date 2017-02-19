# lupo-gti-tripcomputer-kw1281
A tripcomputer for VW Lupo GTI using kw1281 protocoll

*** !!! WARNING: code is quick & dirty - use at own risk - uploaded only for learning purposes - DONT DO IT AT HOME !!! ***

In my 2001 Lupo, i want a display that shows the time and distance driven since engine start. This car is so old that there is no CAN BUS signal at the OBD diagnostic connector, but only the old KW1281 serial communication protocol. Some googling around bring the following sources:

* https://www.blafusel.de/obd/obd.html  (info to OBD2 and especially kw1281)
* http://grauonline.de/wordpress/?p=74  (code base, only slightly modified for my car, see comments on his blog also)
* http://www.avrfreaks.net/forum/alphanumeric-lcd-hd44780-big-font-big-digits-generator-excel-sheet (nice big numbers that you can read at 200Km/H ;-) )

Study the appropriate label file for your ecu (SIMOS3 036906033 for AVY engine in my case) to find your values of interest.
In my case the values of interest are (sketch line from 700 - 749):
* ecu measurement block 4 value 2 (supplyVoltage), value 3 (coolantTemp)
* ecu measurement block 5 value 2 (engineLoad), value 3 (vehicleSpeed)
* ecu measurement block 134 value 1 (oilTemp)
* cant get trip distance from my ecu so i have to calculate it manually (sketch line 983 - 992)

What does it do ?
The arduino talks countinuously to the ecu and grab the values. One loop needs ~600 to 800ms, so in combination with the slow liquid crystal display the update rate looks a litte bit fluent - i like it :-) Short video here: https://www.youtube.com/watch?v=IpqHjx0bSsI

Also the maximus speed is saved to arduino eeprom to survive loss of power. All the other values are zeroed by power loss.

Hardware used:
* Arduino UNO
* USB KKL adapter ‘AutoDia K409 Profi USB‘ like recommendet by Alexander (see link2 above)
* cheap 4x40 hd44780 lcd display from ebay, with green backlight and inverted

Hardware mods to be done:
* open usb kkl cable and cut rx und tx line from FTDI chip (watch datasheet to your FTDI chip to find the correct lines)
* solder vcc, gnd, rx, tx from level shifter (kkl cable) to definded arduino pins (see code file)
* solder lcd display pins to defined arduino pins (see code file)

TODO:
* the OBD connector from the cheap cable has connection problems from time to time, or the OBD connector of my car is not in best condtion
* build housing for the hardware, at this point it looks like ugly hotglue prototype 
* maybe nice big oled display in future, but for now hd44780 does the trick ;-)
