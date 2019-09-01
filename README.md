# Afterthought
This project turned out to be bigger than I first anticipated. My main aim is still to make a competent battery charger, but a few steps needs to be taken along the way. First, reverse engineering and documentation. Currently most of the documentation is in this readme. I have started a KiCAD project to document the circuits of the CPU board in doc/kicad/. High resolution images of the CPU board is available here and in doc/images/. This can be used for reverse engineering the circuitry. 

The i2c reverse engineering is done with an ESP8266 using various methods and experiments. Code is in `src/dps_re.cpp`. The charger project is split up in two approaches. An analogue charger that is based on the ESP32 interfacing through `12VFB` and `LOAD_SHARE` etc. A digital one based on i2c interfacing using an ESP8266. The ESP32 seems to have issues with i2c stability, and the ESP8266 have limited analogue interfaces.


# Background
The DPS-1200FB is a very powerful and compact powersupply. It can supply up to 100A adjustable between 11.8v-12.8v. This is perfect for supplying high currents to 12v equipment like your boat fridge etc. It won't, however, charge any 12v lead-acid or lithium batteries. You would need 14.4v (lead acid) or 14.6v (lithium) to get a full charge. There are some mods floating around on the interwebs on how to increase voltage, how to series connect two PSUs and how to amputate the Over Voltage Protection (OVP). 

# Goal
The goal with this project is to use an Arduino to control the PSU. Ideally you would be able to control output current, voltage, fan speed and on/off. That would allow us to build a powerful lab power supply, a custom battery charger, a flexible mining PSU etc etc. Should I not be able to reach the goal of controlling the PSU digitally through its data port, workarounds might be available.

# How
The PSU has an i2c port through which it reports status and takes commands. It contains two devices that responds on the i2c bus, the EEPROM and the PIC micro processor. Their addresses are set by the server by pulling up or down certain pins in the end connector. The EEPROM can be read and written using this bus. The MCU responds to commands sent to it. raplin (https://github.com/raplin/) has shown that it accepts fan speed commands, and reports fan speed back if queried. He has also documented several other commands and data.

## Connection
The PSU has a double sided edge connector. The majority of which is taken up by the high current output tabs. There are also 6 smaller tabs on each side. These have various functions in turning on the PSU and selecting its i2c address.

![Connector-bottom](https://github.com/slundell/dps_charger/raw/master/doc/images/connector-bottom.jpg)
The bottom part of the connector has pins 1 to 32.
![Connector-bottom](https://github.com/slundell/dps_charger/raw/master/doc/images/connector-top.jpg)
 The top one has 33 to 64.

```
Pinout:
1: +12V Power out
14: GND Power out
27, 28, 29: I2C slave address selection pins. 
30: GND (for I2C and I2C slave address selection, I guess)
31: I2C SCL
32: I2C SDA
33: ENABLE#
34: LOAD SHARE
35: STATUS
36: PRESENT
37: +12V stand-by
38: PSALARM
```

### 34: Load share
The load share both both carry and read information. The output voltage is proportional to the output current. It reads information as other PSUs in parallell with it can pull this voltage up or down to signal how much help they want with power delivery. This can be utilized to regulate the current. The load share pin manipulates the output voltage to regulate current. It knows about the OVP limit as it does not increase voltage over that level.

### 36: Status
3.2V means A-OK, 0V means error.

For a more in-depth look at the different pins' properties: (http://colintd.blogspot.com/2016/10/hacking-hp-common-slot-power-supplies.html)


These are default high, but can be pulled low by connecting them to GND.

<pinout/connection goes here>

## EEPROM
First the EEPROM that stores various permanent data such as manufacturer and serial number. This is of limited use to us, but can be used as "vanity data" to display which kind of PSU we are connected to. There may be some settings data, or alarm logs here, I have yet to check that. The MCU has it's own EEPROM to store various values as well.

The EEPROM is writable from the i2c bus.


```
0x00: 0xFE 0x00 0x00 0x01 0x05 0x0E 0x00 0xEB ........
0x08: 0x01 0x04 0x19 0x08 0x06 0x6C 0xC0 0xC0 .....l..
0x10: 0xC0 0xCA 0x34 0x34 0x31 0x38 0x33 0x30 ..441830    // 0x12 - 0x1B: SPN no
0x18: 0x2D 0x30 0x30 0x31 0xC8 0x31 0x30 0x2F -001.10/    // 0x1D - 0x24: Date, probably date of manufacturing
0x20: 0x31 0x30 0x2F 0x30 0x38 0xC1 0x00 0x5B 10/08..[
0x28: 0x01 0x09 0x19 0xC5 0x44 0x45 0x4C 0x54 ....DELT    // 0x2C - 0x30: Manufacturer
0x30: 0x41 0xDA 0x48 0x50 0x20 0x50 0x52 0x4F A.HP PRO    // 0x32 - 0x4B: PSU Name 
0x38: 0x4C 0x49 0x41 0x4E 0x54 0x20 0x53 0x45 LIANT SE
0x40: 0x52 0x56 0x45 0x52 0x20 0x50 0x53 0x20 RVER PS
0x48: 0x20 0x20 0x20 0x20 0xCA 0x34 0x33 0x37     .437    // 0x4D - 0x56: Unknown. Some kind of part number
0x50: 0x35 0x37 0x32 0x2D 0x42 0x32 0x31 0xC2 572-B21.
0x58: 0x30 0x31 0xCE 0x35 0x41 0x4D 0x4A 0x51 01.5AMJQ    // 0x5B - 0x68: PSU CT no
0x60: 0x30 0x44 0x34 0x44 0x58 0x4F 0x33 0x4C 0D4DXO3L
0x68: 0x55 0x00 0x00 0xC1 0x00 0x00 0x00 0x0A U.......
0x70: 0x00 0x02 0x18 0x12 0xD4 0xB0 0x04 0xA0 ........
0x78: 0x05 0x1E 0x05 0x28 0x23 0x90 0x33 0x50 ...(#.3P
0x80: 0x46 0x20 0x67 0x2F 0x3F 0x0A 0x1A 0xA0 F g/?...
0x88: 0x15 0x00 0x00 0x00 0x00 0x01 0x02 0x0D ........
0x90: 0xB2 0x3E 0x01 0xCE 0x04 0x74 0x04 0xEC .>...t..
0x98: 0x04 0x78 0x00 0x64 0x00 0x10 0x27 0x01 .x.d..'.
0xA0: 0x02 0x0D 0xEF 0x01 0x82 0xB0 0x04 0x38 .......8
0xA8: 0x04 0x28 0x05 0x78 0x00 0x00 0x00 0xFA .(.x....
0xB0: 0x00 0xD0 0x82 0x12 0xE3 0xB9 0x0B 0x00 ........
0xB8: 0x00 0x03 0x20 0x03 0xC0 0x13 0x01 0x80 .. .....
0xC0: 0x00 0x00 0x00 0x00 0x00 0x00 0x50 0x48 ......PH
0xC8: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xD0: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xD8: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xE0: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xE8: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xF0: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xF8: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
```
EEPROM of another DPS-1200FB of a slightly older version. Pulled straight out of server before dumping this data. Suspecting some read errors, e.g. at 0x4E - should probably be a "3".

```
0x00: 0x01 0x00 0x00 0x01 0x05 0x0E 0x00 0xEB ........
0x08: 0x01 0x04 0x19 0xEE 0x37 0x61 0xC0 0xC0 ....7a..
0x10: 0xC0 0xCA 0x34 0x34 0x31 0x38 0x33 0x30 ..441830
0x18: 0x2D 0x30 0x30 0x31 0xC8 0x30 0x35 0x2F -001.05/
0x20: 0x30 0x34 0x2F 0x30 0x37 0xC1 0x00 0x49 04/07..I
0x28: 0x01 0x09 0x19 0xC5 0x44 0x45 0x4C 0x54 ....DELT
0x30: 0x41 0xDA 0x48 0x50 0x20 0x50 0x52 0x4F A.HP PRO
0x38: 0x4C 0x49 0x41 0x4E 0x54 0x20 0x53 0x45 LIANT SE
0x40: 0x52 0x56 0x45 0x52 0x20 0x50 0x53 0x20 RVER PS
0x48: 0x20 0x20 0x20 0x20 0xCA 0x34 0x33 0x37     .4.7
0x50: 0x35 0x37 0x32 0x2D 0x42 0x32 0x31 0xC2 572-B21.
0x58: 0x30 0x31 0xCE 0x35 0x33 0x31 0x32 0x39 01.53129
0x60: 0x30 0x41 0x34 0x44 0x56 0x4E 0x37 0x4C 0A4DVN7L
0x68: 0x59 0x00 0x00 0xC1 0x00 0x00 0x00 0x62 Y......b
0x70: 0x00 0x02 0x18 0x12 0xD4 0xB0 0x04 0xA0 ........
0x78: 0x05 0x1E 0x05 0x28 0x23 0x90 0x33 0x50 ...(#.3P
0x80: 0x46 0x20 0x67 0x2F 0x3F 0x0A 0x1A 0xA0 F g/?...
0x88: 0x15 0x00 0x00 0x00 0x00 0x01 0x02 0x0D ........
0x90: 0xB2 0x3E 0x01 0xCE 0x04 0x74 0x04 0xEC .>...t..
0x98: 0x04 0x78 0x00 0x64 0x00 0x10 0x27 0x01 .x.d..'.
0xA0: 0x02 0x0D 0xEF 0x01 0x82 0xB0 0x04 0x38 .......8
0xA8: 0x04 0x28 0x05 0x78 0x00 0x00 0x00 0xFA .(.x....
0xB0: 0x00 0xD0 0x82 0x12 0xE3 0xB9 0x0B 0x00 ........
0xB8: 0x00 0x03 0x20 0x03 0xC0 0x13 0x01 0x80 .. .....
0xC0: 0x00 0x00 0x00 0x00 0x00 0x00 0x50 0x48 ......PH
0xC8: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xD0: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xD8: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xE0: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xE8: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xF0: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
0xF8: 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 ........
```


# Commands
The MCU responds to i2c commands/registers. The challenge is to figure out which commands do what. According to http://colintd.blogspot.com/2016/10/hacking-hp-common-slot-power-supplies.html the PSU should be communicating according to the PMbus protocol. Here are some specs: https://www.components-mart.com/datasheets/f8/MAX16064ETX-T.pdf. One more detailed here: https://power.murata.com/pub/data/apnotes/acan-51.pdf. However, the register numbers and scaling factors does not correspond to my findings on how the DPS-1200FB responds. The DPS-1200FB most probably communicates over a propritary protocol.

# Hardware
## Main board
![Main Board](https://github.com/slundell/dps_charger/raw/master/doc/images/main_board_backside_older.png)

## Control board
![CPU Board](https://github.com/slundell/dps_charger/raw/master/doc/images/cpu-board-annotated.png)
![CPU Board](https://github.com/slundell/dps_charger/raw/master/doc/images/cpu_board_topside_older.png)
![CPU Board](https://github.com/slundell/dps_charger/raw/master/doc/images/cpu_board_backside_older.png)

## Sub-board
![subboard](https://github.com/slundell/dps_charger/raw/master/doc/images/sub_board_older.png)
![subboard](https://github.com/slundell/dps_charger/raw/master/doc/images/sub_board_newer.png)


# Modifications

## 12VFB tap
The PSU control board gets an 12v feedback signal from the main board. This hoovers around 2.500v. I guess 2.500v means "just right" and <2.500v means "too low" and >2.500v means "too high". This pin is connected to a few components on the board. A few different resistors, probably acting as voltage dividers for the different "consumers" of the feedback signal. One path is through the voltage adjustment potentiometer through to the MCU.

By adjusting the voltage on this pin, the output voltage can be adjusted. I connected a potentiometer between 12VSB, GND and 12VFB (wiper to 12VFB). This allowed me to adjust the voltage from 0.2V up to around 13.7V where the OVP kicks in. Interestingly, UVP did not stop the PSU. I had a halogen bulb connected and measured the voltage on the output terminals to make sure the voltage would stay even under a light load. The PSU I2C reports the actual voltage!

![12VFB Tap](https://github.com/slundell/dps_charger/raw/master/doc/images/12FB-tap.jpg)
Not only did I learn how to adjust the voltage. I also learned that lead-free solder is almost unusable :)
![Under voltage](https://github.com/slundell/dps_charger/raw/master/doc/images/voltage_range.png)

I then tried 16p1s of GBS 20Ah LifePo4 batteries to see if we could pull some amps. According to my meter it was well above 100A. The i2c data shows roughly half of that. Possibly a scaling issue.
![100+A](https://github.com/slundell/dps_charger/raw/master/doc/images/loaded_at_low_voltage.jpg)

## Manipulating the ADC Reference voltage
As far as I know, this is the only known modification to increase the OVP. It is simple but intrusive. By adding a small voltage to the PIC ADCREF-pin (5) to 2.65V increases the OVP from 13.7V to 14.5V, 2.84V to 15.4V). This hack has some drawbacks, as it will change the readings of other analogue signals such as temperature etc. The i2c bus will report wrong voltage values.

# Acknowledgements
## raplin
A big chunk of the information here is from other sources. Richard Aplins github repo documents his reverse engineering of the PSU (https://github.com/raplin/DPS-1200FB). He has disassembled the actual PIC16F source code on the MCU! Through this he was able to figure out the checksum (PEC) calculations and more. He has also figured out the meaning and scale of many of the command registers. 

## colindt
Colin has documented a lot if information regarding the hardware side of things. He figured out the pin-out and other things.
(http://colintd.blogspot.com/)

## brettbeauregard
Brett has made a PID controller library and explains it in great depth in a series of articles. I this to control voltage and current.
http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ https://github.com/br3ttb/Arduino-PID-Library

## Philipp Seidel
Excellent instruction on how to get the PSU started and how to series several PSUs for higher voltage. https://blog.seidel-philipp.de/hp-dps-1200fb-power-supply-hack-for-charging-lipos/

## KANATLI
Came up with the ADC REF hack.
