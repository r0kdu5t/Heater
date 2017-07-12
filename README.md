# Heater 

> Developed by Andrew J. Sands <r0kdu5t@theatrix.org.nz>

## Description

Reads the temperature from a DS18B20 sensor to control a relay/SSR to control attached device (i.e. heater), and ultimately to publish/subscribe via MQTT for controlled response automation.

## Hardware

* Arduinuno Uno plus Ethernet shield, Freetronics EtherTen or equivalent.

* DS18B20 temperature sensor (attached to pin 'sensor')

* SSR or relay (attached to pin 'output/relay/ssr')

* Momentary Push Button (attached to pin 'PB_int')

### *Board Connections*
Pin Num | Description | Used for
------- | ----------- | --------
 
 
 
 A0 | analogPin | CSLT Hall Effect current sensor?
 15 | digitalPin | LED RED_PIN [analogPin A1]
 16 | digitalPin | LED  [analogPin A2] 
 17 | digitalPin | LED  [analogPin A3]

### *Board Connection Table*

| | Function_> | |RX|TX|INT0|INT1|4|PWM|PWM|7|8|PWM|SS|MOSI|MISO| | | | |SDA|SCL|
|----|----------|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | Digital pins | |0|1|2|3|4|5|6|7|8|9|10|11|12|13|(14)|(15)|(16)|(17)|(18)|(19)|
| Analogue pins | | | | | | | | | | | |  |  |  |0|1|2|3|4|5|
| pButton | PushButton ||||X||||||||||||||||||
| oneWire | DS18B20 temperature sensor(s) ||0|1|2|3|4|X|6|7|8|9|10|11|12|13|(14)|(15)|(16)|(17)|(18)|(19)|
| SSR_PIN | Control output ||0|1|2|3|4|5|X|7|8|9|10|11|12|13|(14)|(15)|(16)|(17)|(18)|(19)|
| pushFb | LED in switch ||0|1|2|3|4|5|6|7|8|9|10|11|12|13|(14)|(15)|(16)|(17)|(18)|(19)|
| SS~ | Ethernet ||0|1|2|3|4|5|6|7|8|9|X|11|12|13|(14)|(15)|(16)|(17)|(18)|(19)|
| MOSI~ | Ethernet ||0|1|2|3|4|5|6|7|8|9|10|X|12|13|(14)|(15)|(16)|(17)|(18)|(19)|
| MISO | Ethernet ||0|1|2|3|4|5|6|7|8|9|10|11|X|13|(14)|(15)|(16)|(17)|(18)|(19)|
| SCK | Ethernet ||0|1|2|3|4|5|6|7|8|9|10|11|12|X|(14)|(15)|(16)|(17)|(18)|(19)|
| LOAD | CSLT Hall Effect current sensor? ||0|1|2|3|4|5|6|7|8|9|10|11|12|13|X|(15)|(16)|(17)|(18)|(19)|
| RED_PIN | Red LED ||0|1|2|3|4|5|6|7|8|9|10|11|12|13|(14)|X|(16)|(17)|(18)|(19)|
| GREEN_PIN | Green LED ||0|1|2|3|4|5|6|7|8|9|10|11|12|13|(14)|(15)|X|(17)|(18)|(19)|
| BLUE_PIN | Blue LED ||0|1|2|3|4|5|6|7|8|9|10|11|12|13|(14)|(15)|(16)|X|(18)|(19)|

<!--
	http://alvinalexander.com/technology/markdown-comments-syntax-not-in-generated-output
Name | Description | Digital pins |0|1|2|3|4|5|6|7|8|9|10|11|12|13|(14)|(15)|(16)|(17)|(18)|(19)
 -->
## Required Libraries

1. [DallasTemperature.h](url)

2. [PubSubClient.h](https://github.com/knolleary/pubsubclient)

## Installation

1. Change directory to Arduino's main directory


## License
> Copyright (C) 2017 Andrew J. Sands <r0kdu5t@theatrix.org.nz>

> This program is free software: you can redistribute it and/or modify
> it under the terms of the GNU General Public License as published by
> the Free Software Foundation, either version 3 of the License, or
> (at your option) any later version.

> This program is distributed in the hope that it will be useful,
> but WITHOUT ANY WARRANTY; without even the implied warranty of
> MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
> GNU General Public License for more details.

> You should have received a copy of the GNU General Public License
> along with this program.  If not, see <http://www.gnu.org/licenses/>.
