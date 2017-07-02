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
 2 | digitalPin | PushButton (INT0)
 5 | digitalPin | DS18B20 temperature sensor(s)
 6 | digitalPin | SSR_PIN
 10 | digitalPin | Ethernet / SS~
 11 | digitalPin | Ethernet / MOSI~
 12 | digitalPin | Ethernet / MISO
 13 | digitalPin | Ethernet / SCK
 A0 | analogPin | CSLT Hall Effect current sensor?
 15 | digitalPin | LED RED_PIN [analogPin A1]
 16 | digitalPin | LED GREEN_PIN [analogPin A2] 
 17 | digitalPin | LED BLUE_PIN [analogPin A3]

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
