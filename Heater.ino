/*
   Heater.ino
*/

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2  // Data wire is plugged into port 2 on the Arduino

OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.

/*
   The setup function. We only start the sensors here
*/
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  //Serial.println("Dallas Temperature IC Control Library Demo");
  Serial.println( F("<r0kdu5t@theatrix.org.nz"));

  // Example, not tested. See: http://forum.arduino.cc/index.php?topic=158014.0

  // Combined string in RAM
  //Serial.println( "Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);

  // The string in Flash
  Serial.print( F("Compiled: "));
  Serial.print( F(__DATE__));
  Serial.print( F(", "));
  Serial.print( F(__TIME__));
  Serial.print( F(", "));
  Serial.println( F(__FILE__));
  delay(2000);

  // Start up the library
  sensors.begin();
}

/*
   Main function, get and show the temperature
*/
void loop(void)
{
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(sensors.getTempCByIndex(0));
  delay(1000);
/*
// Variables
// =========
// temperature
// setTemp
// HYSTERESIS
// TRIAC_PIN

// Code
// Check if sensed value is less than set value minus HYSTERESIS 
if (temperature < (setTemp - HYSTERESIS)) {
  // Do what? Turn On Output
  digitalWrite(TRIAC_PIN, HIGH);
}
// Check if sensed value is more than set value plus HYSTERESIS 
if (temperature > (setTemp + HYSTERESIS)) {
  // Do what? Turn Off Output
  digitalWrite(TRIAC_PIN, LOW);
}
 */
}
