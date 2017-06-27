/*
   Heater.ino
*/

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2  // Data wire is plugged into port 2 on the Arduino

OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.

// Variables
// =========
// temperature
float SET_TEMP = 18;
# define HYSTERESIS 2
// TRIAC_PIN
# define RED_PIN 5
# define GREEN_PIN 6
# define BLUE_PIN 7
/*
   The setup function. We only start the sensors here
*/
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  //Serial.println("Dallas Temperature IC Control Library Demo");
  Serial.println( F("Heater.ino by <r0kdu5t@theatrix.org.nz"));

  // Example, not tested. See: http://forum.arduino.cc/index.php?topic=158014.0

  // Combined string in RAM
  //Serial.println( "Compiled: " __DATE__ ", " __TIME__ ", " __VERSION__);

  // The string in Flash
  Serial.print( F("Compiled: "));
  Serial.print( F(__DATE__));
  Serial.print( F(", "));
  Serial.print( F(__TIME__));
  //Serial.print( F(", "));
  //Serial.println( F(__FILE__));
  delay(2000);
  // Setup the output status LEDs.
  byte i;
  for ( i = 5; i < 7; i++) {
    pinMode(i, OUTPUT);
    //sensors[sensorId].status_output
    digitalWrite(i, LOW); // Turn 'Off' LED.
  }

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
  //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  Serial.print("Temperature for the device 1 (index 0) is: ");
  //Serial.println(sensors.getTempCByIndex(0));
  float temperature = sensors.getTempCByIndex(0); // Get value from sensor
  Serial.println(temperature, DEC);

  // Check if sensed value is less than set value minus HYSTERESIS
  if (temperature <= (SET_TEMP - HYSTERESIS)) {
    // Do what? Turn On Output
    digitalWrite(RED_PIN, HIGH);
  }
  // Check if sensed value is more than set value plus HYSTERESIS
  if (temperature >= (SET_TEMP + HYSTERESIS)) {
    // Do what? Turn Off Output
    digitalWrite(BLUE_PIN, HIGH);
  }
  digitalWrite(GREEN_PIN, HIGH);
  delay(1000);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);

}
