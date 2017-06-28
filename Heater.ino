/*
   Heater.ino
*/

/* Network config */
#define ENABLE_DHCP                 true   // true/false
#define ENABLE_MAC_ADDRESS_ROM      true   // true/false
#define MAC_I2C_ADDRESS             0x50   // Microchip 24AA125E48 I2C ROM address
static uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // Set if no MAC ROM
static uint8_t ip[] = { 192, 168, 1, 35 }; // Use if DHCP disabled
// Include the libraries we need
#include <SPI.h>
#include "Ethernet.h"
#include "Wire.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2  // Data wire is plugged into port 2 on the Arduino

OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.

// Variables
// =========
// temperature
byte SET_TEMP = 18;
# define HYSTERESIS 2
// TRIAC_PIN
# define RED_PIN 5
# define GREEN_PIN 6
# define BLUE_PIN 7

// Initialize the Ethernet client library
EthernetClient client;
/*
   The setup function. We only start the sensors here
*/
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  //Serial.println("Dallas Temperature IC Control Library Demo");
  Serial.println( F("Heater.ino by <r0kdu5t@theatrix.org.nz>"));

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
  /*
     DEBUG - Stuff
  */
  // What are my variable values
  Serial.println();
  Serial.println(SET_TEMP, DEC);
  Serial.println(HYSTERESIS, DEC);
  Serial.println();
  delay(2000);

  if ( ENABLE_MAC_ADDRESS_ROM == true )
  {
    Serial.print(F("Getting MAC address from ROM: "));
    mac[0] = readRegister(0xFA);
    mac[1] = readRegister(0xFB);
    mac[2] = readRegister(0xFC);
    mac[3] = readRegister(0xFD);
    mac[4] = readRegister(0xFE);
    mac[5] = readRegister(0xFF);
  } else {
    Serial.print(F("Using static MAC address: "));
  }
  // Print the IP address
  char tmpBuf[17];
  sprintf(tmpBuf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(tmpBuf);

  // setup the Ethernet library to talk to the Wiznet board
  if ( ENABLE_DHCP == true )
  {
    Ethernet.begin(mac);      // Use DHCP
  } else {
    Ethernet.begin(mac, ip);  // Use static address defined above
  }

  // Print IP address:
  Serial.print(F("My IP: http://"));
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    if ( thisByte < 3 )
    {
      Serial.print(".");
    }
  }
  Serial.println();
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
  //Serial.print((int)temperature);

  // Check if sensed value is less than set value minus HYSTERESIS
  if ((int)temperature < (SET_TEMP - HYSTERESIS)) {
    // Do what? Turn On Output
    digitalWrite(RED_PIN, HIGH);
  } else {
    digitalWrite(RED_PIN, LOW);
  }
  // Check if sensed value is more than set value plus HYSTERESIS
  if ((int)temperature > (SET_TEMP + HYSTERESIS)) {
    // Do what? Turn Off Output
    digitalWrite(GREEN_PIN, HIGH);
  } else {
    digitalWrite(GREEN_PIN, LOW);
  }
  //digitalWrite(GREEN_PIN, HIGH);
  delay(1000);
  //digitalWrite(RED_PIN, LOW);
  //digitalWrite(GREEN_PIN, LOW);
  //digitalWrite(BLUE_PIN, LOW);

}

/**
 * Required to read the MAC address ROM
 */
byte readRegister(byte r)
{
  unsigned char v;
  Wire.beginTransmission(MAC_I2C_ADDRESS);
  Wire.write(r);  // Register to read
  Wire.endTransmission();

  Wire.requestFrom(MAC_I2C_ADDRESS, 1); // Read a byte
  while (!Wire.available())
  {
    // Wait
  }
  v = Wire.read();
  return v;
}

