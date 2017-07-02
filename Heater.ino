/*
   Heater.ino
*/
/*--------------------------- Configuration ------------------------------*/
/* Network config */
#define ENABLE_DHCP                 true   // true/false
//
static uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // Set if no MAC ROM
static uint8_t ip[] = { 192, 168, 1, 35 }; // Use if DHCP disabled

// Include the libraries we need
#include <SPI.h>
#include "Ethernet.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2  // Data wire is plugged into port 2 on the Arduino

OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.

// Variables
// =========
// temperature
boolean REQ_HEAT = false;
byte SET_TEMP = 18;
# define HYSTERESIS 2
# define SSR_PIN 4
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

  // Info String in Flash
  Serial.print( F("Compiled: "));
  Serial.print( F(__DATE__));
  Serial.print( F(", "));
  Serial.print( F(__TIME__));
  //Serial.print( F(", "));
  //Serial.println( F(__FILE__));
  delay(2000);
  
  // Setup SSR control pin.
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW); // Turn 'Off' SSR.

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
    // Turn On Output
    REQ_HEAT = true;
    digitalWrite(RED_PIN, HIGH);
  } else {
    digitalWrite(RED_PIN, LOW);
  }
  // Check if sensed value is more than set value plus HYSTERESIS
  if ((int)temperature > (SET_TEMP + HYSTERESIS)) {
    // Turn Off Output
    REQ_HEAT = false;
    digitalWrite(GREEN_PIN, HIGH);
  } else {
    digitalWrite(GREEN_PIN, LOW);
  }
  //digitalWrite(GREEN_PIN, HIGH);
  
  //digitalWrite(RED_PIN, LOW);
  //digitalWrite(GREEN_PIN, LOW);
  //digitalWrite(BLUE_PIN, LOW);
  if ( REQ_HEAT ) {
    SSR_CTRL(true);
  }
  delay(1000);
}
/*
 * SSR control routine.
 */
void SSR_CTRL(boolean HEAT_CTRL ) {
  if ( HEAT_CTRL ) {
    digitalWrite(SSR_PIN, HIGH);
    //
  } else {
    digitalWrite(SSR_PIN, LOW);
  }
}

