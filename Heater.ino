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

#define ONE_WIRE_BUS 5  // Data wire is plugged into port 2 on the Arduino
OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.

/*--------------------------- Variables ------------------------------*/
// temperature
boolean REQ_HEAT = false; // REQUEST HEATING!
boolean OVRDE = false;  // OVER_RIDE or MANUAL
volatile boolean FLAG = false;
unsigned long last_button_time = 0;
byte SET_TEMP = 18;
# define HYSTERESIS 2
# define SSR_PIN 6
# define RED_PIN 15 // analogPin A1
# define GREEN_PIN 16 // analogPin A2
# define BLUE_PIN 17  // analogPin A3

// Initialize the Ethernet client library
EthernetClient client;

// Interrupt Service Routine (ISR)
void pButton() {
  unsigned long button_time = millis();
  //check to see if increment() was called in the last 250 milliseconds
  if (button_time - last_button_time > 250)
  {
    FLAG = true;
    last_button_time = button_time;
  }
}  // end of isr

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

  /*
     Enable interrupt 0 which uses pin 2
     jump to the pButton function on rising edge
  */
  attachInterrupt(digitalPinToInterrupt(2), pButton, RISING);

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
  if (FLAG) {
    // interrupt has occurred
    // REQ_HEAT = !REQ_HEAT
    OVRDE = !OVRDE;
  }
  if ( REQ_HEAT || OVRDE ) {
    SSR_CTRL(true);
  }
  delay(1000);
} // End of loop()

/*
   SSR control routine.
*/
void SSR_CTRL(boolean HEAT_CTRL ) {
  if ( HEAT_CTRL ) {
    digitalWrite(SSR_PIN, HIGH);
    //
  } else {
    digitalWrite(SSR_PIN, LOW);
  }
}

/*
   Playground for testing.
   Refer to: https://arduino.stackexchange.com/questions/20994/why-the-need-to-use-the-volatile-keyword-on-global-variables-when-handling-inter
  volatile boolean flag;

  // Interrupt Service Routine (ISR)
  void isr ()
  {
  flag = true;
  }  // end of isr

  void setup ()
  {
  attachInterrupt (0, isr, CHANGE);  // attach interrupt handler
  }  // end of setup

  void loop ()
  {
  if (flag)
    {
    // interrupt has occurred
    }
  }  // end of loop
*/
