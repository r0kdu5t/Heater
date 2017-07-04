/*
   Heater.ino
*/
/*--------------------------- Configuration ------------------------------*/
/* Network config */
#define ENABLE_DHCP                 true   // true/false
#define ENABLE_MAC_ADDRESS_ROM      true   // true/false
#define MAC_I2C_ADDRESS             0x50   // Microchip 24AA125E48 I2C ROM address
static uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // Set if no MAC ROM
static uint8_t ip[] = { 192, 168, 1, 35 }; // Use if DHCP disabled
/* MQTT config */
//IPAddress broker(192, 168, 31, 65);       // Address of the MQTT broker - "spunkmeyer.theatrix.priv"
static uint8_t broker[] = { 192, 168, 31, 65 };
// Topic base for all comms from this device.
#define TOPICBASE "Home/Sleepy/"

// Include the libraries we need
#include <SPI.h>
#include "Ethernet.h"
#include "Wire.h"
#include <PubSubClient.h>
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

//Start MQTT goodness
void callbackMQTT(char* topic, byte* payload, unsigned int length) {
}

// Initialize the Ethernet client library
EthernetClient ethClient;
PubSubClient client( broker, 1883, callbackMQTT, ethClient); // MQTT object

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

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("SleepyClient", (char *)TOPICBASE "State", 1, 0, "DEAD")) {
      //if (client.connect("SleepyClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      Publish((char *)"State", (char *)"BOOTUP");
      // ... and resubscribe
      // Subscribe to enable bi-directional comms.
      client.subscribe(TOPICBASE "Config/#");  // Allow bootup config fetching using MQTT persist flag!
      //client.subscribe(TOPICBASE "Put/#");     // Send commands to this device, use Home/LetterBox/Get/# for responses.
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      digitalWrite(BLUE_PIN, HIGH);
      delay(5000);
      digitalWrite(BLUE_PIN, LOW);
    }
  }
}

/*
   The setup function. We only start the sensors here
*/
void setup(void)
{
  Serial.begin(9600); // start serial port
  Wire.begin(); // Wake up I2C bus
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
  for ( i = 15; i < 17; i++) {
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
  PublishFloat((char *)"Temperature", temperature);
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
  if (FLAG == true) {
    // interrupt has occurred
    // REQ_HEAT = !REQ_HEAT
    OVRDE = !OVRDE;
    FLAG = false;
  }
  if ( REQ_HEAT || OVRDE ) {
    SSR_CTRL(true);
  }
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
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
   Required to read the MAC address ROM
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

void Publish(char *Topic, char *Message)
{
  char TopicBase[80] = TOPICBASE;

  strcat(TopicBase, Topic);
  client.publish(TopicBase, Message);
}

void PublishFloat(char *Topic, float Value)
{
  char TopicBase[80] = TOPICBASE;
  char Message[10] = "NULL";

  if (!isnan(Value))
    dtostrf(Value, 5, 2, Message);

  strcat(TopicBase, Topic);
  client.publish(TopicBase, Message);
}

