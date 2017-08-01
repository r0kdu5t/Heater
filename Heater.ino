/*
   Heater.ino
*/
/*--------------------------- Configuration ------------------------------*/
//
#define DEBUG_ON   // comment out to supress serial monitor output
//
#ifdef DEBUG_ON
#define DEBUG_PRINT(x)   Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define SERIAL_START(x)  Serial.begin(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define SERIAL_START(x)
#endif
/* Network config */
#define ENABLE_DHCP                 true   // true/false
#define MAC_DS                      true   // true/false If use DS for MAC then make following false
#define ENABLE_MAC_ADDRESS_ROM      false   // true/false
#define MAC_I2C_ADDRESS             0x50   // Microchip 24AA125E48 I2C ROM address
//
// Important - Change following for each different device!!
static uint8_t mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };  // Set if no MAC ROM 
static uint8_t ip[] = { 192, 168, 1, 35 }; // Use if DHCP disabled. Ensure on correct network
/* MQTT config */
//IPAddress broker(192, 168, 62, 65);       // Address of the MQTT broker - "spunkmeyer.theatrix.priv"
static uint8_t broker[] = { 192, 168, 62, 65 };
// Topic base for all comms from this device.
#define TOPICBASE "Home/Sneezy/"


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
float tempValue;
bool override = false;  // OVER_RIDE or MANUAL
typedef enum {
  HEAT_OFF, HEAT_ON, OVER_RIDE
} HeaterStates;
//
HeaterStates state = HEAT_OFF;
HeaterStates lastState;
volatile bool buttonPushed = false;
//volatile boolean FLAG = false;
//unsigned long last_button_time = 0;
byte confSetTemp = 18;
unsigned long confTempDelay = 10000;    // Default temperature publish delay.
unsigned long LastTempMillis = 0;       // Stores the last millis() for determining update delay.
# define HYSTERESIS 2
# define SSR_PIN 6
# define BUTTON_LED_PIN 7
//bool SENT_SSR_STATUS = false;

# define RED_PIN 15 // analogPin A1
# define GREEN_PIN 16 // analogPin A2
# define BLUE_PIN 17  // analogPin A3

//Start MQTT goodness
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';    // Hack to be able to use this as a char string.

  if (strstr(topic, TOPICBASE "Config/"))
  {
    if (strstr(topic, "setTemp")) {
      confSetTemp = atoi((const char *)payload);
      //
      DEBUG_PRINT(F("Set temperature is now: "));
      //DEBUG_PRINTLN(confSetTemp, DEC);
      DEBUG_PRINTLN(confSetTemp);
    }
    else if (strstr(topic, "TempDelay")) {
      confTempDelay = atoi((const char *)payload);
      DEBUG_PRINT(F("Temperature send delay is now "));
      //DEBUG_PRINTLN(confSetTemp, DEC);
      DEBUG_PRINT(confTempDelay);
      DEBUG_PRINTLN(" milliSeconds.");
    }

    /*else if (strstr(topic, "CheckDelay"))
      confCheckDelay = atoi((const char *)payload);

      else if (strstr(topic, "LuxDelay"))
      confLuxDelay = atoi((const char *)payload);
    */
  }
}

// Initialize the Ethernet client library
EthernetClient ethClient;
PubSubClient mqttClient( broker, 1883, callback, ethClient); // MQTT object

void PushButton() // Interrupt Service Routine (ISR) with debounce
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  //check to see if increment() was called in the last 250 milliseconds
  if (interrupt_time - last_interrupt_time > 250)
  {
    buttonPushed = true;
  }
  last_interrupt_time = interrupt_time;
}  // end of isr

#ifdef ENABLE_MAC_ADDRESS_ROM
/*
   Required to read address from DS device, create MAC
*/
void ethernetFromDS() {
  byte i;
  byte dsAddress[8];
  delay( 500 );

  DEBUG_PRINT ("Searching for DS18B20...");
  oneWire.reset_search();
  if ( !oneWire.search(dsAddress) )
  {
    DEBUG_PRINTLN("none found. Using specified MAC Address.");
  }
  else {
    DEBUG_PRINT( "Success! \nSetting MAC address...." );
    mac[1] = dsAddress[3];
    mac[2] = dsAddress[4];
    mac[3] = dsAddress[5];
    mac[4] = dsAddress[6];
    mac[5] = dsAddress[7];
  }
}
#endif

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    DEBUG_PRINT("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("SneezyClient", (char *)TOPICBASE "State", 1, 0, "DEAD")) {
      //if (mqttClient.connect("SleepyClient")) {
      DEBUG_PRINTLN("connected");
      // Once connected, publish an announcement...
      Publish((char *)"State", (char *)"BOOTUP");
      // ... and resubscribe
      // Subscribe to enable bi-directional comms.
      mqttClient.subscribe(TOPICBASE "Config/#");  // Allow bootup config fetching using MQTT persist flag!
      //mqttClient.subscribe(TOPICBASE "Put/#");     // Send commands to this device, use Home/<device_name>/Get/# for responses.
    } else {
      DEBUG_PRINT("failed, rc=");
      DEBUG_PRINT(mqttClient.state());
      DEBUG_PRINTLN(" try again in 5 seconds");
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
  //Serial.begin(9600); // start serial port
  SERIAL_START(9600);

  DEBUG_PRINTLN(F("Heater.ino by <r0kdu5t@theatrix.org.nz>"));
  //Serial.println( F("Heater.ino by <r0kdu5t@theatrix.org.nz>"));

  // Info String in Flash
  Serial.print( F("Compiled: "));
  Serial.print( F(__DATE__));
  Serial.print( F(", "));
  Serial.println( F(__TIME__));
  //Serial.print( F(", "));
  //Serial.println( F(__FILE__));
  delay(2000);

  /*
     Enable interrupt 0 which uses pin 2
     jump to the pButton function on rising edge
  */
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), PushButton, RISING);

  // Setup SSR control pin.
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW); // Turn 'Off' SSR.

  // Setup PushButton LED pin.
  pinMode(BUTTON_LED_PIN, OUTPUT);
  digitalWrite(BUTTON_LED_PIN, HIGH); // Turn 'Off' PushButton LED.

  // Setup the output status LEDs.
  byte i;
  for ( i = 15; i < 17; i++) {
    pinMode(i, OUTPUT);
    //sensors[sensorId].status_output
    digitalWrite(i, LOW); // Turn 'Off' LED.
  }

  // Setup PushButton PWR pin work-a-round
  pinMode(19, OUTPUT);    // pin 19 = A5
  digitalWrite(19, HIGH); // Turn 'On' PushButton PWR.

  //Start Ethernet using mac formed from DS
  if ( MAC_DS == true )
  {
    DEBUG_PRINTLN(F("Getting MAC address from DS: "));
    ethernetFromDS();
  }

  /*
     DEBUG - Stuff
  */
  /*  // What are my variable values
    Serial.println();
    Serial.println(confSetTemp, DEC);
    Serial.println(HYSTERESIS, DEC);
    Serial.println();
    //delay(2000);
  */
  if ( ENABLE_MAC_ADDRESS_ROM == true )
  {
    Wire.begin(); // Wake up I2C bus
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
  DEBUG_PRINTLN(tmpBuf);

  // setup the Ethernet library to talk to the Wiznet board
  if ( ENABLE_DHCP == true )
  {
    Ethernet.begin(mac);      // Use DHCP
  } else {
    Ethernet.begin(mac, ip);  // Use static address defined above
  }
  delay(2000); // Delay 2 seconds - allow Ethernet to come up,
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
  DEBUG_PRINTLN();
  // Start up the library
  sensors.begin();
}

/*
   Main function, get and show the temperature
*/
void loop(void)
{
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  sensors.requestTemperatures(); // Send the command to get temperatures

  tempValue = sensors.getTempCByIndex(0); // Get value from sensor

  if (confTempDelay && (millis() - LastTempMillis > confTempDelay))
  {
    LastTempMillis = millis();
    DEBUG_PRINT(F("Temperature: "));
    DEBUG_PRINTLN(tempValue);
    //Serial.print((int)temperature);
    PublishFloat((char *)"Temperature", tempValue); // Publish temperature value on topic
  }
  // Check if sensed value is less than set value minus HYSTERESIS
  if ((int)tempValue < (confSetTemp - HYSTERESIS))
  {
    // Turn On Output
    state = HEAT_ON;
    // COLD - blue
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(BLUE_PIN, HIGH);
    // Check if sensed value is more than set value plus HYSTERESIS
  }
  else if ((int)tempValue > (confSetTemp + HYSTERESIS))
  {
    // Turn Off Output
    state = HEAT_OFF;
    // TEMP OK - green
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, HIGH);
    digitalWrite(BLUE_PIN, LOW);
  }
/*  
  else
  {
    // ISH - yellow
    //slowToggleLED(RED_PIN);
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(BLUE_PIN, LOW);
    }
*/
  if (buttonPushed == true)
  {
    // interrupt has occurred
    DEBUG_PRINTLN(F("Button Pressed"));
    //
    if (state == HEAT_OFF)
    {
      state = OVER_RIDE;
      /*
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(BLUE_PIN, LOW);
      */
      override = true;
    }
    else if (state == OVER_RIDE || state == HEAT_ON)
    {
      state = HEAT_OFF;
      /*
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, LOW);
      digitalWrite(BLUE_PIN, LOW);
      */
      override = false;
    }
    buttonPushed = false;
  }

  // HEAT_OFF, HEAT_ON, OVER_RIDE
  if (state != lastState)
  {
    DEBUG_PRINT(F("state is: "));
    DEBUG_PRINT(state);
    DEBUG_PRINT(F(" lastState is: "));
    DEBUG_PRINTLN(lastState);
    DEBUG_PRINT(F(" override is: "));
    DEBUG_PRINTLN(override);    
  }

  if (state == OVER_RIDE)
  {
    //slowToggleLED(BUTTON_LED_PIN);
    //override = true;
    state = HEAT_ON;
    //fastToggleLED(BUTTON_LED_PIN);
  }
  //
  else if (state == HEAT_ON)
  {
    //fastToggleLed();
    if (override==true) {slowToggleLED(BUTTON_LED_PIN);}
    //slowToggleLED(BUTTON_LED_PIN);
    if (digitalRead(SSR_PIN) == LOW) {
      //state = digitalRead(13);
      //digitalWrite(SSR_PIN, HIGH);
      Publish((char *)"SSR", (char *)"ON");
      DEBUG_PRINTLN(F("SSR_CTRL: ON "));
      //
    }
    digitalWrite(SSR_PIN, HIGH);
  }
  else if (state == HEAT_OFF)
  {
    // Do Stuff
    if (digitalRead(SSR_PIN) == HIGH) {
      //digitalWrite(SSR_PIN, LOW);
      Publish((char *)"SSR", (char *)"OFF");
      DEBUG_PRINTLN(F("SSR_CTRL: OFF"));
      //
    }
    digitalWrite(SSR_PIN, LOW);
    digitalWrite(BUTTON_LED_PIN, HIGH); // Turn 'Off' PushButton LED
  }
  lastState = state;
  //delay(1000);
} // End of loop()

/*
   fastToggleLed : check mysensors.org - modified
*/
void fastToggleLED(byte ledPin)
{
  static unsigned long fastLedTimer;
  if (millis() - fastLedTimer >= 100UL)
  {
    digitalWrite(ledPin, !digitalRead(ledPin));
    fastLedTimer = millis ();
  }
}
/*
   slowToggleLED : check mysensors.org - modified
*/
void slowToggleLED(byte ledPin)
{
  static unsigned long slowLedTimer;
  if (millis() - slowLedTimer >= 1250UL)
  {
    digitalWrite(ledPin, !digitalRead(ledPin));
    slowLedTimer = millis ();
  }
}

#ifdef ENABLE_MAC_ADDRESS_ROM
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
#endif // ENABLE_MAC_ADDRESS_ROM

void Publish(char *Topic, char *Message)
{
  char TopicBase[80] = TOPICBASE;

  strcat(TopicBase, Topic);
  mqttClient.publish(TopicBase, Message);
}

void PublishFloat(char *Topic, float Value)
{
  char TopicBase[80] = TOPICBASE;
  char Message[10] = "NULL";

  if (!isnan(Value))
    dtostrf(Value, 5, 2, Message);

  strcat(TopicBase, Topic);
  mqttClient.publish(TopicBase, Message);
}

