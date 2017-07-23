#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"




/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "FRITZ!Box 4020 SE"
#define WLAN_PASS       "xxxxxxxxxxxxxxxxxxxx"

/************************* MQTT       Setup *********************************/

#define AIO_SERVER      "xx.xx.xxx.xxx"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "xxxxx"
#define AIO_KEY         "xxxxx"

// SI7021 I2C address is 0x40(64)
#define si7021Addr 0x40

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish tempchannel = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/home/garden/temp");
Adafruit_MQTT_Publish humichannel = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/home/garden/humi");


// Bug workaround for Arduino 1.6.6, it seems to need a function declaration
// for some reason (only affects ESP8266, likely an arduino-builder bug).
void MQTT_connect();

void getSiData(unsigned int *_ret_data, byte _i2c_command);
  
void setup() {
  Serial.begin(115200);
  Wire.begin();

  //reset sensor by sending 0xFE command to the Si7021 address
  Wire.beginTransmission(si7021Addr);
  Wire.write(0xFE); // Write reset command
  Wire.endTransmission();
  delay(15); // Default = 15ms

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
}
  
void loop() {
    //sensor returns 2 bytes via I2C. It will be converted to temperature or humidity later
    unsigned int data[2];

    //Send humidity measurement command and get response into the array 'data'
    getSiData(data, 0xE5);
 
    // Convert the data
    float humidity  = ((data[0] * 256.0) + data[1]);
    humidity = ((125 * humidity) / 65536.0) - 6;
 
    // Send temperature measurement command
    getSiData(data, 0xE3);
  
    // Convert the data
    float temp  = ((data[0] * 256.0) + data[1]);
    float celsTemp = ((175.72 * temp) / 65536.0) - 46.85;
    float fahrTemp = celsTemp * 1.8 + 32;

  
    Serial.print("Temperature = ");
    Serial.print(celsTemp);
    Serial.println(" *C");

    MQTT_connect();
    Serial.print(F("\nSending temp val "));
    Serial.print(celsTemp);
    Serial.print("...");
    if (! tempchannel.publish(celsTemp)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
    
    Serial.print(F("\nSending humidity val "));
    Serial.print(humidity);
    Serial.print("...");
    if (! humichannel.publish(humidity)) {
      Serial.println(F("Failed"));
    } else {
      Serial.println(F("OK!"));
    }
    
    Serial.println();
    Serial.println("Going into deep sleep for 20 seconds");
    ESP.deepSleep(300e6);
}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

void getSiData(unsigned int *_ret_data, byte _i2c_command)
{
  // start i2c communication 
  Wire.beginTransmission(si7021Addr);
  //send i2c command to sensor
  Wire.write(_i2c_command);
  // we are done with our transmission...close i2c communication
  Wire.endTransmission();
  delay(85);
 
  // Request 2 bytes of data
  Wire.requestFrom(si7021Addr, 2);
  // Read 2 bytes of data and save it to _ret_data which points to 'data[2]'
  if(Wire.available() == 2)
  {
    _ret_data[0] = Wire.read();
    _ret_data[1] = Wire.read();
  }
}

