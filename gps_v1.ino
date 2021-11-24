#include "UbidotsEsp32Mqtt.h"

const char *UBIDOTS_TOKEN = "BBFF-SlqcWJr8JGxZtsEHP4WZG9iB4fPOha";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "Shady nettverk";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "heiheihei";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "esp32gps";   // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "pos"; // Put here your Variable label to which data  will be published

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

unsigned long timer;

float latitude = 1.25164;
float longitude = -77.28426;

Ubidots ubidots(UBIDOTS_TOKEN);

#include <HardwareSerial.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial2 // RX - IO17; TX - IO16
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Adafruit GPS logging start test!");

  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);

    

}

void loop() {
  // put your main code here, to run repeatedly:

}
