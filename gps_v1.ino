#include "UbidotsEsp32Mqtt.h"

const char *UBIDOTS_TOKEN = "BBFF-8eYG4XFUFDUWjtzgxbu7WMXwj2EavJ";  // Token til IELETgruppe4
const char *WIFI_SSID = "Shady_nettverk";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "heiheihei";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "esp32";   // Put here your Device label to which data  will be published
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
  
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // 
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  Serial.print("123");
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
    
  delay(1000);
  timer = millis();
}


void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void loop() {
  char c = GPS.read(); // Henter GPS-data

  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }

  if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {/* Reserves memory to store context key values, add as much as you need */
    char* str_lat = (char*)malloc(sizeof(char) * 10);
    char* str_lng = (char*)malloc(sizeof(char) * 10);
    //char str_lat,str_lng,context;

    // Parser GPS-data
    if (GPS.newNMEAreceived()) {
      Serial.print(GPS.lastNMEA());
      if (!GPS.parse(GPS.lastNMEA()))
        return;
    }
    
    if (GPS.fix) { // Hvis det er GPS-signal
      longitude = GPS.longitude;
      latitude = GPS.latitude;
      Serial.println(" "); Serial.print("Longitude: "); Serial.print(longitude);
      Serial.print("   Latitude: "); Serial.print(latitude); Serial.println(" ");
    }
    else { // Hvis det ikke er GPS-signal
      Serial.println("...");
    }

    /* Saves the coordinates as char */
    sprintf(str_lat, "%f", latitude);
    sprintf(str_lng, "%f", longitude);

    /* Reserves memory to store context array */
    char* context = (char*)malloc(sizeof(char) * 30);

    /* Adds context key-value pairs */
    ubidots.addContext("lat", str_lat);
    ubidots.addContext("lng", str_lng);

    /* Builds the context with the coordinates to send to Ubidots */
    ubidots.getContext(context);

    ubidots.add(VARIABLE_LABEL, 1, context); 
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
  }
  
  ubidots.loop();
}
