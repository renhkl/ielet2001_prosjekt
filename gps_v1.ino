#include <HardwareSerial.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial2 // RX - IO17; TX - IO16
Adafruit_GPS GPS(&GPSSerial);

#define GPSECHO false

uint32_t timer = millis();


void setup() {
  while (!Serial);
  Serial.begin(115200);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  //GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
}

void loop() {
  char c = GPS.read(); // Henter GPS-data

  // Parser GPS-data
  if (GPS.newNMEAreceived()) {
    Serial.print(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }


  if (millis() - timer > 5000) { // Kjører hvert femte sekund
    timer = millis(); // Resetter timer
    if (GPS.fix) { // Hvis det er GPS-signal
      float x = GPS.longitude;
      float y = GPS.latitude;

      // 
    }
    else { // Hvis det ikke er GPS-signal
      Serial.println("...");
    }
  }
}
