#include "Adafruit_PM25AQI.h"

HardwareSerial pmSerial(1);  // Using Serial1 on ESP32

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

void setup() {
  Serial.begin(115200);

//3s for sensor to wake up
  delay(1000);

  pmSerial.begin(9600, SERIAL_8N1, 19, -1);  // RX on 19, no TX)

  // Connect to the sensor over UART
  if (! aqi.begin_UART(&pmSerial)) {
    Serial.println("No PM2.5!");
    while (1) delay(10);
  }

  Serial.println("PM2.5 found");
}

void loop() {
  PM25_AQI_Data data;

  if (!aqi.read(&data)){
    Serial.println("Could not read sensor AQI");
    delay(2000);
  }
  
  Serial.print(F("PM 2.5: ")); Serial.print(data.pm25_standard); Serial.print(F("ug/m3 (Environmental)\n"));

  delay(5000);
}
