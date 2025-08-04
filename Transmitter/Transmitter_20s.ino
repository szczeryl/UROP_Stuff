//COM 8. Transmitter

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include "Adafruit_PM25AQI.h"
#include <SensirionI2cScd4x.h>

//initialie sensors 
HardwareSerial pmSerial(1);  
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
SensirionI2cScd4x scd41;

uint8_t broadcastAddress[] = {0x38, 0x18, 0x2B, 0x30, 0x86, 0x64}; // receiver MAC address 
esp_now_peer_info_t peerInfo;

typedef struct struct_message {
    bool dataReady;
    uint16_t co2Concentration;
    float temperature;
    float relativeHumidity;
    uint16_t pm25_env;
} struct_message;

struct_message sensorData;

unsigned long lastPMReading = 0;
const unsigned long pmInterval = 20000;
unsigned long lastSCDReading = 0;
const unsigned long scdInterval = 20000;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setupSensors() {
  // PM2.5
    pmSerial.begin(9600, SERIAL_8N1, 19, -1); //mereka ckp yg PM2.5 must use 9600?? tapi x tau
    if (!aqi.begin_UART(&pmSerial)) {
        Serial.println("Could not find PM2.5 sensor!");
        while(1) delay(10);
    }

  //SCD41 
    Wire.begin();
    scd41.begin(Wire, SCD41_I2C_ADDR_62);
    
    scd41.stopPeriodicMeasurement();
    delay(500);
    
    // Start to measure 
    uint16_t error = scd41.startPeriodicMeasurement();
    if (error) {
        Serial.print("SCD41 init error: ");
        Serial.println(error);
        while(1) delay(10);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100);

    setupSensors();

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while(1) delay(10);
    }

    //esp_now_register_send_cb(OnDataSent);
    
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        while(1) delay(10);
    }

    Serial.println("System initialized");
}

void readPM25() {
    PM25_AQI_Data data;
    if (aqi.read(&data)) {
        sensorData.pm25_env = data.pm25_env;
        Serial.print("PM2.5: ");
        Serial.print(data.pm25_env);
        Serial.println(" μg/m³");
    } else {
        Serial.println("PM2.5 read failed");
    }
}

void readSCD41() {
    bool dataReady = false;
    uint16_t error = scd41.getDataReadyStatus(dataReady);
    
    if (!error && dataReady) {
        error = scd41.readMeasurement(sensorData.co2Concentration, 
                                    sensorData.temperature, 
                                    sensorData.relativeHumidity);
        if (!error) {
            sensorData.dataReady = true;
            Serial.print("CO2: ");
            Serial.print(sensorData.co2Concentration);
            Serial.print("ppm, Temp: ");
            Serial.print(sensorData.temperature);
            Serial.print("°C, RH: ");
            Serial.print(sensorData.relativeHumidity);
            Serial.println("%");
        } else {
            Serial.println("SCD41 read error");
        }
    }
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - lastPMReading >= pmInterval) {
        readPM25();
        lastPMReading = currentMillis;
    }

    if (currentMillis - lastSCDReading >= scdInterval) {
        readSCD41();
        lastSCDReading = currentMillis;
        
        esp_err_t result = esp_now_send(broadcastAddress, 
                                      (uint8_t *)&sensorData, 
                                      sizeof(sensorData));
        if (result != ESP_OK) {
            Serial.println("ESP-NOW send failed");
        }
    }

    delay(100); // small delay 
}

//use millis instead of delay as data can continuously be read
//need to use for multitasking 
