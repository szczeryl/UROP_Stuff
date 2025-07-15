//SCD41 Sensor 
#include <esp_now.h>
#include <WiFi.h>
#include <Arduino.h>
#include <SensirionI2cScd4x.h>
#include <Wire.h>

#ifndef NO_ERROR
#define NO_ERROR 0
#endif

SensirionI2cScd4x sensor;
static char errorMessage[64];
static int16_t error;

// Receiver Mac Address
uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0xC5, 0xFE, 0xDC};

// Data structure to send
typedef struct struct_message {
    bool dataReady;
    uint16_t co2Concentration;
    float temperature;
    float relativeHumidity;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setupSensor() {
    Wire.begin();
    sensor.begin(Wire, SCD41_I2C_ADDR_62);

    uint64_t serialNumber = 0;
    delay(30);
    
    // Ensure sensor in clean state
    error = sensor.wakeUp();
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute wakeUp(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
    }
    
    error = sensor.stopPeriodicMeasurement();
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
    }
    
    error = sensor.reinit();
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute reinit(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
    }
    
    // Read serial number
    error = sensor.getSerialNumber(serialNumber);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
    Serial.print("Serial number: 0x");
    Serial.print((uint32_t)(serialNumber >> 32), HEX);
    Serial.println((uint32_t)(serialNumber & 0xFFFFFFFF), HEX);
    
    // Start periodic measurements
    error = sensor.startPeriodicMeasurement();
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        return;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register send callback
    esp_now_register_send_cb(OnDataSent);
 
    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
 
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    //sensor
    setupSensor();

    Serial.println("Transmitter initialized");
}

void loop() {
    bool dataReady = false;
    
    // Check if data is ready
    error = sensor.getDataReadyStatus(dataReady);
    if (error != NO_ERROR) {
        Serial.print("Error trying to execute getDataReadyStatus(): ");
        errorToString(error, errorMessage, sizeof errorMessage);
        Serial.println(errorMessage);
        delay(5000);
        return;
    }

    if (dataReady) {
      // read
        error = sensor.readMeasurement(myData.co2Concentration, myData.temperature, myData.relativeHumidity);
        if (error != NO_ERROR) {
            Serial.print("Error trying to execute readMeasurement(): ");
            errorToString(error, errorMessage, sizeof errorMessage);
            Serial.println(errorMessage);
        } else {
            myData.dataReady = true;
            
            Serial.print("CO2: ");
            Serial.print(myData.co2Concentration);
            Serial.print(" ppm, Temp: ");
            Serial.print(myData.temperature);
            Serial.print(" °C, RH: ");
            Serial.print(myData.relativeHumidity);
            Serial.println(" %");
            
            //Send via ESP-NOW
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
            if (result != ESP_OK) {
                Serial.println("Error sending the data");
            }
        }
    } else {
        Serial.println("Data not ready yet");
    }
    
    delay(5000);  // Wait 5 seconds between measurements
}
