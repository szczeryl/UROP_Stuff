#include <esp_now.h>
#include <WiFi.h>

#define RELAY_PIN 27

typedef struct struct_message {
    bool dataReady;
    uint16_t co2Concentration;
    float temperature;
    float relativeHumidity;
    uint16_t pm25_standard;
} struct_message;

void setup() {
    Serial.begin(115200);
    pinMode(RELAY_PIN, OUTPUT);  
    digitalWrite(RELAY_PIN, LOW); // start with relay off
    
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while(1) delay(10);
    }
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("Receiver Ready - Waiting for data...");
}

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int data_len) {
    struct_message receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    
    if (data_len == sizeof(struct_message)) {
        Serial.print("PM2.5: "); 
        Serial.print(receivedData.pm25_standard); 
        Serial.println(" μg/m³");
        
        if (receivedData.pm25_standard > 30) {
            digitalWrite(RELAY_PIN, HIGH);
            Serial.println("High PM2.5 - Relay on");
            delay(1000);

        } else {
            digitalWrite(RELAY_PIN, LOW);
            Serial.println("Normal PM2.5 - Relay off");
        }
    }
}

void loop() {}  
