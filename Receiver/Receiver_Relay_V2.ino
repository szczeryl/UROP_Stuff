#include <esp_now.h>
#include <WiFi.h>

#define RELAY_PIN 27
#define PM_THRESHOLD 30 //define threshold as 30 μg/m³ for now. need to change for real life application.
#define REQUIRED_CONSECUTIVE_READINGS 3

typedef struct struct_message {
    bool dataReady;
    uint16_t co2Concentration;
    float temperature;
    float relativeHumidity;
    uint16_t pm25_standard;
} struct_message;

uint8_t highCount = 0; //counter for number of high reading occurences 
bool relayState = false; //set relay to off first 

void setup() {
    Serial.begin(115200);
    pinMode(RELAY_PIN, OUTPUT);  
    digitalWrite(RELAY_PIN, LOW);
    
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
        
        if (receivedData.pm25_standard > PM_THRESHOLD) {
            highCount += 1;
            Serial.print("Count: ");
            Serial.println(highCount);
            
            //on relay if >=3 high readings 
            if (highCount >= REQUIRED_CONSECUTIVE_READINGS && !relayState) {
                digitalWrite(RELAY_PIN, HIGH);
                relayState = true;
                Serial.println("High PM2.5 - Relay ON");
            }
        } 
        else {
              highCount = 0; //reset when reading drops to low region again
            if (relayState) {
                digitalWrite(RELAY_PIN, LOW);
                relayState = false; //off originally (link back to the start)
                Serial.println("Normal PM2.5 - Relay OFF");
            }
        }
    }
}

void loop() {}
