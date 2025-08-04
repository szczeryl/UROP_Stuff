//COM 4. Receiver 
#include <esp_now.h>
#include <WiFi.h>
#include <math.h> // to handle isnan()

#define RELAY_PIN 27
#define PM_THRESHOLD 30.0
#define REQUIRED_CONSECUTIVE_READINGS 3
#define NUM_AVERAGE_READINGS 20

typedef struct struct_message {
    bool dataReady;
    uint16_t co2Concentration;
    float temperature;
    float relativeHumidity;
    uint16_t pm25_env;  // PM2.5 value (unsigned integer)
} struct_message;

float pmReadings[NUM_AVERAGE_READINGS]; 
int readingIndex = 0;                   
int validReadings = 0;
uint8_t highCount = 0;                 
bool relayState = false;                // relay OFF at start 

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
        // Store valid PM2.5 readings (ignore NaN/0)
        if (!isnan(receivedData.pm25_env) && receivedData.pm25_env > 0) {
            pmReadings[readingIndex] = (float)receivedData.pm25_env; // Convert uint16_t to float
            readingIndex = (readingIndex + 1) % NUM_AVERAGE_READINGS;
            validReadings = min(validReadings + 1, NUM_AVERAGE_READINGS);

            // Calculate average every NUM_AVERAGE_READINGS
            if (validReadings == NUM_AVERAGE_READINGS) {
                float sum = 0.0;
                for (int i = 0; i < NUM_AVERAGE_READINGS; i++) {
                    sum += pmReadings[i];
                }
                
                //to test average values for now 
                Serial.print("Buffer: ");
                    for (int i = 0; i < NUM_AVERAGE_READINGS; i++) {
                    Serial.print(pmReadings[i]);
                    Serial.print(" ");
                    }
                    Serial.println();

                float averagePM25 = sum / NUM_AVERAGE_READINGS;
                validReadings = 0; // Reset for next cycle

                Serial.print("PM2.5 (Avg): ");
                Serial.print(averagePM25, 1);
                Serial.println(" μg/m³");

                // Threshold logic
                if (averagePM25 > PM_THRESHOLD) {
                    highCount++;
                    Serial.print("High Count: ");
                    Serial.println(highCount);

                    if (highCount >= REQUIRED_CONSECUTIVE_READINGS && !relayState) {
                        digitalWrite(RELAY_PIN, HIGH);
                        relayState = true;
                        Serial.println("Relay ON (High PM2.5)");
                    }
                } else {
                    highCount = 0;
                    if (relayState) {
                        digitalWrite(RELAY_PIN, LOW);
                        relayState = false;
                        Serial.println("Relay OFF (Normal PM2.5)");
                    }
                }
            }
        } else {
            Serial.println("Invalid PM2.5 reading received");
        }
    }
}

void loop() {}
