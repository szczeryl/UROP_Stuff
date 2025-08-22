//COM 3
#include <esp_now.h>
#include <WiFi.h>
#include <math.h> // to handle isnan()

// Define multiple relay pins
#define RELAY_PIN_1 25 //fan
#define RELAY_PIN_2 26 //light
#define RELAY_PIN_3 27 //pump
#define PM_THRESHOLD 12.0
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
bool relayState1 = false;  // fan
bool relayState2 = false;  // light
bool relayState3 = false;  // pump

void setup() {
    Serial.begin(115200);
    
    pinMode(RELAY_PIN_1, OUTPUT);
    pinMode(RELAY_PIN_2, OUTPUT);
    pinMode(RELAY_PIN_3, OUTPUT); //set all as output
    
    digitalWrite(RELAY_PIN_1, LOW);
    digitalWrite(RELAY_PIN_2, LOW);
    digitalWrite(RELAY_PIN_3, LOW); //all off 

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

            if (validReadings == NUM_AVERAGE_READINGS) {
                float min = pmReadings[0];
                float max = pmReadings[0];
                float sum = pmReadings[0];
                
                for (int i = 1; i < NUM_AVERAGE_READINGS; i++) {
                    if (pmReadings[i] < min) min = pmReadings[i];
                    if (pmReadings[i] > max) max = pmReadings[i];
                    sum += pmReadings[i];
                }

                // To test average values for now 
                Serial.print("Buffer: ");
                for (int i = 0; i < NUM_AVERAGE_READINGS; i++) {
                    Serial.print(pmReadings[i]);
                    Serial.print(" ");
                }
                Serial.println();

                float averagePM25 = (sum - min - max) / (NUM_AVERAGE_READINGS - 2);
                validReadings = 0; // Reset for next cycle

                Serial.print("PM2.5 (Avg): ");
                Serial.print(averagePM25, 1);
                Serial.println(" μg/m³");

                // Threshold logic - control all three relays based on PM2.5 level
                if (averagePM25 > PM_THRESHOLD) {
                    highCount++;
                    Serial.print("High Count: ");
                    Serial.println(highCount);

                    if (highCount >= REQUIRED_CONSECUTIVE_READINGS) {
                        // Turn ON all relays if not already on
                        if (!relayState1) {
                            digitalWrite(RELAY_PIN_1, HIGH);
                            relayState1 = true;
                            Serial.println("Fan ON (High PM2.5)");
                        }
                        if (!relayState2) {
                            digitalWrite(RELAY_PIN_2, HIGH);
                            relayState2 = true;
                            Serial.println("Light ON (High PM2.5)");
                        }
                        if (!relayState3) {
                            digitalWrite(RELAY_PIN_3, HIGH);
                            relayState3 = true;
                            Serial.println("Pump ON (High PM2.5)");
                        }
                    }
                } else {
                    highCount = 0;
                    // Turn OFF all relays if not already off
                    if (relayState1) {
                        digitalWrite(RELAY_PIN_1, LOW);
                        relayState1 = false;
                        Serial.println("Fan OFF (Normal PM2.5)");
                    }
                    if (relayState2) {
                        digitalWrite(RELAY_PIN_2, LOW);
                        relayState2 = false;
                        Serial.println("Light OFF (Normal PM2.5)");
                    }
                    if (relayState3) {
                        digitalWrite(RELAY_PIN_3, LOW);
                        relayState3 = false;
                        Serial.println("Pump OFF (Normal PM2.5)");
                    }
                }
            }
        } else {
            Serial.println("Invalid PM2.5 reading received");
        }
    }
}

void loop() {
    // Your main loop code here (if any)
}
