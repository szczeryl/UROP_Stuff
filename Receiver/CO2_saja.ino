//COM 3 (CO2 Receiver)
#include <esp_now.h>
#include <WiFi.h>
#include <math.h> // to handle isnan()

#define RELAY_PIN_1 25 //fan
#define RELAY_PIN_2 26 //light
#define RELAY_PIN_3 27 //pump

#define CO2_THRESHOLD 1000.0  //ppm
#define REQUIRED_CONSECUTIVE_READINGS 3
#define NUM_AVERAGE_READINGS 20

typedef struct struct_message {
    bool dataReady;
    uint16_t co2Concentration; 
    float temperature;
    float relativeHumidity;
    uint16_t pm25_env;  
} struct_message;

float co2Readings[NUM_AVERAGE_READINGS]; 
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
    Serial.println("CO2 Receiver Ready - Waiting for data...");
}

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int data_len) {
    struct_message receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    if (data_len == sizeof(struct_message)) {
        if (!isnan(receivedData.co2Concentration) && receivedData.co2Concentration >0) { // Reasonable range check
            co2Readings[readingIndex] = (float)receivedData.co2Concentration;
            readingIndex = (readingIndex + 1) % NUM_AVERAGE_READINGS;
            validReadings = min(validReadings + 1, NUM_AVERAGE_READINGS);

            // raw co2 values
            /*Serial.print("Raw CO2: ");
            Serial.print(receivedData.co2Concentration);
            Serial.println(" ppm");*/

            if (validReadings == NUM_AVERAGE_READINGS) {
                float minCO2  = co2Readings[0];
                float maxCO2 = co2Readings[0];
                float sumCO2 = co2Readings[0];
                
                for (int i = 1; i < NUM_AVERAGE_READINGS; i++) {
                    if (co2Readings[i] < minCO2) minCO2 = co2Readings[i];
                    if (co2Readings[i] > maxCO2) maxCO2 = co2Readings[i];
                    sumCO2 += co2Readings[i];
                }

                // buffer. Delete later 
                Serial.print("CO2 Buffer: ");
                for (int i = 0; i < NUM_AVERAGE_READINGS; i++) {
                    Serial.print(co2Readings[i]);
                    Serial.print(" ");
                }
                Serial.println();

                // Remove max min
                float averageCO2 = (sumCO2 - minCO2 - maxCO2) / (NUM_AVERAGE_READINGS - 2);
                validReadings = 0; // Reset back to 0

                Serial.print("CO2 (Avg): ");
                Serial.print(averageCO2, 1);
                Serial.println(" ppm");

                if (averageCO2 > CO2_THRESHOLD) { //greater than 1000ppm
                    highCount++;
                    Serial.print("High Count: ");
                    Serial.println(highCount);

                    if (highCount >= REQUIRED_CONSECUTIVE_READINGS) {
                        if (!relayState1) { //only if off 
                            digitalWrite(RELAY_PIN_1, HIGH);
                            relayState1 = true;
                            Serial.println("Fan ON (High CO2)");
                        }
                        if (!relayState2) {
                            digitalWrite(RELAY_PIN_2, HIGH);
                            relayState2 = true;
                            Serial.println("Light ON (High CO2)");
                        }
                        if (!relayState3) {
                            digitalWrite(RELAY_PIN_3, HIGH);
                            relayState3 = true;
                            Serial.println("Pump ON (High CO2)");
                        }
                    }
                } else {
                    highCount = 0;
                    if (relayState1) { //if on then off 
                        digitalWrite(RELAY_PIN_1, LOW);
                        relayState1 = false;
                        Serial.println("Fan OFF (Normal CO2)");
                    }
                    if (relayState2) {
                        digitalWrite(RELAY_PIN_2, LOW);
                        relayState2 = false;
                        Serial.println("Light OFF (Normal CO2)");
                    }
                    if (relayState3) {
                        digitalWrite(RELAY_PIN_3, LOW);
                        relayState3 = false;
                        Serial.println("Pump OFF (Normal CO2)");
                    }
                }
            }
        } else {
            Serial.println("Invalid CO2 reading received");
        }
    }
}

void loop() {
}
