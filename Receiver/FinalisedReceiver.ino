//COM 4 Port
#include <esp_now.h>
#include <WiFi.h>
#include <math.h> // to handle isnan()

// set gpio pins 
#define RELAY_PIN_1 25 //fan
#define RELAY_PIN_2 26 //light
#define RELAY_PIN_3 27 //pump

// Threshold for pm2.5 and co2
#define PM_THRESHOLD 12.0
#define CO2_THRESHOLD 1000.0 // ppm
#define REQUIRED_CONSECUTIVE_READINGS 3
#define NUM_AVERAGE_READINGS 20

typedef struct struct_message {
    bool dataReady;
    uint16_t co2Concentration;
    float temperature;
    float relativeHumidity;
    uint16_t pm25_env;
} struct_message;

// arrays for pm2.5 and co2
float pmReadings[NUM_AVERAGE_READINGS];
float co2Readings[NUM_AVERAGE_READINGS];
int pmReadingIndex = 0;
int co2ReadingIndex = 0;
int pmValidReadings = 0;
int co2ValidReadings = 0;

// counters for pm2.5 and co2
uint8_t pmHighCount = 0;
uint8_t co2HighCount = 0;

// set everything off 
bool relayState1 = false;  // fan
bool relayState2 = false;  // light
bool relayState3 = false;  // pump

void setup() {
    Serial.begin(115200);
    
    pinMode(RELAY_PIN_1, OUTPUT);
    pinMode(RELAY_PIN_2, OUTPUT);
    pinMode(RELAY_PIN_3, OUTPUT);
    
    digitalWrite(RELAY_PIN_1, LOW);
    digitalWrite(RELAY_PIN_2, LOW);
    digitalWrite(RELAY_PIN_3, LOW);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while(1) delay(10);
    }
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("Receiver Ready - Waiting for PM2.5 and CO2 data...");
}

void processPM25Data(uint16_t pm25_value) {
    // Store valid PM2.5 readings
    if (!isnan(pm25_value) && pm25_value > 0) {
        pmReadings[pmReadingIndex] = (float)pm25_value;
        pmReadingIndex = (pmReadingIndex + 1) % NUM_AVERAGE_READINGS;
        pmValidReadings = min(pmValidReadings + 1, NUM_AVERAGE_READINGS);

        if (pmValidReadings == NUM_AVERAGE_READINGS) {
            float min = pmReadings[0];
            float max = pmReadings[0];
            float sum = pmReadings[0];
            
            for (int i = 1; i < NUM_AVERAGE_READINGS; i++) {
                if (pmReadings[i] < min) min = pmReadings[i];
                if (pmReadings[i] > max) max = pmReadings[i];
                sum += pmReadings[i];
            }

            float averagePM25 = (sum - min - max) / (NUM_AVERAGE_READINGS - 2);
            pmValidReadings = 0;

            Serial.print("PM2.5 (Avg): ");
            Serial.print(averagePM25);
            Serial.println(" μg/m³");

            // if higher than threshold; logic 
            if (averagePM25 > PM_THRESHOLD) {
                pmHighCount++;
                Serial.print("PM High Count: ");
                Serial.println(pmHighCount);

                if (pmHighCount >= REQUIRED_CONSECUTIVE_READINGS) {
                    activateRelays("High PM2.5");
                }
            } else {
                pmHighCount = 0;
                checkDeactivateRelays("Normal PM2.5");
            }
        }
    }
}

void processCO2Data(uint16_t co2_value) {
    // Store valid CO2 readings
    if (!isnan(co2_value) && co2_value > 0) {
        co2Readings[co2ReadingIndex] = (float)co2_value;
        co2ReadingIndex = (co2ReadingIndex + 1) % NUM_AVERAGE_READINGS;
        co2ValidReadings = min(co2ValidReadings + 1, NUM_AVERAGE_READINGS);

        if (co2ValidReadings == NUM_AVERAGE_READINGS) {
            float min = co2Readings[0];
            float max = co2Readings[0];
            float sum = co2Readings[0];
            
            for (int i = 1; i < NUM_AVERAGE_READINGS; i++) {
                if (co2Readings[i] < min) min = co2Readings[i];
                if (co2Readings[i] > max) max = co2Readings[i];
                sum += co2Readings[i];
            }

            float averageCO2 = (sum - min - max) / (NUM_AVERAGE_READINGS - 2);
            co2ValidReadings = 0;

            Serial.print("CO2 (Avg): ");
            Serial.print(averageCO2);
            Serial.println(" ppm");

            // CO2 threshold logic
            if (averageCO2 > CO2_THRESHOLD) {
                co2HighCount++;
                Serial.print("CO2 High Count: ");
                Serial.println(co2HighCount);

                if (co2HighCount >= REQUIRED_CONSECUTIVE_READINGS) {
                    activateRelays("High CO2");
                }
            } else {
                co2HighCount = 0;
                checkDeactivateRelays("Normal CO2");
            }
        }
    }
}

void activateRelays(const char* reason) {
    // if not ON, Turn ON all relays 
    if (!relayState1) {
        digitalWrite(RELAY_PIN_1, HIGH);
        relayState1 = true;
        Serial.print("Fan ON (");
        Serial.print(reason);
        Serial.println(")");
    }
    if (!relayState2) {
        digitalWrite(RELAY_PIN_2, HIGH);
        relayState2 = true;
        Serial.print("Light ON (");
        Serial.print(reason);
        Serial.println(")");
    }
    if (!relayState3) {
        digitalWrite(RELAY_PIN_3, HIGH);
        relayState3 = true;
        Serial.print("Pump ON (");
        Serial.print(reason);
        Serial.println(")");
    }
}

void checkDeactivateRelays(const char* reason) {
    // Only turn OFF relays if both co2 and pm2.5 below threshold 
    if (pmHighCount == 0 && co2HighCount == 0) {
        if (relayState1) {
            digitalWrite(RELAY_PIN_1, LOW);
            relayState1 = false;
            Serial.print("Fan OFF (");
            Serial.print(reason);
            Serial.println(")");
        }
        if (relayState2) {
            digitalWrite(RELAY_PIN_2, LOW);
            relayState2 = false;
            Serial.print("Light OFF (");
            Serial.print(reason);
            Serial.println(")");
        }
        if (relayState3) {
            digitalWrite(RELAY_PIN_3, LOW);
            relayState3 = false;
            Serial.print("Pump OFF (");
            Serial.print(reason);
            Serial.println(")");
        }
    }
}

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int data_len) {
    struct_message receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    if (data_len == sizeof(struct_message)) {
        // final PM2.5 data
        if (!isnan(receivedData.pm25_env) && receivedData.pm25_env > 0) {
            processPM25Data(receivedData.pm25_env);
        } else {
            Serial.println("Invalid PM2.5 reading received");
        }

        // final CO2 data
        if (!isnan(receivedData.co2Concentration) && receivedData.co2Concentration > 0) {
            processCO2Data(receivedData.co2Concentration);
        } else {
            Serial.println("Invalid CO2 reading received");
        }

       /*Serial.print("Raw - PM2.5: ");
        Serial.print(receivedData.pm25_env);
        Serial.print(" μg/m³, CO2: ");
        Serial.print(receivedData.co2Concentration);
        Serial.print(" ppm, Temp: ");
        Serial.print(receivedData.temperature);
        Serial.print("°C, Humidity: ");
        Serial.print(receivedData.relativeHumidity);
        Serial.println("%")*/
    }
}

void loop() {
    delay(100);
}
