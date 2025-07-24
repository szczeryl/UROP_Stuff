#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
    bool dataReady;
    uint16_t co2Concentration;
    float temperature;
    float relativeHumidity;
    uint16_t pm25_standard;
} struct_message;

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int data_len) {
    const uint8_t *mac_addr = esp_now_info->src_addr; //extract MAC address 
    
    struct_message receivedData;
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    
    Serial.print("Received from: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(mac_addr[i], HEX);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    
    if (data_len == sizeof(struct_message)) {
        Serial.print("CO2: "); Serial.print(receivedData.co2Concentration); Serial.println(" ppm");
        Serial.print("Temp: "); Serial.print(receivedData.temperature); Serial.println(" °C");
        Serial.print("Humidity: "); Serial.print(receivedData.relativeHumidity); Serial.println(" %");
        Serial.print("PM2.5: "); Serial.print(receivedData.pm25_standard); Serial.print(F("ug/m3 (Environmental)\n"));
    } else {
        Serial.println("Invalid data length received");
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        return;
    }
    
    // Register with correct signature
    esp_now_register_recv_cb(OnDataRecv);
    
    Serial.println("Receiver Ready");
}

void loop() {}
