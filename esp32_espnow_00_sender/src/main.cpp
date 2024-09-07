/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1
unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings
 int readingId = 0;         // Show Readind number after initial, incremental


//MAC Address of the receiver 
//uint8_t broadcastAddress[] = {0xCC, 0xDB, 0xA7, 0x31, 0xBF, 0x48};  // for specific reciever
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};    // for boardcast , non specific reciever

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int id;
    float temp;
    float hum;
    int readingId;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Set values to send
  // strcpy(myData.a, "THIS IS A CHAR");
  // myData.b = random(1,20);
  // myData.c = 1.2;
  // myData.d = false;
  
   unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    //Set values to send
    myData.id = int(BOARD_ID);
    myData.temp = float(random(25,35));
    myData.hum = float(random(75,99));
    myData.readingId = readingId++;
     
    //Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.print("send id:: "); Serial.println(int(BOARD_ID));
      Serial.print("Send temp:: "); Serial.println(myData.temp);
      Serial.print("Send hum:: "); Serial.println(myData.hum);
      Serial.print("Send readingID:: "); Serial.println(myData.readingId);

      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
  delay (5000);
}