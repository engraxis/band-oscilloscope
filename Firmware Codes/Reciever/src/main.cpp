# define DEBUG

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// REPLACE WITH THE MAC Address of the receiver 
uint8_t broadcastAddressThigh[] = {0xF8, 0xB3, 0xB7, 0x21, 0x0C, 0xC8}; // This address of the arm band used against the Leg band.
//uint8_t broadcastAddressThigh[] = {0xEC, 0x94, 0xCB, 0x4A, 0x3C, 0x10}; // This address of the Leg band.
uint8_t broadcastAddressShank[] = {0xFC, 0xF5, 0xC4, 0x0F, 0x27, 0xA0};

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    char ID;
    int FSR1;
    int FSR2;
    int FSR3;
    int FSR4;
    int FSR5;
    int FSR6;
    int FSR7;
    int FSR8;
    float IMU1;
    float IMU2;
    float IMU3;
    float IMU4;
    float IMU5;
    float IMU6;
    float IMU7;
    float IMU8;
    float IMU9;
    float IMU10;
    float IMU11;
    float IMU12;
    float IMU13;
    float IMU14;
    float IMU15;
    float IMU16;
} struct_message;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadingsThigh;
struct_message incomingReadingsShank;
struct_message totalData[2] = {incomingReadingsThigh, incomingReadingsShank};
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;
int board = 0;
char incomingID;

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  incomingID = incomingReadings.ID;
  if (incomingID == 't') {
    board = 0;
  } else if (incomingID == 's') {
    board = 1;
  }
  totalData[board].ID = incomingReadings.ID;
  totalData[board].FSR1 = incomingReadings.FSR1;
  totalData[board].FSR2 = incomingReadings.FSR2;
  totalData[board].FSR3 = incomingReadings.FSR3;
  totalData[board].FSR4 = incomingReadings.FSR4;
  totalData[board].FSR5 = incomingReadings.FSR5;
  totalData[board].FSR6 = incomingReadings.FSR6;
  totalData[board].FSR7 = incomingReadings.FSR7;
  totalData[board].FSR8 = incomingReadings.FSR8;
  totalData[board].IMU1 = incomingReadings.IMU1;
  totalData[board].IMU2 = incomingReadings.IMU2;
  totalData[board].IMU3 = incomingReadings.IMU3;
  totalData[board].IMU4 = incomingReadings.IMU4;
  totalData[board].IMU5 = incomingReadings.IMU5;
  totalData[board].IMU6 = incomingReadings.IMU6;
  totalData[board].IMU7 = incomingReadings.IMU7;
  totalData[board].IMU8 = incomingReadings.IMU8;
  totalData[board].IMU9 = incomingReadings.IMU9;
  totalData[board].IMU10 = incomingReadings.IMU10;
  totalData[board].IMU11 = incomingReadings.IMU11;
  totalData[board].IMU12 = incomingReadings.IMU12;
  totalData[board].IMU13 = incomingReadings.IMU13;
  totalData[board].IMU14 = incomingReadings.IMU14;
  totalData[board].IMU15 = incomingReadings.IMU15;
  totalData[board].IMU16 = incomingReadings.IMU16;
}

void setup() {
  Serial.begin(230400);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register thigh as peer
  memcpy(peerInfo.peer_addr, broadcastAddressThigh, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register shank as peer
  memcpy(peerInfo.peer_addr, broadcastAddressShank, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop(){
    // Simulate a non-blocking task with millis
    unsigned long currentMillis = millis();
    static unsigned long previousMillis = 0;
    const long interval = 20;  // ms interval
  
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      for (int i=0; i<=1; i++) {
        Serial.print(totalData[i].ID); Serial.print(",");
        Serial.print(totalData[i].FSR1); Serial.print(",");
        Serial.print(totalData[i].FSR2); Serial.print(",");
        Serial.print(totalData[i].FSR3); Serial.print(",");
        Serial.print(totalData[i].FSR4); Serial.print(",");
        Serial.print(totalData[i].FSR5); Serial.print(",");
        Serial.print(totalData[i].FSR6); Serial.print(",");
        Serial.print(totalData[i].FSR7); Serial.print(",");
        Serial.print(totalData[i].FSR8); Serial.print(",");
        Serial.print(totalData[i].IMU1); Serial.print(",");
        Serial.print(totalData[i].IMU2); Serial.print(",");
        Serial.print(totalData[i].IMU3); Serial.print(",");
        Serial.print(totalData[i].IMU4); Serial.print(",");
        Serial.print(totalData[i].IMU5); Serial.print(",");
        Serial.print(totalData[i].IMU6); Serial.print(",");
        Serial.print(totalData[i].IMU7); Serial.print(",");
        Serial.print(totalData[i].IMU8); Serial.print(",");
        Serial.print(totalData[i].IMU10); Serial.print(",");
        Serial.print(totalData[i].IMU10); Serial.print(",");
        Serial.print(totalData[i].IMU11); Serial.print(",");
        Serial.print(totalData[i].IMU12); Serial.print(",");
        Serial.print(totalData[i].IMU13); Serial.print(",");
        Serial.print(totalData[i].IMU14); Serial.print(",");
        Serial.print(totalData[i].IMU15); Serial.print(",");
        Serial.println(totalData[i].IMU16);
      }
  }
}