//#define DEBUG

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <WiFi.h>
#include <esp_now.h>

typedef enum {
  NUM_FSR = 8,
  NUM_IMU = 16, // 3xGyro, 3xAcc, 3xMagnetometer,  4xQuaternion, 3xGravity, 3xCalibration_Status 

  FSR_SAMPLERATE_DELAY_NS = 1000,
  BNO055_SAMPLERATE_DELAY_NS = 10000
} Constants;

typedef enum {
  GREEN,
  RED
} LEDColor;

typedef float DataType;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

unsigned long init_delay = micros();
signed long total_delay = 0;

// Wifi Stuff, write below the receiver ESP's mac address:
uint8_t broadcastAddress[] = {0xA4, 0xCF, 0x12, 0x81, 0x87, 0x1C};

typedef struct struct_message {
    char id;
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

String success;
struct_message FSRReadings;
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  #ifdef DEBUG
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
  #endif
}

//timing stuff
unsigned long start_time;
unsigned long time_elapsed;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(230400);
  WiFi.mode(WIFI_MODE_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

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

  if (!Wire.begin(23, 22)) {
    Serial.println("Error starting wire");
    while (1);
  }

  analogReadResolution(9);

  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }

  // Set the BNO055 sensor mode to NDOF
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
  delay(1000);

  FSRReadings.id = 's';

  start_time = micros();
}

void loop()
{
  //get time
  time_elapsed = micros() - start_time;

  FSRReadings.FSR1 = (DataType) constrain((analogRead(32)), 0, 511);
  FSRReadings.FSR2 = (DataType) constrain((analogRead(33)), 0, 511);
  FSRReadings.FSR3 = (DataType) constrain((analogRead(34)), 0, 511);
  FSRReadings.FSR4 = (DataType) constrain((analogRead(35)), 0, 511);
  FSRReadings.FSR5 = (DataType) constrain((analogRead(36)), 0, 511);
  FSRReadings.FSR6 = (DataType) constrain((analogRead(37)), 0, 511);
  FSRReadings.FSR7 = (DataType) constrain((analogRead(38)), 0, 511);
  FSRReadings.FSR8 = (DataType) constrain((analogRead(39)), 0, 511);

  sensors_event_t gyro, linearAccelData;
  bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  // calibration variables initialization
  uint8_t system_cal, gyro_cal, accel_cal, mag_cal;
  system_cal = gyro_cal = accel_cal = mag_cal = 0;
  
  imu::Quaternion quat = bno.getQuat();
  quat.normalize();
  imu::Vector<3> read_data_gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  imu::Vector<3> read_data_gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> read_data_liacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> read_euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
  int16_t  gyro_cal2, accel_cal2, mag_cal2; //system_cal2,
  gyro_cal2=gyro_cal;
  accel_cal2=accel_cal;
  mag_cal2=mag_cal;

  FSRReadings.IMU1 = read_data_gravity.x(); //9
  FSRReadings.IMU2 = read_data_gravity.y(); //10
  FSRReadings.IMU3 = read_data_gravity.z(); //11

  FSRReadings.IMU4 = read_data_gyro.x(); //12
  FSRReadings.IMU5 = read_data_gyro.y(); //13
  FSRReadings.IMU6 = read_data_gyro.z(); //14

  imu::Quaternion read_data_quat = bno.getQuat();
  FSRReadings.IMU7 = read_data_quat.w(); //15
  FSRReadings.IMU8 = read_data_quat.x(); //16
  FSRReadings.IMU9 = read_data_quat.y(); //17
  FSRReadings.IMU10 = read_data_quat.z(); //18

  FSRReadings.IMU11 = read_data_liacc.x(); //19
  FSRReadings.IMU12 = read_data_liacc.y(); //20
  FSRReadings.IMU13 = read_data_liacc.z(); //21

  //ZYX Tait-Bryan angle
  FSRReadings.IMU14 = read_euler.x();
  FSRReadings.IMU15 = read_euler.y();
  FSRReadings.IMU16 = read_euler.z();

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &FSRReadings, sizeof(FSRReadings));
  
  #ifdef DEBUG
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  #endif

  // variable delay by 1000us or 1000Hz
  int delay_diff = micros() - start_time - time_elapsed;
  if (delay_diff < 20000) {
    delayMicroseconds(20000 - delay_diff);
  }
}