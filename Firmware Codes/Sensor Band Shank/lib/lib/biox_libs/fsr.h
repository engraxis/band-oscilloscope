// Create timer interrupt of 10msec for IMU reading

/* Firmware for ESP32 board
  - Initiate connection with Python Script (PS)
  - IF data_req is received start sending data continuously
  - If disconnect recv  - restart main loop

  - Connection close/re-open works properly
  - Data rate too slow with 126 hz on BT
*/

#include "BluetoothSerial.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"

// #include "commands.h"
#include "../include/commandsINT.h"
#include "../include/BLEServer.h"

#define DEFAULT_INPUT_VOLTAGE 1
// #define ENABLE_SHUTDOWN
//#define ENABLE_CHARGE_INDICATION

// #define DISABLE_IMU
// #define DISABLE_FSR

// typedef float DataType;
typedef short int DataType;


#define NUM_FSR 8
//#define NUM_IMU 13
#define NUM_IMU 14 // Includes step counter

#define SERIAL_USB Serial
#define SERIAL_BT SerialBT

#define SERIAL SERIAL_BT
#define BAUDRATE 9600

DataType net_fsr = 0;

//DataType data[NUM_FSR + NUM_IMU] = {9901, 9902, 9903, 9904, 9905, 9906, 9907, 9908, 9909, 9910, 9911, 9912, 9913, 9914, 9915, 9916, 9917, 9918, 9919, 9920, 9921};
DataType data[NUM_FSR + NUM_IMU] = {9901, 9902, 9903, 9904, 9905, 9906, 9907, 9908, 9909, 9910, 9911, 9912, 9913, 9914, 9915, 9916, 9917, 9918, 9919, 9920, 9921, 9922};

byte buff[(NUM_FSR + NUM_IMU)*sizeof(DataType)] = {};


#define NUM_data 20
byte buff_data[NUM_data];

float imu_euler_offset[3]={0,0,0};

#define STEP_SAMPLES 100
float acc_samples[3][STEP_SAMPLES] = {};
float total_vector[STEP_SAMPLES] = {};
float total_vector_avg[STEP_SAMPLES] = {};
int sample_counter = 0;
DataType steps, flag = 0;
float threshold = 6;

uint8_t fsr_temp[8] = {0,0,0,0,0,0,0,0};
uint8_t fsr_total[16] = {};


//===================================================================================
// TYPECASTS
//===================================================================================

bool initBluetooth()
{
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }

  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }

  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }

}


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define BUTTON_PIN_BITMASK 0x000000000 // 2^0 in hex
RTC_DATA_ATTR int bootCount = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
BluetoothSerial SERIAL_BT;


////////////////////////////
//                        //
//      time related      //
//        constants       //
////////////////////////////
float current_time = 0.0;
//float imu_read_time = 10000; // is in micro seconds which is 10 milli second
float imu_read_time = 25000; // is in micro seconds which is 10 milli second
float imu_start_timer = 0;
float imu_end_timer = 0;
float send_time = 500; // is in micro seconds which is 5 milli second
float timer_send = 0.0;
float sleep_time = 20000; // is in micro seconds which is 5 milli second
float pre_sleep_time = 0.0;
float charge_time = 1500; // is in micro seconds which is 2 second
float pre_charge_time = 0; //

float transmit_time = 500; // is in micro seconds which is 2 second
float pre_transmit_time = 0; //

float look_for_off = 0;
////////////////////////////
//                        //
//  Serial Communication  //
//        constants       //
////////////////////////////
//char command_received = 'n';//
Command command_received = NONE;
// char send_data = 'n';//
bool send_data = false;
bool calibrate_amplifier = false;
bool calibration_status = true;


////////////////////////////
//                        //
//         I/O pins       //
//                        //
////////////////////////////
const int red_pin =  13;
const int grn_pin =  15;
const int dac_pin =  26;
int input_voltage = 10;
const int charge_id =  25;
int val = 0;

bool first_loop = true;

unsigned long timer = 0;
long imu_loopTime = 8000;   // microseconds

bool conn_established = false;

void setup()
{
  pinMode(charge_id, INPUT);
  pinMode(red_pin, OUTPUT);
  pinMode(grn_pin, OUTPUT);
  ////////////////////////////
  //                        //
  //    Initialize setup    //
  //        files           //
  ////////////////////////////

  Wire.begin(23, 22);
  SERIAL_USB.begin(BAUDRATE);

  initBluetooth();
  const uint8_t* point = esp_bt_dev_get_address();

  char bt_name[13] = {};
  sprintf(bt_name, "AALBAND_%02X%02X", (int)point[4], (int)point[5]);
  Serial.println(bt_name);

  //SERIAL_BT.begin(bt_name);
  ///////////////////////////////////////
  // Create the BLE Device
  //BLEDevice::init("UART Service");
  BLEDevice::init(bt_name);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
//   p_TX_FSR_Characteristic = pService->createCharacteristic(
// 										CHARACTERISTIC_UUID_TX_FSR,
//                                         BLECharacteristic::PROPERTY_INDICATE
// 									);

  p_TX_FSR_Characteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX_FSR,
										BLECharacteristic::PROPERTY_NOTIFY
									);

  p_TX_FSR_Characteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  //////////////////////////////////////

  analogReadResolution(9);

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 0);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    SERIAL_USB.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  bno.setExtCrystalUse(true);

  dacWrite(dac_pin, input_voltage);

  //// Get the current euler angle as an offset. Used to initialize future euler angles as 0
//   float offset_x_total = 0, offset_y_total = 0, offset_z_total = 0;
//   for(int i=0; i<10; i++)
//   {
//     sensors_event_t euler_offset;
//     bno.getEvent(&euler_offset, Adafruit_BNO055::VECTOR_EULER);
//     offset_x_total += (&euler_offset)->orientation.x;
//     offset_y_total += (&euler_offset)->orientation.y;
//     offset_z_total += (&euler_offset)->orientation.z;
//     delay(100);
//   }
//   imu_euler_offset[0] = offset_x_total/10;
//   imu_euler_offset[1] = offset_y_total/10;
//   imu_euler_offset[2] = offset_z_total/10;


  //   ////////////////////////////
  //   //                        //
  //   //     finish setup       //
  //   //        files           //
  //   ////////////////////////////
  start_up();
  timer = micros();

}

void loop()
{
    #ifdef ENABLE_CHARGE_INDICATION
        val = digitalRead(charge_id);
    #else
        val = 1;
    #endif

  if (val == 0)
  {
    charge_indication();
  }
  else
  {
    digitalWrite(red_pin, LOW);

    shut_down_condition();

    if (SERIAL_BT.available())
    {
      command_received = (Command) SERIAL_BT.read();
    }
    if (SERIAL_USB.available())
    {
      command_received = (Command) SERIAL_USB.read();
    }
    switch (command_received)
    {
      case INIT:
        {
          command_received = NONE;
          conn_established = true;
          write_cmd(ACK);
          SERIAL.flush();
        //   delay(100);
          break;
        }
    }
  }
  conn_established = true;
  bool first_fsr_frame = true;
  while (conn_established)
  {
    #ifdef DISABLE_FSR
        data[0] = 9901;
        data[1] = 9902;
        data[2] = 9903;
        data[3] = 9904;
        data[4] = 9905;
        data[5] = 9906;
        data[6] = 9907;
        data[7] = 9908;
    #else
        readFSR();
    #endif
    if (first_fsr_frame)
    {
        memcpy(fsr_total,fsr_temp, sizeof(fsr_temp));
        first_fsr_frame = false;
    }
    else
    {
        memcpy(fsr_total + sizeof(fsr_temp),fsr_temp, sizeof(fsr_temp));
        first_fsr_frame = true;
    }

    // Read IMU every imu_read_time micros, default 10000=10 millis
    if (micros() - timer >= imu_read_time)
    {
      readIMU();
      timer = micros();
      if(sample_counter > STEP_SAMPLES - 1)
      {
          // Calculate step
          sample_counter = 0;
      }
      else
      {
          acc_samples[0][sample_counter] = ((float)data[14])/100;
          acc_samples[1][sample_counter] = ((float)data[15])/100;
          acc_samples[2][sample_counter] = ((float)data[16])/100;

          total_vector[sample_counter] += acc_samples[0][sample_counter] * acc_samples[0][sample_counter];
          total_vector[sample_counter] += acc_samples[1][sample_counter] * acc_samples[1][sample_counter];
          total_vector[sample_counter] += acc_samples[2][sample_counter] * acc_samples[2][sample_counter];
          total_vector[sample_counter] = sqrt(total_vector[sample_counter]);

          total_vector_avg[sample_counter] = (total_vector[sample_counter] + total_vector[sample_counter - 1]) / 2 ;

        //   SERIAL_USB.print("average: ");
        //   SERIAL_USB.println(total_vector_avg[sample_counter]);

            if (total_vector_avg[sample_counter]> threshold && flag == 0)
            {
                steps = steps + 1;
                flag = 1;
            }
            else if (total_vector_avg[sample_counter] > threshold && flag == 1)
            {
                // Don't Count
            }
            if (total_vector_avg[sample_counter]< threshold   && flag == 1)
            {
                flag = 0;
            }
            if (steps < 0) {
                steps = 0;
            }

            // SERIAL_USB.print("Steps: ");
            // SERIAL_USB.print(steps);
            data[21] = (DataType) steps;

          sample_counter++;
      }

    }

    if (deviceConnected)
    {

        p_TX_FSR_Characteristic->setValue(fsr_total, sizeof(fsr_total));    // Must send an array of uint8_t
        p_TX_FSR_Characteristic->notify();

		delay(1); // bluetooth stack will go into congestion, if too many packets are sent
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }


    // if (SERIAL_BT.available())
    // {
    //   command_received = (Command) SERIAL_BT.read();
    // }
    // if (SERIAL_USB.available())
    // {
    //   command_received = (Command) SERIAL_USB.read();
    // }

    switch (command_received)
    {
    //   case INIT:
    //     {
    //       command_received = NONE;
    //       conn_established = true;
    //       write_cmd(ACK);
    //       SERIAL.flush();
    //       break;
    //     }
      case DISCONNECT:
        {
          send_data = false;
          conn_established = false;
          command_received = NONE;
          shut_down();
          esp_deep_sleep_start();
          break;
        }
      case START_TRANSMISSION:
        {
          command_received = NONE;
          send_data = true;
          break;
        }
      case STOP_TRANSMISSION:
        {
          command_received = NONE;
          send_data = false;
          digitalWrite(grn_pin, HIGH);
          break;
        }
      case GAIN_RESET:
        {
          command_received = NONE;
          input_voltage = DEFAULT_INPUT_VOLTAGE;
          dacWrite(dac_pin, input_voltage);
          calibration_status = true;
          calibrate_amplifier = false;
          break;
        }
      case CALIBRATE:
        {
          command_received = NONE;
          calibrate_amplifier = true;
          send_data = true;
          break;
        }
    }

    if (send_data)
    {
      if (micros() - timer_send >= send_time)
      {
        timer_send = micros();
        // sendData_old();
        sendData();
        transmitt_indication();
      }
    }
    else
    {
      digitalWrite(grn_pin, HIGH);
    }
    if (calibration_status)
    {
      if (calibrate_amplifier)
      {
        calibrate();
      }
    }
    shut_down_condition();
    #ifdef ENABLE_CHARGE_INDICATION
        val = digitalRead(charge_id);
    #else
        val = 1;
    #endif
    if (val == 0)
    {
      send_data = false;
      conn_established = false;
      command_received = NONE;
      digitalWrite(grn_pin, HIGH);
      digitalWrite(red_pin, HIGH);
      charge_indication();
    }
  }

}

////////**********//////////
////////////////////////////
//                        //
//      Additional        //
//      Functions         //
////////////////////////////
///////***********//////////


void readFSR()
{
    // data[0] = (DataType) constrain((analogRead(32) / 2.1), 0, 254);
    // data[1] = (DataType) constrain((analogRead(33) / 2.1), 0, 254);
    // data[2] = (DataType) constrain((analogRead(34) / 2.1), 0, 254);
    // data[3] = (DataType) constrain((analogRead(35) / 2.1), 0, 254);
    // data[4] = (DataType) constrain((analogRead(36) / 2.1), 0, 254);
    // data[5] = (DataType) constrain((analogRead(37) / 2.1), 0, 254);
    // data[6] = (DataType) constrain((analogRead(38) / 2.1), 0, 254);
    // data[7] = (DataType) constrain((analogRead(39) / 2.1), 0, 254);

    memset(fsr_temp, 0, sizeof(fsr_temp)); // Clear the buffer
    fsr_temp[0] = (uint8_t) constrain((analogRead(32) / 2.1), 0, 254);
    fsr_temp[1] = (uint8_t) constrain((analogRead(33) / 2.1), 0, 254);
    fsr_temp[2] = (uint8_t) constrain((analogRead(34) / 2.1), 0, 254);
    fsr_temp[3] = (uint8_t) constrain((analogRead(35) / 2.1), 0, 254);
    fsr_temp[4] = (uint8_t) constrain((analogRead(36) / 2.1), 0, 254);
    fsr_temp[5] = (uint8_t) constrain((analogRead(37) / 2.1), 0, 254);
    fsr_temp[6] = (uint8_t) constrain((analogRead(38) / 2.1), 0, 254);
    fsr_temp[7] = (uint8_t) constrain((analogRead(39) / 2.1), 0, 254);
}


void readIMU()
{
  unsigned long imu_timer = 0;
  imu_start_timer = micros();

#ifndef DISABLE_IMU
    sensors_event_t gravity;
    bno.getEvent(&gravity, Adafruit_BNO055::VECTOR_GRAVITY);
    data[8] = (DataType) ((&gravity)->acceleration.x  * 100);
    data[9] = (DataType) ((&gravity)->acceleration.y  * 100);
    data[10] = (DataType) ((&gravity)->acceleration.z  * 100);


    sensors_event_t gyro;
    bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);
    data[11] = (DataType) ((&gyro)->gyro.x  * 100);
    data[12] = (DataType) ((&gyro)->gyro.y  * 100);
    data[13] = (DataType) ((&gyro)->gyro.z  * 100);


    sensors_event_t linacc;
    bno.getEvent(&linacc, Adafruit_BNO055::VECTOR_LINEARACCEL);
    data[14] = (DataType) ((&linacc)->acceleration.x  * 100);
    data[15] = (DataType) ((&linacc)->acceleration.y  * 100);
    data[16] = (DataType) ((&linacc)->acceleration.z  * 100);

    imu::Quaternion quat = bno.getQuat();

    data[17] = (DataType) (quat.w() * 10000);
    data[18] = (DataType) (quat.x() * 10000);
    data[19] = (DataType) (quat.y() * 10000);
    data[20] = (DataType) (quat.z() * 10000);
#endif


    ////////////////////////
    // sensors_event_t euler;
    // bno.getEvent(&euler, Adafruit_BNO055::VECTOR_EULER);
    // data[17] = (DataType) ((&euler)->orientation.x * 10);
    // data[18] = (DataType) ((&euler)->orientation.y * 10);
    // data[19] = (DataType) ((&euler)->orientation.z * 10);

    // x- Roll is positive and increasing when moving downward. -90 degrees <= roll <= 90 degrees */
    // y- Pitch is positive and increasing when moving upwards. -180 degrees <= pitch <= 180 degrees)

    // sensors_event_t euler;
    // bno.getEvent(&euler, Adafruit_BNO055::VECTOR_EULER);
    // // data[17] = (DataType) (((&euler)->orientation.x - imu_euler_offset[0]) * 10);
    // // data[18] = (DataType) (((&euler)->orientation.y - imu_euler_offset[1]) * 10);
    // // data[19] = (DataType) (((&euler)->orientation.z - imu_euler_offset[2]) * 10);
    // data[17] = (DataType) (((&euler)->orientation.roll - imu_euler_offset[0]) * 10);
    // data[18] = (DataType) (((&euler)->orientation.pitch - imu_euler_offset[1]) * 10);
    // data[19] = (DataType) (((&euler)->orientation.heading - imu_euler_offset[2]) * 10);

  imu_end_timer = micros();
  if ((imu_end_timer - imu_start_timer) < imu_loopTime)
  {
    delayMicroseconds(imu_loopTime - (imu_end_timer - imu_start_timer));
  }
}


void sendData()
{
  uint16_t count = 0;
  for (uint8_t i = 0; i < (NUM_FSR + NUM_IMU); i++)
  {
    byte* byteData = (byte*)(data + i);
    for (uint8_t j = 0; j < sizeof(DataType); j++)
    {
      buff[count + j] = byteData[j];
    }
    count += sizeof(DataType); // sizeof(float)
  }
  SERIAL.write(buff, (NUM_FSR + NUM_IMU)*sizeof(DataType));

  memset(buff, 0, sizeof(buff)); // Clear the buffer
  //memset(data,0,sizeof(data));  // Clear data array
  //delay(10);
}


////////////////////////////
//                        //
//   condition for shut   //
//          down          //
////////////////////////////
void shut_down_condition()
{

#ifdef ENABLE_SHUTDOWN
  readFSR();
  net_fsr = (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7]) / (8);
  if (digitalRead(0) == 0)
  {
    shut_down();
    esp_deep_sleep_start();
  }

  if (net_fsr < (input_voltage * 0.85 * 2.1 + 15))
  {
    if (pre_sleep_time == 0)
    {
      pre_sleep_time = millis();
    }
    else
    {
      current_time = millis();
      if ((current_time - pre_sleep_time) > sleep_time)
      {
        shut_down();
        esp_deep_sleep_start();
      }
    }
  }
  else
  {
    pre_sleep_time = 0;
  }
#endif

}

////////////////////////////
//                        //
//     initiate shut      //
//   down and start up    //
////////////////////////////
void shut_down()
{
  for (int i = 0; i <= 5; i++) {
    digitalWrite(red_pin, LOW);
    delay(500);
    digitalWrite(red_pin, HIGH);
    delay(500);
  }
}

void start_up()
{
  for (int i = 0; i <= 1; i++) {
    digitalWrite(red_pin, LOW);
    digitalWrite(grn_pin, LOW);
    delay(1500);
    digitalWrite(red_pin, HIGH);
    digitalWrite(grn_pin, HIGH);
    delay(500);
  }
}

////////////////////////////
//                        //
//       initiate         //
//     calibration        //
////////////////////////////
void calibrate()
{
  for (int i = 0; i <= 7; i++)
  {
    if (data[i] > 425)
    {
      calibration_status = false;
      calibrate_amplifier = false;
    }
  }
  if (calibration_status)
  {
    input_voltage = input_voltage + 1;
    dacWrite(dac_pin, input_voltage);
  }
}


////////////////////////////
//                        //
//        charge          //
//      indication        //
////////////////////////////
void charge_indication()
{
    current_time = millis();
    if ((current_time - pre_charge_time) > charge_time)
    {
        digitalWrite(red_pin, LOW);
        delay(500);
        digitalWrite(red_pin, HIGH);
        pre_charge_time = current_time;
    }
}

////////////////////////////
//                        //
//    data transmitt      //
//      indication        //
////////////////////////////
void transmitt_indication()
{
  current_time = millis();
  if ((current_time - pre_transmit_time) > transmit_time)
  {
    digitalWrite(grn_pin, !digitalRead(grn_pin));
    pre_transmit_time = current_time;
  }
}


void write_cmd(enum Command cmd)
{
  uint8_t* Command = (uint8_t*) &cmd;
  SERIAL.write(Command, sizeof(uint8_t));
//   char* Command = (char*) &cmd;
//   SERIAL.println(Command);
}


Command read_cmd()
{
  return (Command) SERIAL.read();
}

//// functionalities to add in the code
//1- use charge pin to indicate if it is charging--
//2- Command to start sarting and stoping data transmission--
//3- Command to increase or decrease input voltage--
//4- sleep mode when FSR sensors are silent for more then 10 seconds--.


// BATTERY NOT Attached - code stuck in charging indication  - RED LED BLNK
