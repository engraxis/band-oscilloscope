/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE"
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second.
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_gap_ble_api.h"
#include <Esp.h>
#include <esp32-hal-bt.h>
#include "bt.h"

BLEServer *pServer = NULL;
BLECharacteristic * p_TX_FSR_Characteristic;
BLECharacteristic * p_TX_IMU_Characteristic;
BLECharacteristic * p_TX_Steps_Characteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
//uint8_t txValue = 254;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

// #define SERVICE_UUID           "6E400005-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
// #define CHARACTERISTIC_UUID_TX_FSR "6E400006-B5A3-F393-E0A9-E50E24DCCA9E"
// #define CHARACTERISTIC_UUID_TX_IMU "6E400007-B5A3-F393-E0A9-E50E24DCCA9E"
// #define CHARACTERISTIC_UUID_RX "6E400008-B5A3-F393-E0A9-E50E24DCCA9E"

// #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
// #define CHARACTERISTIC_UUID_TX_FSR "6E400009-B5A3-F393-E0A9-E50E24DCCA9E"
// #define CHARACTERISTIC_UUID_TX_IMU "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
// #define CHARACTERISTIC_UUID_RX "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"

#define SERVICE_UUID           "42426001-0bfe-4adb-aefb-5b5708b77691" // UART service UUID
#define CHARACTERISTIC_UUID_TX_FSR "42426002-0bfe-4adb-aefb-5b5708b77691"
#define CHARACTERISTIC_UUID_TX_IMU "42426003-0bfe-4adb-aefb-5b5708b77691"
#define CHARACTERISTIC_UUID_TX_STEPS "42426004-0bfe-4adb-aefb-5b5708b77691"
#define CHARACTERISTIC_UUID_RX "42426005-0bfe-4adb-aefb-5b5708b77691"

#define BLE_MAX_DATA_SIZE 20



class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};

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

static void checkResult(esp_err_t err, const __FlashStringHelper* s)
{
	if (err != ESP_OK) {
		Serial.printf("[ERR %08x] ", err);
		Serial.println(s);
	}
}


void initBLEService() {
  Serial.begin(115200);

  initBluetooth();
  const uint8_t* point = esp_bt_dev_get_address();

  char bt_name[13] = {};
  sprintf(bt_name, "AALBAND_%02X%02X", (int)point[4], (int)point[5]);
  //Serial.println(bt_name);

  // Create the BLE Device
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
                                        BLECharacteristic::PROPERTY_READ |
										BLECharacteristic::PROPERTY_NOTIFY
									);

  p_TX_IMU_Characteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX_IMU,
                                        BLECharacteristic::PROPERTY_READ |
										BLECharacteristic::PROPERTY_NOTIFY
									);

  p_TX_Steps_Characteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX_STEPS,
										BLECharacteristic::PROPERTY_READ
									);

  p_TX_FSR_Characteristic->addDescriptor(new BLE2902());
  p_TX_IMU_Characteristic->addDescriptor(new BLE2902());
  p_TX_Steps_Characteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  // pServer->getAdvertising()->start();

//   BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
//   pAdvertising->addServiceUUID(SERVICE_UUID);
//   pAdvertising->setScanResponse(true);
//   pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
//   pAdvertising->setMinPreferred(0x12);

//   BLEDevice::startAdvertising();

//   Serial.print(bt_name);
//   Serial.println("\t Waiting a client connection to notify...");

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  Serial.print(bt_name);
  Serial.println("\t Waiting a client connection to notify...");

//   esp_ble_adv_params_t advParams = { 0 };
//   advParams.channel_map = ADV_CHNL_ALL;
//   checkResult(esp_ble_gap_start_advertising(&advParams), F("Failed to start advertising."));

//   esp_ble_adv_data_t advData = { 0 };
//   advData.flag = ESP_BLE_ADV_FLAG_NON_LIMIT_DISC;
//   advData.include_name = true;
//   advData.min_interval = 0x0100;
//   advData.max_interval = 0x0100;
//   checkResult(esp_ble_gap_config_adv_data(&advData), F("Failed to configure advertisement data."));

//   esp_ble_adv_params_t advParams = { 0 };
//   advParams.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
//   advParams.adv_int_max = 0x0100;
//   advParams.adv_int_min = 0x0100;
//   advParams.adv_type = ADV_TYPE_NONCONN_IND;
//   advParams.channel_map = ADV_CHNL_ALL;
//   advParams.own_addr_type = BLE_ADDR_TYPE_PUBLIC;

//   checkResult(esp_ble_gap_start_advertising(&advParams), F("Failed to start advertising."));

    // memcpy(arr_joint,arr1, sizeof(arr1));
    // memcpy(arr_joint + sizeof(arr1), arr2, sizeof(arr2));
}

// void publishBLEdata(uint8_t* txValue, size_t length, char mode, uint16_t delay_ns = 3300)
void publishBLEdata(uint8_t* txValue, size_t length, char mode)
{

    BLECharacteristic * characteristic;

    switch (mode)
    {
    case 'f':
        characteristic = p_TX_FSR_Characteristic;
        break;

    case 'i':
        characteristic = p_TX_IMU_Characteristic;
        break;

    default:
        characteristic = p_TX_FSR_Characteristic;
        break;
    }

    if (deviceConnected) {

        characteristic->setValue(txValue, length);
        characteristic->notify();
        // delayMicroseconds(delay_ns);

        // if (length > BLE_MAX_DATA_SIZE)
        // {
        //     for (int i = 0; i < (length / BLE_MAX_DATA_SIZE); i++)
        //     {
        //         characteristic->setValue(txValue + (BLE_MAX_DATA_SIZE * i), BLE_MAX_DATA_SIZE);
        //         characteristic->notify();
        //         delayMicroseconds(delay_ns);
        //     }
        //     int remainder = length % BLE_MAX_DATA_SIZE;
        //     if (remainder > 0)
        //     {
        //         characteristic->setValue(txValue + (length - remainder), remainder);
        //         characteristic->notify();
        //         delayMicroseconds(delay_ns);
        //     }
        // }
        // else
        // {
        //     characteristic->setValue(txValue, length);
        //     characteristic->notify();
        //     delayMicroseconds(delay_ns);

        // }
	}

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}

void publishBLEdata(uint16_t& steps, uint16_t delay_ns = 3300)
{
    p_TX_Steps_Characteristic->setValue(steps);
    delayMicroseconds(delay_ns);
}

