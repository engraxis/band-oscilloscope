#include "BluetoothSerial.h"
// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"

#include "commandsINT.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


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

BluetoothSerial SerialBT;

void startBTConnection() {
  Serial.begin(115200);

  initBluetooth();
  const uint8_t* point = esp_bt_dev_get_address();

  char bt_name[13] = {};
  sprintf(bt_name, "AALBAND_%02X%02X", (int)point[4], (int)point[5]);
  //Serial.println(bt_name);

  // SerialBT.begin(bt_name, true); master mode?
  SerialBT.begin(bt_name);


  Serial.print(bt_name);
  Serial.println("\t Band initialized in BTC mode...");
}


void publish(uint8_t* data, size_t length)
{
    SerialBT.write(data, length);
}

//void writeCMD(enum Command cmd)
void writeCMD(Command cmd)
{
  uint8_t* Command = (uint8_t*) &cmd;
  SerialBT.write(Command, sizeof(uint8_t));
}

Command readCMD()
{
  return (Command) SerialBT.read();
}


// void readCMD(uint8_t* cmd)
// {
//     *cmd = SerialBT.read();
// }

void endConnection()
{
    SerialBT.end();
}
