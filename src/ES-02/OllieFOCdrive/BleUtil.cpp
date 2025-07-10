#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include "BleUtil.h"
#include "RobotUtil.h"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

BleDataTypDef ble_rx;



// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400011-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400012-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400013-B5A3-F393-E0A9-E50E24DCCA9E"


void ble_rx_data_clear(void);

void ble_cmd_maneuver_processing(void);

void ble_cmd_wifi_processing(void);

void ble_cmd_send_device_info(void);

void ble_cmd_restart(void);


class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t *rxData = pCharacteristic->getData();

      if (pCharacteristic->getLength() == 20) {
        int i;
        for (i = 0; i < 20; i++) {
          ble_rx.frame[i] = *(rxData + i);            //Transfer data
        }
        if (ble_rx.remaining_pack == 0) {
          ble_rx.remaining_pack = ble_rx.frame[3];
        }


        //Status setting and acquisition commands
        ble_rx.state = BLE_STATE_RECEIVE_OK;
        ble_rx.cmd = ble_rx.frame[2];
      }
    }
};

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer) {
      deviceConnected = false;
      // Restart the broadcast to allow for reconnection
      pServer->getAdvertising()->start();
    }
};

static void dev_name_build(char ble_name[]) {
  uint8_t mac[7];
  esp_read_mac(mac, ESP_MAC_BT);
  mac[6] = 0;
  char i;

  for (i = 0; i < 6; i++) //Convert the mac address to contain only 0-9/a-z
  {
    mac[i] = mac[i] % 36; // 10+26=36

    if (mac[i] <= 9) mac[i] = mac[i] + '0'; //0-9
    else if (mac[i] <= 35) mac[i] = mac[i] - 10 + 'a'; //a-z
  }

  sprintf(ble_name, "%s%s", robot_model_base, mac);
  Serial.printf("get_dev_name:");
  Serial.println(ble_name);

  sprintf(ble_name, "%s%s", robot_model_base, "test");
  Serial.printf(ble_name);
  Serial.printf("\r\n");
}

void ble_init() {
  char ble_name[20] = {0};
  dev_name_build(ble_name);
  // Create the BLE Device
  BLEDevice::init(ble_name);
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
          CHARACTERISTIC_UUID_TX,
          BLECharacteristic::PROPERTY_NOTIFY
  );

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
          CHARACTERISTIC_UUID_RX,
          BLECharacteristic::PROPERTY_WRITE
  );

  pRxCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  ble_rx_data_clear();
}


void ble_send_data(uint8_t *data, uint8_t len) {
  pTxCharacteristic->setValue(data, len);  //reply
  pTxCharacteristic->notify();
}

void ble_rx_data_clear() {
  int i;
  for (i = 0; i < 50; i++) {
    ble_rx.data[i] = 0;
  }
  ble_rx.index = 0;
}

void ble_rx_data_add(uint8_t *data, uint8_t len) {
  int i;
  for (i = 0; i < len; i++) {
    ble_rx.data[ble_rx.index] = *(data + i);
    ble_rx.index++;
    if (ble_rx.index > (BLE_DATA_SIZE - 1)) {
      ble_rx.index = BLE_DATA_SIZE - 1;
    }
  }
}

void ble_loop(void) {
  //The BLE data reception has been completed
  if (ble_rx.state == BLE_STATE_RECEIVE_OK) {
    //BLE reply
    ble_send_data((uint8_t *) ble_rx.frame, 20);
    //Merge data packets
    if (ble_rx.remaining_pack > 0) {
      ble_rx_data_add(&ble_rx.frame[5], 15);
      ble_rx.remaining_pack--;
      //There is still BLE data to be received and status changes
      ble_rx.state = BLE_STATE_RECEIVE_WAIT;
    } else //Process data packets
    {
      //Merge the last package of data；if there is only one package of data, that is also the last one
      ble_rx_data_add(&ble_rx.frame[5], 15);

      switch (ble_rx.cmd) {
        case CMD_MANEUVER: {
          ble_cmd_maneuver_processing();
        }
          break;
        case CMD_WIFI: {
          ble_cmd_wifi_processing();
        }
          break;
        case CMD_DEVICE_INFO: {
          ble_cmd_send_device_info();
        }
          break;
        case CMD_RESTART: {
          ble_cmd_restart();
        }
          break;
      }
      //The data processing is completed and the status is changed to idle
      ble_rx.state = BLE_STATE_IDLE;
      ble_rx_data_clear();
    }
  }
}

//
void ble_cmd_maneuver_processing(void) {
  CmdManeuverTypDef *ble_maneuver;
  ble_maneuver = (CmdManeuverTypDef *) ble_rx.data;
// The processing logic of webSocket is used, so JSON needs to be generated
  StaticJsonDocument<300> doc;
  char jsonBuffer[300];
  doc["roll"] = ble_maneuver->roll;
  doc["height"] = ble_maneuver->height;

  int16_t linear;
  linear = (int16_t)((ble_maneuver->linear_H << 8) | ble_maneuver->linear_L);
  doc["linear"] = linear;
  doc["angular"] = ble_maneuver->angular;
  doc["stable"] = ble_maneuver->stable;
  doc["mode"] = "basic";
  switch (ble_maneuver->dir) {
    case 0:
      doc["dir"] = "stop";
      break;
    case 1:
      doc["dir"] = "jump";
      break;
    case 2:
      doc["dir"] = "forward";
      break;
    case 3:
      doc["dir"] = "back";
      break;
    case 4:
      doc["dir"] = "left";
      break;
    case 5:
      doc["dir"] = "right";
      break;
  }
  doc["joy_y"] = ble_maneuver->joy_y;
  doc["joy_x"] = ble_maneuver->joy_x;

  serializeJson(doc, jsonBuffer);

  String mode_str = doc["mode"];
  if (mode_str == "basic") {
    rp.parseBasic(doc);
  }
  ble_rx.state = BLE_STATE_IDLE;
}

void ble_cmd_wifi_processing(void) {
  uint8_t ssid[50] = {0};
  uint8_t password[30] = {0};
  uint8_t TAB = 0;
  uint8_t i, j = 0;

  for (i = 0; i < BLE_DATA_SIZE; i++) {
    //Determine whether the wifi name has ended
    if (ble_rx.data[i] == '\t') {
      TAB = 1;
      ssid[i] = 0;//Fill the end with 0
      i++;
    }
    if (TAB == 0) {
      //Judge the validity of characters
      if (ble_rx.data[i] >= ' ' && ble_rx.data[i] <= '~') {
        ssid[i] = ble_rx.data[i];
      } else {
        Serial.printf("cmd_wifi Err, data[%d]", i);
        return;
      }
    } else if (TAB == 1) {
      //Judge the validity of characters
      if (ble_rx.data[i] >= ' ' && ble_rx.data[i] <= '~') {
        password[j++] = ble_rx.data[i];
      } else if (ble_rx.data[i] == 0) {
        password[j++] = 0;//Fill the end with 0
        break;
      } else {
        Serial.printf("cmd_wifi Err, data[%d] = 0x%x", i, ble_rx.data[i]);
        return;
      }
    }
  }

  StaticJsonDocument<300> doc;
  doc["model"] = "basic";
  doc["type"] = MESSAGE_TYPE.SYS_WIFI;
  doc["ssid"] = (const char *) ssid;
  doc["password"] = (const char *) password;
  doc["state"] = WIFI_STATE.CLIENT;
  rp.parseBasic(doc);
  extern char wifi_mode;
  if (wifi_mode == WIFI_STA) WiFi.disconnect();
}

void ble_cmd_send_device_info() {
  StaticJsonDocument<300> doc;
  doc["model"] = "basic";
  doc["type"] = MESSAGE_TYPE.GET_DEVICE_INFO;
  rp.parseBasic(doc);
}

void ble_cmd_restart() {
  StaticJsonDocument<300> doc;
  doc["model"] = "basic";
  doc["type"] = MESSAGE_TYPE.SYS_RESTART;
  rp.parseBasic(doc);
}


void ble_test(void) {
  return;
  while (1) {
    ble_loop();
  }
}

void ble_send_string(const String &message) {
  // Maximum payload size per packet (fixed 15 bytes)
  const uint8_t MAX_PAYLOAD_SIZE = 15;
  // Calculate total number of packets (round up)
  uint16_t total_length = message.length();
  uint8_t total_packets = (total_length + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;

  // Return immediately if string is empty
  if (total_packets == 0) return;

  for (uint8_t packet_idx = 0; packet_idx < total_packets; packet_idx++) {
    // Calculate current packet payload length
    uint16_t start_pos = packet_idx * MAX_PAYLOAD_SIZE;
    uint8_t payload_len = min(MAX_PAYLOAD_SIZE, (uint8_t)(total_length - start_pos));

    // Send string fragment directly (no packet header)
    ble_send_data((uint8_t * )(message.c_str() + start_pos), payload_len);

    // Add short delay to prevent transmission congestion (adjust according to actual hardware)
    delay(10);
  }
}