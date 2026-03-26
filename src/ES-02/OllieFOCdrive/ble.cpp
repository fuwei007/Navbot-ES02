#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include "ble.h"
#include <cppQueue.h>
#include "robot.h"
#include "string.h"

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic;
bool oldDeviceConnected = false;

BleDataTypDef ble_rx;
CmdManeuverTypDef ble_ctrler;

cppQueue	ble_tx_q(sizeof(BleDataTypDef*), 10, FIFO);	//Create a Bluetooth transmission queue, with first-in-first-out principle.

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "6E400011-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400012-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400013-B5A3-F393-E0A9-E50E24DCCA9E"

void ble_tx_processing(void);
void ble_rx_processing(void);
void ble_rx_data_add(uint8_t* data, uint8_t len);
void ble_rx_data_clear(void);
void ble_cmd_maneuver_processing(void);
void ble_cmd_wifi_processing(void);
void ble_cmd_json_processing(void);
bool ble_frames_validation(void);


class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    uint8_t* rxData = pCharacteristic->getData();
    LED2_TOGGLE();
    if (pCharacteristic->getLength() == 20) {
      int i;
      for (i = 0; i < 20; i++) {
        ble_rx.frame[i] = *(rxData + i);  //Transfer data
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

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    rp.ble_connected = true;
    Serial.println("ble connected");
  };

  void onDisconnect(BLEServer* pServer) {
    rp.ble_connected = false;
    // Restart the broadcast to allow for reconnection
    pServer->getAdvertising()->start();

    memset(&ble_ctrler, 0, sizeof(CmdManeuverTypDef));
    Serial.println("ble disconnected");
  }
};

void ble_init() {
  char ble_name[20] = { 0 };
  rp.build_dev_name(ble_name);
  // Create the BLE Device
  BLEDevice::init(ble_name);
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  ble_rx_data_clear();
}
void ble_send_data(uint8_t* data, uint8_t len) {
  if (len > 20) {
    len = 20;
  }
  LED2_TOGGLE();
  pTxCharacteristic->setValue(data, len);  //reply
  pTxCharacteristic->notify();
}


void ble_loop(void) {
  ble_rx_processing();
  ble_tx_processing();
}
void ble_rx_processing(void) {
  //The BLE data reception has been completed
  if (ble_rx.state == BLE_STATE_RECEIVE_OK) {

    if (ble_frames_validation() == false) {
      ble_rx_data_clear();
      Serial.println("ble deta err");
      ble_rx.state = BLE_STATE_IDLE;
      return;
    }
    //BLE reply
    ble_send_data((uint8_t*)ble_rx.frame, 20);
    //Merge data packets
    if (ble_rx.remaining_pack > 0) {
      ble_rx_data_add(&ble_rx.frame[5], 15);
      ble_rx.remaining_pack--;
      //There is still BLE data to be received and status changes
      ble_rx.state = BLE_STATE_RECEIVE_WAIT;
    } else  //Process data packets
    {
      //Merge the last package of data；if there is only one package of data, that is also the last one
      ble_rx_data_add(&ble_rx.frame[5], 15);
      switch (ble_rx.cmd) {
        case CMD_MANEUVER:
          {
            ble_cmd_maneuver_processing();
          }
          break;
        case CMD_WIFI:
          {
            //ble_cmd_wifi_processing();
          }
          break;
        case CMD_JSON:
          {
            ble_cmd_json_processing();
          }
          break;
      }
      //The data processing is completed and the status is changed to idle
      ble_rx.state = BLE_STATE_IDLE;
      ble_rx_data_clear();
    }
  }
}
void ble_tx_processing(void) {

  /*
  If the queue is empty, no sending will be carried out.
  */
  if(ble_tx_q.getCount()==0){
    return;
  }
  /*
  If the queue is not empty but the Bluetooth connection is not established, then clear the queue.
  */
  if(rp.ble_connected == false){
    BleDataTypDef* _ble_tx;
    ble_tx_q.peek(&_ble_tx);
    ble_tx_q.drop();
    free(_ble_tx);
    Serial.print("BLE not connected, deleing queue; Queue remaining: ");Serial.println(ble_tx_q.getCount());
    return;
  }
  

  /*
  Read the queue data. After sending is completed, manual queue deletion and memory release are required.
  */
  BleDataTypDef* ble_tx;
  ble_tx_q.peek(&ble_tx);
  /*
  `ble_tx.index` is the index of the current data being sent.
  If it is greater than or equal to the total length,
  it indicates that the sending is complete or there is no data to be sent.
  */ 
  if (ble_tx->index >= ble_tx->len) {
    ble_tx->state = BLE_STATE_SEND_FINISH;
    Serial.printf("finish!!!  ble_tx.index >= ble_tx.len  , ble_tx.index : %d , ble_tx.len : %d \r\n", ble_tx->index, ble_tx->len);
    ble_tx_q.drop();
    free(ble_tx);
    Serial.print("Queue remaining: ");Serial.println(ble_tx_q.getCount());
    return;
  }
  /*
  Control the interval time of Bluetooth data frames
  */
  static int8_t time_tick = 100;
  if (time_tick > 0) {
    time_tick -= 10;
    return;
  } else {
    time_tick = 100;
  }

  Serial.print("ble send frame -> ");
  ble_tx->state = BLE_STATE_SEND_BEING;

  ble_tx->frame[0] = 0x55;
  ble_tx->frame[1] = 0xAA;
  ble_tx->frame[2] = 0x02;  //json
  ble_tx->frame[3] = (ble_tx->len - ble_tx->index) / 15 - ((ble_tx->len - ble_tx->index)%15 ? 0:1);
  ble_tx->frame[4] = 0;  //(ble_tx.len) / 15 ;

  memcpy(&ble_tx->frame[5], &ble_tx->data[ble_tx->index], 15);

  
  ble_send_data((uint8_t*)ble_tx->frame, 20);

  uint8_t i;
  for (i = 0; i < 20; i++) {
    Serial.print(ble_tx->frame[i],HEX);
    Serial.print(" ");
  }
  Serial.println("----");

  ble_tx->index += 15;
}
void ble_offline_processing(){
  //Clear the Bluetooth data and status after the Bluetooth connection is disconnected to prevent accidental operations.
  if(rp.ble_connected == false){
  }
}
void ble_send_string(const String& message) {
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
    ble_send_data((uint8_t*)(message.c_str() + start_pos), payload_len);

    // Add short delay to prevent transmission congestion (adjust according to actual hardware)
    delay(10);
  }
}
bool ble_frames_validation(void) {
  static uint8_t last_remaining_pack;
  static uint8_t last_cmd;
  //Fixed header
  if (ble_rx.frame[0] != 0x55) return false;
  if (ble_rx.frame[1] != 0xAA) return false;
  //If there is a subsequent frame in the previous data, the command must be the same.
  if ((ble_rx.frame[2] != last_cmd) && (last_remaining_pack > 0)) return false;
  if (ble_rx.frame[3] != ble_rx.remaining_pack) return false;

  last_cmd = ble_rx.frame[2];
  last_remaining_pack = ble_rx.remaining_pack;
  return true;
}

void ble_rx_data_add(uint8_t* data, uint8_t len) {
  int i;
  for (i = 0; i < len; i++) {
    ble_rx.data[ble_rx.index] = *(data + i);
    ble_rx.index++;
    if (ble_rx.index > (BLE_DATA_SIZE - 1)) {
      ble_rx.index = BLE_DATA_SIZE - 1;
    }
  }
  ble_rx.data[ble_rx.index] = 0;
}

void ble_rx_data_clear() {
  // int i;
  // for (i = 0; i < BLE_DATA_SIZE; i++) {
  //   ble_rx.data[i] = 0;
  // }
  ble_rx.index = 0;
  ble_rx.remaining_pack = 0;
}

//
void ble_cmd_maneuver_processing(void) {
  memcpy(ble_ctrler.ch, ble_rx.data, 10);
  ble_rx.state = BLE_STATE_IDLE;
}
void ble_cmd_json_processing(void) {
  String payload_str = String((char*)ble_rx.data);
  StaticJsonDocument<300> doc;
  deserializeJson(doc, payload_str);
  rp.parseJson(doc);
}
// void ble_cmd_restart_processing() {
//   StaticJsonDocument<300> doc;
//   doc["type"] = MESSAGE_TYPE.SYS_RESTART;
//   rp.parseJson(doc);
// }
void ble_tx_add_data(char* data, int len) {

  if(ble_tx_q.isFull() ==true){
    Serial.println("Queue is full, failed to add data");
    return ;
  }

  if(len>BLE_DATA_SIZE-1){
    len = BLE_DATA_SIZE-1;
  }

  BleDataTypDef* ble_tx;
  ble_tx = (BleDataTypDef*)malloc(sizeof(BleDataTypDef));
  if(ble_tx == NULL)
  {
    Serial.println("error!!!!malloc false,ble txdata");
    return;
  }
  memcpy(ble_tx->data, data, len);
  /*
  The remaining bytes are filled with zeros; 
  at most, 15 bytes are filled with zeros because sending one frame of data is 15 bytes long.
  */
 uint8_t i;
  for(i=0;i+len<BLE_DATA_SIZE;i++)
  {
    *(ble_tx->data+i+len ) = 0;
    if(i>15) break;
  }

  ble_tx->len = len;
  ble_tx->index = 0;
  ble_tx->state = BLE_STATE_SEND_READY;

  ble_tx_q.push(&ble_tx);
  Serial.println("ble_tx_add_data:");
  Serial.println((char*)ble_tx->data);
  Serial.print("Queue remaining:");Serial.println(ble_tx_q.getCount());

}
void ble_tx_add_string(String str) {
  char buffer[1024] = { 0 };
  int len = 1 + str.length();
  str.toCharArray(buffer, len);
  ble_tx_add_data(buffer, len);
}
void ble_tx_add_json(StaticJsonDocument<1024> &doc)
{
  String jsonStr;
  serializeJson(doc, jsonStr);
  ble_tx_add_string(jsonStr);
}



/*****************************************************      test    ***************************************************************/
bool one_second_tick(void);
bool ten_msec_tick(void);
void test_rx_json(char* data);



void ble_test(void) {
  return;
  Serial.begin(115200);
  ble_init();
  while (1) {
    if (ten_msec_tick()) {
      ble_loop();
    }
    if (one_second_tick()) {

      test_rx_json("{\"type\":\"get_device_info\"}");
    }
  }
}
void test_rx_json(char* data) {
  String payload_str = String(data);
  StaticJsonDocument<300> doc;
  deserializeJson(doc, payload_str);
  rp.parseJson(doc);
}
