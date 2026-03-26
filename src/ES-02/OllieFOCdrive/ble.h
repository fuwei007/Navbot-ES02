#pragma once

#define BLE_DATA_SIZE 1024
#include <ArduinoJson.h>


  //Please refer to the protocol specification for the mapping value.
#define BLE_CH0_MAX 100
#define BLE_CH0_MIN -100

#define BLE_CH1_MAX 100
#define BLE_CH1_MIN 0

#define BLE_CH2_MAX 100
#define BLE_CH2_MIN -100

#define BLE_CH3_MAX 100
#define BLE_CH3_MIN -100

#define BLE_CH4_MAX 2
#define BLE_CH4_MIN 0

#define BLE_CH5_MAX 1
#define BLE_CH5_MIN 0

#define BLE_CH6_MAX 1
#define BLE_CH6_MIN 0

#define BLE_CH7_MAX 2
#define BLE_CH7_MIN 0

#define BLE_CH8_MAX 5
#define BLE_CH8_MIN -5

#define BLE_CH9_MAX 5
#define BLE_CH9_MIN -5



typedef struct CmdManeuverTypDef {
  int8_t ch[10];
} CmdManeuverTypDef;

typedef struct CmdWifiTypDef {
  uint8_t header[2];
  uint8_t idle1[3];

} CmdWifiTypDef;


enum BLE_STATE {
  BLE_OFF_LINE = 0,
  BLE_STATE_IDLE = 1,
  BLE_STATE_RECEIVE_OK = 2,
  BLE_STATE_RECEIVE_WAIT = 3,
  BLE_STATE_WAITING_PROCEDD = 4,

  BLE_STATE_SEND_READY = 20,
  BLE_STATE_SEND_BEING = 21,
  BLE_STATE_SEND_FINISH = 22
};
enum BLE_CMD {
  CMD_RESTART = -1,
  CMD_MANEUVER = 0x10,
  CMD_WIFI = 1,
  CMD_JSON = 2,
  CMD_ACK = 0x7F
};

typedef struct BleDataTypDef {
  uint8_t frame[20];
  uint8_t cmd;                  //The processing command is consistent with the third byte in the received data
  uint8_t remaining_pack;       //The received data frames are still needed. After the reception is completed, data processing will be carried out
  uint8_t data[BLE_DATA_SIZE];  //Bluetooth cache data, including the merged ones, should be released after processing
  int16_t len;
  int16_t index;
  uint8_t state;
  uint8_t timeout;
} BleDataTypDef;


void ble_init();
//void ble_send_data(uint8_t* data, uint8_t len);
void ble_loop(void);
void ble_test(void);

void ble_send_string(const String &message);
void ble_tx_add_string(String str);
void ble_rx_add_data(char *data, int len);
void ble_tx_add_json(StaticJsonDocument<1024> &doc);
void ble_offline_processing(void);

extern BleDataTypDef ble_rx;
extern CmdManeuverTypDef ble_ctrler;









