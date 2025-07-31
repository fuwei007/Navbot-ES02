#pragma once

#define BLE_DATA_SIZE 80


typedef struct CmdManeuverTypDef
{
    int8_t CH1_ROLL;       // Left/right tilt (-100~100)
    uint8_t CH2_HEIGHT;    // Height (0~100)
    int8_t CH3_PITCHING;   // Forward/backward movement (-100~100)
    int8_t CH4_YAW;        // Rotation (-100~100)
    uint8_t SWA_EN;        // On/off switch (0:stop, 1:start, 2:start+touch)
    uint8_t SWB_POSTURE;   // Mode switch (0:posture, 1:marker)
    uint8_t SWC_ROLL_MODE; // Tilt mode (0:manual, 1:auto)
    uint8_t SWD_POSTURE_OPTION; // Posture option (0:default, 1:pitch, 2:ball balance)
    int8_t VRA_BALL_X;     // Ball X (-5~5)
    int8_t VRB_BALL_Y;     // Ball Y (-5~5)
    uint8_t NULL2[5];      // Reserved bits
} CmdManeuverTypDef;

typedef struct CmdWifiTypDef
{
    uint8_t header[2];
    uint8_t idle1[3];

} CmdWifiTypDef;


enum BLE_STATE
{
    BLE_OFF_LINE = 0,
    BLE_STATE_IDLE = 1,
    BLE_STATE_RECEIVE_OK = 2,
    BLE_STATE_RECEIVE_WAIT = 3,
    BLE_STATE_WAITING_PROCEDD = 4,

    BLE_STATE_SEND_READY = 20,
    BLE_STATE_SEND_BEING = 21,
    BLE_STATE_SEND_FINISH = 22
};
enum BLE_CMD
{
    CMD_MANEUVER = 0x10,
    CMD_WIFI = 1,
    CMD_JSON = 2,
    CMD_RESTART = 9,
};

typedef struct BleDataTypDef
{
    uint8_t frame[20];
    uint8_t cmd;              //The processing command is consistent with the third byte in the received data
    uint8_t remaining_pack;   //The received data frames are still needed. After the reception is completed, data processing will be carried out
    uint8_t data[BLE_DATA_SIZE];         //Bluetooth cache data, including the merged ones, should be released after processing
    int16_t len;
    int16_t index;
    uint8_t state;
    uint8_t timeout;
} BleDataTypDef;


void ble_init();

//void ble_send_data(uint8_t* data, uint8_t len);
void ble_loop(void);

void ble_test(void);

void ble_rx_add_string(String str);

void ble_rx_add_data(char *data, int len);

extern BleDataTypDef ble_rx, ble_tx;







