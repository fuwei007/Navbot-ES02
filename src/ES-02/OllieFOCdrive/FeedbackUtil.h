#pragma once

#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "BleUtil.h"
#include "RobotUtil.h"
#include "WebSocketClientUtil.h"


struct {
    int ALL = 0;
    int BLE = 1;
    int WEB_SOCKET_CLIENT = 2;
} FEEDBACK_CHANNEL;


void feedback_util_send_message(int send_channel);
