#pragma once

#include <WiFi.h>
#include <ArduinoJson.h>
#include "MyStorageUtil.h"
#include "RobotUtil.h"

/**
 * @brief Initializes WiFi connectivity
 *
 * Configures WiFi hardware and establishes connection parameters.
 * Must be called once during system initialization.
 */
void wifi_init(void);

/**
 * @brief Maintains WiFi connection state
 *
 * Call this function repeatedly in the main application loop.
 */
void wifi_loop(void);

/**
 * @brief Gets current WiFi connection state
 * @return String representing the current state ("server", "client", or "close")
 */
String get_wifi_state(void);

/**
 * @brief WiFi operation mode states
 *
 * Defines possible WiFi operation modes:
 * - SERVER: Device acts as WiFi access point
 * - CLIENT: Device connects as WiFi station
 * - CLOSE: WiFi interface is disabled
 */
struct
{
    String SERVER = "server";
    String CLIENT = "client";
    String CLOSE = "close";
} WIFI_STATE;

