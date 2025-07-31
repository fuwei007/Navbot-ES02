#pragma once

#include "MyStorageUtil.h"
#include "WiFiUtil.h"
#include "RobotUtil.h"

/**
 * @brief Initializes the WebSocket client connection
 *
 * Sets up the WebSocket client with default configurations and
 * establishes connection parameters. Must be called once during setup.
 */
void web_sockets_client_init();

/**
 * @brief Maintains WebSocket client operations
 *
 * Should be called continuously in the main program loop.
 */
void web_sockets_client_loop();

void web_sockets_client_send_message(String value);