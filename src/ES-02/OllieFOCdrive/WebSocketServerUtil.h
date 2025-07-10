#pragma once


#include "MyStorageUtil.h"
#include "WiFiUtil.h"
#include "RobotUtil.h"

/**
 * @brief Initializes the WebSocket server
 *
 * Configures and starts the WebSocket server with default settings.
 * Call this once during application startup before any client connections.
 */
void web_sockets_server_init();

/**
 * @brief Processes WebSocket server events
 *
 * Maintains active WebSocket sessions. Call this repeatedly in the main loop.
 */
void web_sockets_server_loop();