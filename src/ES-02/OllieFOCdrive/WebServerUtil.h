#pragma once

#include <WebServer.h>
#include "WiFiUtil.h"
#include "BasicWeb.h"

/**
 * @brief Initializes the web server utility
 *
 * Sets up all required web server routes and configurations.
 * Must be called once during system startup.
 */
void web_server_util_init(void);

/**
 * @brief Handles web server operations
 *
 * Processes incoming client requests and maintains server state.
 * Should be called regularly in the main program loop.
 */
void web_server_util_loop(void);

// Global WebServer instance for handling HTTP requests
// Provides access to web server functionality throughout the application
extern WebServer webserver;