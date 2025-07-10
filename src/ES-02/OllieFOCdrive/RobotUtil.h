#pragma once

#include <ArduinoJson.h>
#include "esp_adc_cal.h"
#include "WiFiUtil.h"
#include "esp_mac.h"

// Message type constants
struct
{
    String SYS_WIFI = "sys_wifi";                  // WiFi system control message
    String SYS_WEB_SOCKET_SERVER = "sys_web_socket_server";  // WebSocket server control
    String SYS_RESTART = "sys_restart";            // System restart message
    String GET_DEVICE_INFO = "get_device_info";    // Device info request message
} MESSAGE_TYPE;

/**
 * @class RobotProtocol
 * @brief Robot communication protocol handler
 *
 *
 * Parses and processes robot control protocol including system commands and motion control
 */
class RobotProtocol
{
public:
    RobotProtocol();   // Constructor
    ~RobotProtocol();  // Destructor

    /**
     * @brief Single protocol processing cycle
     *
     * Should be called periodically in main loop to process pending messages
     */
    void spinOnce(void);

    /**
     * @brief Main command parser entry point
     * @param doc JSON document containing command
     *
     * Routes JSON content to appropriate handler functions
     */
    void parseBasic(StaticJsonDocument<300> &doc);

private:
    /**
     * @brief Print command content (debug)
     * @param doc JSON document to print
     */
    void printDoc(StaticJsonDocument<300> &doc);

    /**
     * @brief Handle system control commands
     * @param doc JSON document containing command
     *
     * Processes system-level commands like WiFi config and restart
     */
    void isSys(StaticJsonDocument<300> &doc);

    /**
     * @brief Handle motion control commands
     * @param doc JSON document containing command
     *
     * Processes robot movement related commands
     */
    void isMotion(StaticJsonDocument<300> &doc);
};

// Global variables
extern char robot_model_base[];  // Robot model base information
extern RobotProtocol rp;         // Global RobotProtocol instance