#pragma once

#include "driver/temp_sensor.h"
#include <ArduinoJson.h>
#include "esp_adc_cal.h"
#include "WiFiUtil.h"
#include "esp_mac.h"
#include "FUTABA_SBUS.h"
#include "FeedbackUtil.h"

// Message type constants
struct
{
    String SYS_WIFI = "sys_wifi";                  // WiFi system control message
    String SYS_WEB_SOCKET_SERVER = "sys_web_socket_server";  // WebSocket server control
    String SYS_RESTART = "sys_restart";            // System restart message
    String GET_DEVICE_INFO = "get_device_info";    // Device info request message
} MESSAGE_TYPE;

// Mode type constants
struct
{
    String BASIC = "basic";
} MODE_TYPE;

// Robot joystick axis index constants
struct
{
    int LEFT_ROCKER_X = 0;   // Left joystick X-axis (horizontal) index
    int LEFT_ROCKER_Y = 1;   // Left joystick Y-axis (vertical) index
    int RIGHT_ROCKER_X = 3;  // Right joystick X-axis (horizontal) index
    int RIGHT_ROCKER_Y = 2;  // Right joystick Y-axis (vertical) index

    int BALANCE_MODE = 4; // Balance mode toggle (0-off, 1-on)
    int SERVO_RESET = 5; // Servo calibration reset (0-off, 1-on)
    int BALL_HANDLER = 6; // Ball handling mechanism (0-off, 2-on)
    int POSTURE_MODE = 7; // Auto posture adjustment (0-off, 1-on)
} ROBOT_ROCKER_SUBSCRIPT;


// communication protocol attributes
struct
{
    String MODE = "mode";
    String TYPE = "type";

    String LINEAR = "linear";
    String ANGULAR = "angular";
    String STABLE = "stable";

    String ROLL = "roll";
    String HEIGHT = "height";
    String JOY_Y = "joy_y";
    String JOY_X = "joy_x";


    String BALANCE_MODE = "balance_mode";
    String SERVO_RESET = "servo_reset";
    String BALL_HANDLER = "ball_handler";
    String POSTURE_MODE = "posture_mode";


    String WIFI_SSID = "ssid";
    String WIFI_PASSWORD = "password";
    String WIFI_STATE = "state";

    String WEB_SOCKET_CLIENT_HOST = "web_socket_client_host";
    String WEB_SOCKET_CLIENT_PORT = "web_socket_client_port";
    String WEB_SOCKET_CLIENT_URL = "web_socket_client_url";
} COMMUNICATION_PROTOCOL_ATTRIBUTES;

struct
{
    int ROCKER_INPUT_MAXIMUM = 100;
    int ROCKER_INPUT_MINIMUM = -100;
    int ROCKER_OUTPUT_MAXIMUM = 1666;
    int ROCKER_OUTPUT_MINIMUM = 333;

    int RIGHT_ROCKER_INPUT_MAXIMUM = 100;
    int RIGHT_ROCKER_INPUT_MINIMUM = -100;
    int RIGHT_ROCKER_OUTPUT_MAXIMUM = 1666;
    int RIGHT_ROCKER_OUTPUT_MINIMUM = 333;

    int HEIGHT_INPUT_MAXIMUM = 100;
    int HEIGHT_INPUT_MINIMUM = -100;
    int HEIGHT_OUTPUT_MAXIMUM = 1666;
    int HEIGHT_OUTPUT_MINIMUM = 333;

    int SWITCH_INPUT_MAXIMUM = 1;
    int SWITCH_INPUT_MAXIMUM_PRO = 2;
    int SWITCH_INPUT_MINIMUM = 0;
    int SWITCH_OUTPUT_MAXIMUM = 1666;
    int SWITCH_OUTPUT_MINIMUM = 333;
} MOTION_DATA_RANGE;

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
    double battery_voltage;
    double pcb_version = 1.0;
    double fahrenheit;
    double centigrade;
    double battery_level;
    int status;

    RobotProtocol();   // Constructor
    ~RobotProtocol();  // Destructor

    double get_pcb_version();

    /**
     * @brief Initialize the robot protocol module
     * @param sBus Pointer to FUTABA_SBUS object for SBUS communication interface
     *
     * Must be called before using any protocol functions to set up SBUS communication.
     * The pointer will be stored for future communication handling.
     */
    void init(FUTABA_SBUS *sBus);

    /**
     * Initialization and reset of the control channel
     */
    void sBusInit();

    /**
     * @brief Single protocol processing cycle
     *
     * Should be called periodically in main loop to process pending messages
     */
    void spinOnce(void);

    double get_fahrenheit(void);

    double get_degree_centigrade(void);

    double get_battery_voltage(void);

    double get_battery_level(void);

    int get_robot_status(void);

    /**
     * @brief Main command parser entry point - Old
     * @param doc JSON document containing command
     *
     * Routes JSON content to appropriate handler functions
     */
    void parseBasic(StaticJsonDocument<500> &doc);


    /**
     * @brief Main command parser entry point - New
     * @param doc JSON document containing command
     *
     * Routes JSON content to appropriate handler functions
     */
    void parseJson(StaticJsonDocument<500> &doc);

private:
    /**
     * @brief Print command content (debug)
     * @param doc JSON document to print
     */
    void printDoc(StaticJsonDocument<500> &doc);

    /**
     * @brief Handle system control commands
     * @param doc JSON document containing command
     *
     * Processes system-level commands like WiFi config and restart
     */
    void isSys(StaticJsonDocument<500> &doc);

    /**
     * @brief Handle motion control commands
     * @param doc JSON document containing command
     *
     * Processes robot movement related commands
     */
    void isMotion(StaticJsonDocument<500> &doc);

    /**
     * @brief Linear interpolation mapping between input and output ranges
     *
     * @param input_value The input value to map
     * @param input_min Minimum value of input range
     * @param input_max Maximum value of input range
     * @param output_min Minimum value of output range
     * @param output_max Maximum value of output range
     * @return float The mapped value in output range
     *
     * @note Returns -1 if input range is invalid (input_min == input_max)
     * to maintain compatibility with AVR behavior
     */
    float mapf(long x, long in_min, long in_max, float out_min, float out_max);

    /**
     * @brief Convert joystick raw input to normalized float value
     * @param value Raw joystick input
     * @return Normalized float value in range [333, 1666]
     *
     * Note: The integer input is automatically converted to float during calculation.
     */
    float rockerConversion(int value);

    /**
     * @brief Convert raw height input to physical height value
     * @param value Raw height input
     * @return Actual height in meters as float
     *
     * Note: The integer input is automatically converted to float during calculation.
     */
    float heightConversion(int value);

    /**
     * @brief Convert boolean switch input to standardized float value
     * @param value Boolean input (true/false)
     * @return Float representation (0.0f for false, 1.0f for true)
     *
     * @note
     * - Provides type conversion from digital to analog representation
     * - Useful for consistent interface with systems expecting float values
     */
    float switchConversion(int value);

    /**
     * @brief Convert boolean switch input to standardized float value
     * @param value Boolean input (true/false)
     * @return Float representation (0.0f for false, 1.0f for true)
     *
     * @note
     * - Provides type conversion from digital to analog representation
     * - Useful for consistent interface with systems expecting float values
     */
    float switchPROConversion(int value);
};

// Global variables
extern char robot_model_base[];  // Robot model base information
extern RobotProtocol rp;         // Global RobotProtocol instance