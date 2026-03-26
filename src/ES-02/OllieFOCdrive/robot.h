#pragma once

#include <ArduinoJson.h>


#define LED2_BUILTIN 35
#define LED2_TOGGLE() digitalWrite(LED2_BUILTIN, !digitalRead(LED2_BUILTIN));


class RobotProtocol {
public:
  double battery_voltage = 8.4f;
  double pcb_version;
  double fahrenheit;
  double centigrade;
  double battery_level;
  int status;
  int16_t offset_roll = 0;
  int uncontrollable = 0;   //Too much tilt and loss of control
  float uncontrollable_angle = 35.0f;
  float recovery_angle      = 15.0f;
  float m1_direction;
  float m2_direction;

  bool charge = false;
  bool socket_connected = false;
  bool ble_connected = false;
  bool wifi_connected = false;
  // char wifi_state      = WIFI_CLOSE;

  bool send_status_flag = false;

  String show_expression;
  int show_expression_time = -1;  //

  uint8_t test_number=0;

  // StaticJsonDocument<WIFI_INFO_JSON_SIZE> wifi_info_json;
  // StaticJsonDocument<CONFIG_JSON_SIZE> config_json;
  // StaticJsonDocument<512> status_json;

  RobotProtocol();
  ~RobotProtocol();
  double get_battery_voltage(void);
  double get_battery_level(void);
  void get_dev_name(char dev_name[]);
  void build_dev_name(char dev_name[]);
  void config_json_init(void);
  void printDoc(StaticJsonDocument<300> &doc);
  void isSys(StaticJsonDocument<300> &doc);
  void parseJson(StaticJsonDocument<300> &doc);
  void send_status(void);
  void send_heartbeat(void);
  void json_test(char *json_arr);
  void set_uncontrollable_angle(StaticJsonDocument<300> &doc);
  void set_recovery_angle(StaticJsonDocument<300> &doc);
  void set_test_number(StaticJsonDocument<300> &doc);
  void test_log_output(void);
private:
  uint8_t *_now_buf;
  uint8_t *_old_buf;
  uint8_t _len;
  void UART_WriteBuf(void);
  void json_is_sys_set_name(StaticJsonDocument<300> &doc);
};



extern RobotProtocol rp;


