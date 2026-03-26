
#include "robot.h"
#include <ArduinoJson.h>
#include <vector>
#include <esp_mac.h>
RobotProtocol rp;


RobotProtocol::RobotProtocol() {
}

RobotProtocol::~RobotProtocol() {
}


void RobotProtocol::build_dev_name(char dev_name[]) {
  char basc[] = "navbot_es02-";
  uint8_t mac[7];
  esp_efuse_mac_get_default(mac);
  mac[6] = 0;
  char i;

  for (i = 0; i < 6; i++)  //Convert the mac address to contain only 0-9/a-z
  {
    mac[i] = mac[i] % 36;  // 10+26=36

    if (mac[i] <= 9) mac[i] = mac[i] + '0';             //0-9
    else if (mac[i] <= 35) mac[i] = mac[i] - 10 + 'a';  //a-z
  }
  sprintf(dev_name, "%s%s", basc, mac);
}

void RobotProtocol::parseJson(StaticJsonDocument<300> &doc) {
  if (doc["type"].isNull() == true) {
    // Serial.println("JSON type is null ");
    // yield();
    // parseBasic(doc);
  } else {
    Serial.println("JSON type is sys");
    isSys(doc);
  }
}

void RobotProtocol::isSys(StaticJsonDocument<300> &doc) {
  String type = doc["type"];
  Serial.print("type:");
  Serial.println(type);
  // if (type == MESSAGE_TYPE.SYS_WIFI) {
  //   json_is_sys_wifi(doc);
  // } else if (type == MESSAGE_TYPE.SYS_WEB_SOCKET_SERVER) {
  //   json_is_sys_web_socket_server(doc);
  // } else if (type == MESSAGE_TYPE.SYS_RESTART) {
  //   ESP.restart();
  // } else if (type == MESSAGE_TYPE.GET_DEVICE_INFO) {
  //   feedback_util_send_message(FEEDBACK_CHANNEL.ALL);
  // } else if (type == MESSAGE_TYPE.OFF_SERVO) {
  //   sms_sts.off_all_servo();
  // } else if (type == MESSAGE_TYPE.ON_SERVO) {
  //   sms_sts.on_all_servo();
  // } else if (type == MESSAGE_TYPE.CALIBRATION_SERVO) {
  //   calibrate_servo();
  // } else if (type == MESSAGE_TYPE.SET_SERVO_ID1) {
  //   set_setvo_id1();
  // } else if (type == MESSAGE_TYPE.SET_SERVO_ID2) {
  //   set_setvo_id2();
  // } else if (type == MESSAGE_TYPE.SET_NAME) {
  //   json_is_sys_set_name(doc);
  // } else if (type == MESSAGE_TYPE.SHOW_EXPRESSION) {
  //   json_is_sys_show_expression(doc);
  // } else if (type == MESSAGE_TYPE.GET_EXPRESSION) {
  //   json_is_sys_send_expression(FEEDBACK_CHANNEL.ALL); 
  // } else if (type == MESSAGE_TYPE.SET_CLOUD_TOKEN) {
  //   json_is_sys_set_cloud_token(doc);
  // } else if (type == MESSAGE_TYPE.SET_OPENAI_TOKEN) {
  //   json_is_sys_set_openai_token(doc);
  // } else if (type == MESSAGE_TYPE.SET_TEST_NUMBER) {
  //   set_test_number(doc);
  // } else if (type == MESSAGE_TYPE.SET_UNCONTROLLABLE_ANGLE) {
  //   set_uncontrollable_angle(doc);
  // } else if (type == MESSAGE_TYPE.SET_RECOVERY_ANGLE) {
  //   set_recovery_angle(doc);
  // } else {
  //   Serial.println("Invalid json keyworeds.\r\n");
  // }
}



