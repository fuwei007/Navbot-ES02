#include "RobotUtil.h"

char robot_model_base[] = "navbot_es02-";
RobotProtocol rp;

RobotProtocol::RobotProtocol() {
  Serial.println("RobotProtocol:RobotProtocol");
}

RobotProtocol::~RobotProtocol() {
  Serial.println("RobotProtocol:~RobotProtocol");
}

void RobotProtocol::spinOnce(void) {
  Serial.println("RobotProtocol:spinOnce");
}

void RobotProtocol::printDoc(StaticJsonDocument<300> &doc) {
  char receive_command[300];
  serializeJson(doc, receive_command);
  Serial.printf("receive_command:%s \r\n", receive_command);
}

void RobotProtocol::isSys(StaticJsonDocument<300> &doc) {
  String type = doc["type"];
  if (type.length() == 0 || type == "null") {
    return;
  }
  Serial.print("type:");
  Serial.println(type);

  if (type == MESSAGE_TYPE.SYS_WIFI) {
    String ssid = doc["ssid"];
    String password = doc["password"];
    String state = doc["state"];

    // save data
    if (state == WIFI_STATE.SERVER || state == WIFI_STATE.CLIENT || state == WIFI_STATE.CLOSE) {
      storage_util.write(&StorageKey.WIFI_STATE, state);
    }
    storage_util.write(&StorageKey.WIFI_SSID, ssid);
    storage_util.write(&StorageKey.WIFI_PASSWORD, password);

    // read data
    Serial.print("READ SAVE WIFI STATE:");
    Serial.println(storage_util.read(&StorageKey.WIFI_STATE));
    Serial.print("READ SAVE WIFI SSID:");
    Serial.println(storage_util.read(&StorageKey.WIFI_SSID));
    Serial.print("READ SAVE WIFI PASSWORD:");
    Serial.println(storage_util.read(&StorageKey.WIFI_PASSWORD));

  } else if (type == MESSAGE_TYPE.SYS_WEB_SOCKET_SERVER) {
    String host = doc["host"];
    uint16_t port = doc["port"];
    String url = doc["url"];

    // save data
    storage_util.write(&StorageKey.WEB_SOCKET_HOST, host);
    storage_util.writeUint16T(&StorageKey.WEB_SOCKET_PORT, port);
    storage_util.write(&StorageKey.WEB_SOCKET_URL, url);

    // read data
    Serial.print("READ SAVE WEB SOCKET HOST:");
    Serial.println(storage_util.read(&StorageKey.WEB_SOCKET_HOST));
    Serial.print("READ SAVE WEB SOCKET PORT:");
    Serial.println(storage_util.readToUint16T(&StorageKey.WEB_SOCKET_PORT));
    Serial.print("READ SAVE WEB SOCKET URL:");
    Serial.println(storage_util.read(&StorageKey.WEB_SOCKET_URL));
  } else if (type == MESSAGE_TYPE.SYS_RESTART) {
    ESP.restart();
  } else if (type == MESSAGE_TYPE.GET_DEVICE_INFO) {

  }
}

void RobotProtocol::isMotion(StaticJsonDocument<300> &doc) {
  // Joystick control explanation
  // Left joystick up/down -> ascend/descend
  // Left joystick left/right -> lean left/right (lowers one side)
  // Right joystick up/down -> move forward/backward
  // Right joystick left/right -> rotate left/right
}

void RobotProtocol::parseBasic(StaticJsonDocument<300> &doc) {
  printDoc(doc);
  isSys(doc);
  isMotion(doc);
}
