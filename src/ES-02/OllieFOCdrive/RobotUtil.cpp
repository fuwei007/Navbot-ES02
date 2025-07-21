#include "RobotUtil.h"

char robot_model_base[] = "navbot_es02-";
RobotProtocol rp;

FUTABA_SBUS *robot_util_s_bus;

RobotProtocol::RobotProtocol() {
  Serial.println("RobotProtocol:RobotProtocol");
}

RobotProtocol::~RobotProtocol() {
  Serial.println("RobotProtocol:~RobotProtocol");
}

void RobotProtocol::spinOnce(void) {
  Serial.println("RobotProtocol:spinOnce");
}

void RobotProtocol::init(FUTABA_SBUS *sBus) {
  robot_util_s_bus = sBus;
  if (robot_util_s_bus->controlType != CONTROL_TYPE.SBUS) {
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.LEFT_ROCKER_X] = heightConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.LEFT_ROCKER_Y] = heightConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.RIGHT_ROCKER_X] = rockerConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.RIGHT_ROCKER_Y] = rockerConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.BALANCE_MODE] = switchPROConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.SERVO_RESET] = switchConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.BALL_HANDLER] = switchConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.POSTURE_MODE] = switchConversion(0);
  }
}

float RobotProtocol::mapf(long x, long in_min, long in_max, float out_min, float out_max) {
  long divisor = (in_max - in_min);
  if (divisor == 0) {
    return -1; //AVR returns -1, SAM returns 0
  }
  return (x - in_min) * (out_max - out_min) / divisor + out_min;
}

void RobotProtocol::printDoc(StaticJsonDocument<500> &doc) {
  char receive_command[500];
  serializeJson(doc, receive_command);
  Serial.printf("receive_command:%s \r\n", receive_command);
}

void RobotProtocol::isSys(StaticJsonDocument<500> &doc) {
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

float RobotProtocol::rockerConversion(int value) {
  return mapf(value,
              MOTION_DATA_RANGE.ROCKER_INPUT_MAXIMUM,
              MOTION_DATA_RANGE.ROCKER_INPUT_MINIMUM,
              MOTION_DATA_RANGE.ROCKER_OUTPUT_MAXIMUM,
              MOTION_DATA_RANGE.ROCKER_OUTPUT_MINIMUM
  );
}

float RobotProtocol::heightConversion(int value) {
  return mapf(value,
              MOTION_DATA_RANGE.HEIGHT_INPUT_MAXIMUM,
              MOTION_DATA_RANGE.HEIGHT_INPUT_MINIMUM,
              MOTION_DATA_RANGE.HEIGHT_OUTPUT_MAXIMUM,
              MOTION_DATA_RANGE.HEIGHT_OUTPUT_MINIMUM
  );
}

float RobotProtocol::switchConversion(int value) {
  return mapf(value,
              MOTION_DATA_RANGE.SWITCH_INPUT_MAXIMUM,
              MOTION_DATA_RANGE.SWITCH_INPUT_MINIMUM,
              MOTION_DATA_RANGE.SWITCH_OUTPUT_MAXIMUM,
              MOTION_DATA_RANGE.SWITCH_OUTPUT_MINIMUM
  );
}

float RobotProtocol::switchPROConversion(int value) {
  return mapf(value,
              MOTION_DATA_RANGE.SWITCH_INPUT_MAXIMUM_PRO,
              MOTION_DATA_RANGE.SWITCH_INPUT_MINIMUM,
              MOTION_DATA_RANGE.SWITCH_OUTPUT_MAXIMUM,
              MOTION_DATA_RANGE.SWITCH_OUTPUT_MINIMUM
  );
}

void RobotProtocol::isMotion(StaticJsonDocument<500> &doc) {

  int stable = doc[MOTION_ATTRIBUTE.STABLE];

  if (stable != 1) {
    return;
  }

  robot_util_s_bus->controlType = CONTROL_TYPE.OTHER;

  if (robot_util_s_bus->controlType != CONTROL_TYPE.SBUS) {
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.LEFT_ROCKER_X] = heightConversion(doc[MOTION_ATTRIBUTE.HEIGHT]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.LEFT_ROCKER_Y] = heightConversion(doc[MOTION_ATTRIBUTE.ROLL]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.RIGHT_ROCKER_X] = rockerConversion(doc[MOTION_ATTRIBUTE.JOY_Y]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.RIGHT_ROCKER_Y] = rockerConversion(doc[MOTION_ATTRIBUTE.JOY_X]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.BALANCE_MODE] = switchPROConversion(doc[MOTION_ATTRIBUTE.BALANCE_MODE]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.SERVO_RESET] = switchConversion(doc[MOTION_ATTRIBUTE.SERVO_RESET]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.BALL_HANDLER] = switchConversion(doc[MOTION_ATTRIBUTE.BALL_HANDLER]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.POSTURE_MODE] = switchConversion(doc[MOTION_ATTRIBUTE.POSTURE_MODE]);
  }
}

void RobotProtocol::parseBasic(StaticJsonDocument<500> &doc) {
  printDoc(doc);
  isSys(doc);
  isMotion(doc);
}
