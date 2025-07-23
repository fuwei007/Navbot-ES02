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
  sBusInit();
}

void RobotProtocol::sBusInit() {
  if (robot_util_s_bus->controlType != CONTROL_TYPE.SBUS) {
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.LEFT_ROCKER_X] = heightConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.LEFT_ROCKER_Y] = heightConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.RIGHT_ROCKER_X] = rockerConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.RIGHT_ROCKER_Y] = rockerConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.BALANCE_MODE] = switchPROConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.SERVO_RESET] = switchConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.BALL_HANDLER] = switchConversion(0);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.POSTURE_MODE] = switchConversion(0);
    Serial.println("RobotProtocol->sBusInit");
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
  String type = doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.TYPE];
  if (type.length() == 0 || type == "null") {
    return;
  }
  Serial.print("type:");
  Serial.println(type);

  if (type == MESSAGE_TYPE.SYS_WIFI) {
    String ssid = doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.WIFI_SSID];
    String password = doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.WIFI_PASSWORD];
    String state = doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.WIFI_STATE];

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
    String host = doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.WEB_SOCKET_CLIENT_HOST];
    uint16_t port = doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.WEB_SOCKET_CLIENT_PORT];
    String url = doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.WEB_SOCKET_CLIENT_URL];

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
              MOTION_DATA_RANGE.RIGHT_ROCKER_INPUT_MINIMUM,
              MOTION_DATA_RANGE.RIGHT_ROCKER_INPUT_MAXIMUM,
              MOTION_DATA_RANGE.RIGHT_ROCKER_OUTPUT_MINIMUM,
              MOTION_DATA_RANGE.RIGHT_ROCKER_OUTPUT_MAXIMUM
  );
}

float RobotProtocol::heightConversion(int value) {
  return mapf(value,
              MOTION_DATA_RANGE.HEIGHT_INPUT_MINIMUM,
              MOTION_DATA_RANGE.HEIGHT_INPUT_MAXIMUM,
              MOTION_DATA_RANGE.HEIGHT_OUTPUT_MINIMUM,
              MOTION_DATA_RANGE.HEIGHT_OUTPUT_MAXIMUM
  );
}

float RobotProtocol::switchConversion(int value) {
  return mapf(value,
              MOTION_DATA_RANGE.SWITCH_INPUT_MINIMUM,
              MOTION_DATA_RANGE.SWITCH_INPUT_MAXIMUM,
              MOTION_DATA_RANGE.SWITCH_OUTPUT_MINIMUM,
              MOTION_DATA_RANGE.SWITCH_OUTPUT_MAXIMUM
  );
}

float RobotProtocol::switchPROConversion(int value) {
  return mapf(value,
              MOTION_DATA_RANGE.SWITCH_INPUT_MINIMUM,
              MOTION_DATA_RANGE.SWITCH_INPUT_MAXIMUM_PRO,
              MOTION_DATA_RANGE.SWITCH_OUTPUT_MINIMUM,
              MOTION_DATA_RANGE.SWITCH_OUTPUT_MAXIMUM
  );
}

void RobotProtocol::isMotion(StaticJsonDocument<500> &doc) {

  int stable = doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.STABLE];

  if (stable != 1) {
    robot_util_s_bus->controlType = CONTROL_TYPE.SBUS;
    if (robot_util_s_bus->toChannels != 1) {
      // If the remote control is not connected, it will be reset.
      sBusInit();
    }
    return;
  } else {
    robot_util_s_bus->controlType = CONTROL_TYPE.OTHER;
  }

  if (robot_util_s_bus->controlType != CONTROL_TYPE.SBUS) {
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.LEFT_ROCKER_X] = rockerConversion(doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.ROLL]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.LEFT_ROCKER_Y] = heightConversion(doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.HEIGHT]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.RIGHT_ROCKER_X] = rockerConversion(doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.JOY_X]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.RIGHT_ROCKER_Y] = rockerConversion(doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.JOY_Y]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.BALANCE_MODE] = switchPROConversion(doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.BALANCE_MODE]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.SERVO_RESET] = switchConversion(doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.SERVO_RESET]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.BALL_HANDLER] = switchConversion(doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.BALL_HANDLER]);
    robot_util_s_bus->channels[ROBOT_ROCKER_SUBSCRIPT.POSTURE_MODE] = switchConversion(doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.POSTURE_MODE]);
  }
}

void RobotProtocol::parseBasic(StaticJsonDocument<500> &doc) {
  printDoc(doc);
  isSys(doc);
  isMotion(doc);
}
