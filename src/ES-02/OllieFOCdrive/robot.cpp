
#include "robot.h"
#include <ArduinoJson.h>
#include <vector>
#include <esp_mac.h>
#include <arduino.h>
#include "filter.h"
#include "ble.h"

const Message_TypeDef MESSAGE_TYPE;


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
    Serial.println("JSON type is null ");
  } else {
    Serial.println("JSON type is sys");
    isSys(doc);
  }
}

void RobotProtocol::isSys(StaticJsonDocument<300> &doc) {
  String type = doc["type"];
  Serial.print("type:");
  Serial.println(type);
 if (type == MESSAGE_TYPE.GET_DEVICE_INFO) {
    send_device_info();
  } else {
    Serial.println("Invalid json keyworeds.\r\n");
  }
}


// Send all information
void RobotProtocol::send_device_info() {
  // Get device information and send
  String device_info = get_device_info();

  // Send data according to the specified channel
  Serial.print("device info:");

  Serial.println(device_info);

  ble_tx_add_string(device_info);
}

String RobotProtocol::get_device_info() {
  // Serialize to string
  String jsonStr;
  // Create static JSON document (allocate memory pool size)
  StaticJsonDocument<1024> doc;

  // Read voltage
  double battery_voltage = rp.get_battery_voltage();
  // Calculate battery level
  double battery_percentage = rp.get_battery_level();
  
  // Format to retain two decimal places (rounded to 2 decimal places)
  battery_voltage = round(battery_voltage * 100) / 100;
  battery_percentage = round(battery_percentage) / 1;

  // Add simple values
  doc["type"] = "get_device_info";
  doc["battery_level"] = rp.battery_level;
  doc["battery_voltage"] = rp.battery_voltage;
  serializeJson(doc, jsonStr);
  return jsonStr;
}
double RobotProtocol::get_battery_voltage() {
  // static biquadFilter_t VoltageFilterLPF;
  uint16_t VoltageADC = analogRead(17);
  // float VoltageADCf = biquadFilterApply(&VoltageFilterLPF, VoltageADC);
  battery_voltage = (float)7.77 / 813.43 * VoltageADC;
  return battery_voltage;
}

double RobotProtocol::get_battery_level() {
  double currentVoltage = battery_voltage;
  double maxVoltage = 8.3;
  double minVoltage = 7.0;

  // Ensure the voltage is within a reasonable range

  if (currentVoltage >= maxVoltage) return 100.0;
  if (currentVoltage <= minVoltage) return 0.0;

  // Calculate the percentage
  double battery_percentage = (currentVoltage - minVoltage) / (maxVoltage - minVoltage) * 100.0;

  battery_percentage = round(battery_percentage) / 1;  // Format to retain two significant digits (rounded to 2 decimal places)

  // Battery logic correction (prevent out-of-bounds and display of 0 data)
  if (battery_percentage > 100) {
    battery_percentage = 100;
  } else if (battery_percentage < 1) {
    battery_percentage = 1;
  }
  battery_level = battery_percentage;
  return battery_level;
}
void RobotProtocol::json_test(char *json_arr) {
  Serial.print("json test: ");
  Serial.println(json_arr);
  String payload_str = String(json_arr);
  StaticJsonDocument<300> doc;
  DeserializationError error = deserializeJson(doc, payload_str);
  if (error) {
    Serial.println("json data error");
  } else {
    parseJson(doc);
  }
}





