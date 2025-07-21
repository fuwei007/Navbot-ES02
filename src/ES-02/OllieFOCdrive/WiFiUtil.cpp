#include "WiFiUtil.h"

// Configure the parameters related to the AP (hotspot) mode
const char *ap_password = "12345678";

IPAddress AP_IP(192, 168, 1, 11);
IPAddress AP_GATEWAY(192, 168, 1, 11);
IPAddress AP_SUBNET(255, 255, 255, 0);


char wifi_mode;

// access the Wi-Fi network in AP mode
void wifi_set_sta() {
  String wifi_ssid = storage_util.read(&StorageKey.WIFI_SSID);
  String wifi_password = storage_util.read(&StorageKey.WIFI_PASSWORD);

  WiFi.setHostname("navbot-es02-123456");
  wifi_mode = WIFI_STA;
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);

  String wifi_info = String("ssid:") + wifi_ssid + ",password:" + wifi_password;
  Serial.println("Connecting to Wi-Fi: " + wifi_info);
}

static void dev_name_build(char dev_name[]) {

  uint8_t mac[7];
  esp_read_mac(mac, ESP_MAC_BT);
  mac[6] = 0;
  char i;

  for (i = 0; i < 6; i++) //Convert the mac address to contain only 0-9/a-z
  {
    mac[i] = mac[i] % 36; // 10+26=36

    if (mac[i] <= 9) mac[i] = mac[i] + '0'; //0-9
    else if (mac[i] <= 35) mac[i] = mac[i] - 10 + 'a'; //a-z
  }

  sprintf(dev_name, "%s%s", robot_model_base, mac);
  Serial.printf(dev_name);
  Serial.printf("\r\n");
}

void wifi_set_ap(void) {
  char ap_name[20] = {0};
  dev_name_build(ap_name);
  wifi_mode = WIFI_AP;
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(ap_name, ap_password);
}

String get_wifi_state(void) {
  String wifi_state = storage_util.read(&StorageKey.WIFI_STATE);
  if (wifi_state == "null" || wifi_state.length() == 0) {
    wifi_state = WIFI_STATE.SERVER;
  }
  return wifi_state;
}

void wifi_init(void) {
  String wifi_state = get_wifi_state();
  Serial.printf("wifi_state: %s\n", wifi_state.c_str());
  if (wifi_state == WIFI_STATE.CLIENT) {
    Serial.println("start wifi client...");
    wifi_set_sta(); // Wi-Fi client
  } else if (wifi_state == WIFI_STATE.SERVER) {
    Serial.println("start wifi server...");
    wifi_set_ap(); // Wi-Fi server
  }
}

void wifi_loop(void) {
  static char wifi_status = 0;
  static char connect_count_down = 10;

  if (wifi_mode != WIFI_STA) return;

  //If the Wi-Fi is not connected, it will actively connect every 10 seconds
  if (WiFi.status() != WL_CONNECTED) {
    connect_count_down--;
    if (connect_count_down == 0) {
      wifi_set_sta();
      connect_count_down = 10;
    }
    wifi_status = 0;

  } else //Print the IP information once when connecting via Wi-Fi
  {
    connect_count_down = 1;
    if (wifi_status == 0) {
      // Print the IP address
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    }
    wifi_status = 1;
  }

}

