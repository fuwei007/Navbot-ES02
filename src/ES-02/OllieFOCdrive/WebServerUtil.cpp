#include "WebServerUtil.h"


//WebServer instance
WebServer webserver; // server


void basic_web_callback(void) {
  webserver.send(300, "text/html", BasicWeb);
}

void web_server_util_init(void) {
  if (get_wifi_state() != WIFI_STATE.CLOSE) {
    webserver.begin();
    webserver.on("/", HTTP_GET, basic_web_callback);
  }
}

void web_server_util_loop(void) {
  webserver.handleClient();
}
