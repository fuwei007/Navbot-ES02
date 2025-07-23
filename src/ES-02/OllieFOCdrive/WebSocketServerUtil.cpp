#include "WebSocketServerUtil.h"
#include <WebSocketsServer.h>

WebSocketsServer websocket = WebSocketsServer(81); // Define a webSocket server to process messages sent by clients

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_TEXT) {
    String payload_str = String((char *) payload);
    StaticJsonDocument<500> doc;
    DeserializationError error = deserializeJson(doc, payload_str);

    String mode_str = doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.MODE];
    if (mode_str == MODE_TYPE.BASIC) {
      rp.parseBasic(doc);
    }
  }
}

void web_sockets_server_init() {
  if (get_wifi_state() != WIFI_STATE.CLOSE) {
    websocket.begin();
    websocket.onEvent(webSocketEventCallback);
  }
}

void web_sockets_server_loop() {
  websocket.loop();
}