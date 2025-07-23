#include "WebSocketClientUtil.h"
#include <WebSocketsClient.h>

WebSocketsClient webSocketClient = WebSocketsClient();

void eventCallback(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      printf("WEB SOCKET CLIENT:DISCONNECTED\n");
      break;

    case WStype_CONNECTED:
      printf("WEB SOCKET CLIENT:CONNECTED\n");
      break;
  }

  if (type == WStype_TEXT) {
    String payload_str = String((char *) payload);

    // Serial.print("Web Socket Client Receive A Message:");
    // Serial.println(payload_str);

    StaticJsonDocument<500> doc;
    DeserializationError error = deserializeJson(doc, payload_str);

    // printf("eFuse Two Point: Supported\n");
    String mode_str = doc[COMMUNICATION_PROTOCOL_ATTRIBUTES.MODE];
    if (mode_str == MODE_TYPE.BASIC) {
      rp.parseBasic(doc);
    }
  }
}

void web_sockets_client_init(void) {
  if (get_wifi_state() == WIFI_STATE.CLIENT) {
    String web_socket_client_host = storage_util.read(&StorageKey.WEB_SOCKET_HOST);
    uint16_t web_socket_client_port = storage_util.readToUint16T(&StorageKey.WEB_SOCKET_PORT);
    String web_socket_client_url = storage_util.read(&StorageKey.WEB_SOCKET_URL);

    if (web_socket_client_host.length() > 0 && web_socket_client_port > 0 && web_socket_client_url.length() > 0) {
      String connectionString =
              (web_socket_client_port == 443 ? "wss://" : "ws://") + web_socket_client_host + ":" +
              String(web_socket_client_port) + web_socket_client_url;

      Serial.println("WebSocket Connecting Address: " + connectionString);

      if (443 == web_socket_client_port) {
        webSocketClient.beginSSL(web_socket_client_host, web_socket_client_port, web_socket_client_url);
      } else {
        webSocketClient.begin(web_socket_client_host, web_socket_client_port, web_socket_client_url);
      }

      // Set event callback handler function
      webSocketClient.onEvent(eventCallback);

      // Optional: Set the reconnection interval (in milliseconds)
      webSocketClient.setReconnectInterval(10000);

      // Ping every 15 seconds, with a 3-second timeout and 2 failed attempts resulting in disconnection.
      webSocketClient.enableHeartbeat(15000, 3000, 2);
    } else {
      String param = "{web_socket_client_host:" + web_socket_client_host + ",web_socket_client_port:" +
                     web_socket_client_port + ",web_socket_client_url:" + web_socket_client_url + "}";
      Serial.println("Connecting to WebSocket: ERROR->Parameter verification failed->" + param);
    }
  }
}


void web_sockets_client_loop(void) {
  webSocketClient.loop();
}

