#pragma once

#include <ArduinoJson.h>

// Storage key definition structure
typedef struct StorageKeyInfo
{
    char name[30];  // Key name stored in flash
} StorageKeyInfo;

// Collection of all storage keys used in the system
struct
{
    // Wi-Fi configuration keys
    StorageKeyInfo WIFI_SSID = {"wifi_ssid"};
    StorageKeyInfo WIFI_PASSWORD = {"wifi_password"};
    StorageKeyInfo WIFI_STATE = {"wifi_state"};

    // WebSocket configuration keys
    StorageKeyInfo WEB_SOCKET_HOST = {"web_socket_host"};
    StorageKeyInfo WEB_SOCKET_PORT = {"web_socket_port"};
    StorageKeyInfo WEB_SOCKET_URL = {"web_socket_url"};
} StorageKey;

/**
 * @class StorageUtil
 * @brief Non-volatile storage access utility
 *
 * Provides read/write operations for persistent storage (e.g., SPIFFS, EEPROM, or Preferences)
 */
class StorageUtil
{
public:
    /**
     * @brief Read string value from storage
     * @param key Pointer to StorageKeyInfo containing the key name
     * @return String containing the stored value
     */
    String read(StorageKeyInfo *key);

    /**
     * @brief Read value into character buffer
     * @param key Pointer to StorageKeyInfo containing the key name
     * @param value Output buffer for the stored value
     * @return size_t Number of bytes read
     */
    size_t read(StorageKeyInfo *key, char *value);

    /**
     * @brief Read unsigned 16-bit integer value
     * @param key Pointer to StorageKeyInfo containing the key name
     * @return uint16_t Retrieved value
     */
    uint16_t readToUint16T(StorageKeyInfo *key);

    /**
     * @brief Write string value to storage
     * @param key Pointer to StorageKeyInfo containing the key name
     * @param value String value to store
     * @return size_t Number of bytes written
     */
    size_t write(StorageKeyInfo *key, String value);

    /**
     * @brief Write unsigned 16-bit integer value
     * @param key Pointer to StorageKeyInfo containing the key name
     * @param value Integer value to store
     * @return size_t Number of bytes written
     */
    size_t writeUint16T(StorageKeyInfo *key, uint16_t value);

private:
    /**
     * @brief Initialize storage subsystem
     */
    void init(void);
};

// Global storage utility instance
extern StorageUtil storage_util;
