#include "MyStorageUtil.h"
#include <Preferences.h>

StorageUtil storage_util;

Preferences db;

bool init_completed = false;

void StorageUtil::init(void) {
  if (!init_completed) {
    init_completed = true;
    db.begin("storage_util", false);
    Serial.print("StorageUtil Init Preferences");
  }
}

String StorageUtil::read(StorageKeyInfo *key) {
  if (!key) return String();
  init();
  return db.getString(key->name, String());;
}

size_t StorageUtil::read(StorageKeyInfo *key, char *value) {
  if (!key || !value) return 0;
  init();
  return db.getString(key->name, value, sizeof(value));
}

uint16_t StorageUtil::readToUint16T(StorageKeyInfo *key) {
  if (!key) return 0;
  init();
  return db.getUShort(key->name);
}

size_t StorageUtil::write(StorageKeyInfo *key, String value) {
  if (!key) return 0;
  init();
  return db.putString(key->name, value);
}

size_t StorageUtil::writeUint16T(StorageKeyInfo *key, uint16_t value) {
  if (!key) return 0;
  init();
  return db.putUShort(key->name, value);
}
