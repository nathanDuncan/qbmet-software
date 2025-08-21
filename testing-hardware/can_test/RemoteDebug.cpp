#include "RemoteDebug.h"

RemoteDebug Debug; // Global instance

RemoteDebug::RemoteDebug() {
}

void RemoteDebug::begin(const char* hostname) {
  Serial.print("RemoteDebug started: ");
  Serial.println(hostname);
}

void RemoteDebug::println(const char* msg) {
  // Serial.println(msg);
}

void RemoteDebug::printf(const char* fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.println(buf);
}
