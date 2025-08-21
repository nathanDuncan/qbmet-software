#ifndef REMOTE_DEBUG_H
#define REMOTE_DEBUG_H

#include <Arduino.h>
#include <stdarg.h>

// A minimal RemoteDebug class implementation.
class RemoteDebug {
public:
  RemoteDebug();

  // Call this to initialize the remote debug interface (for example, to print a startup message)
  void begin(const char* hostname);

  // Print a line (with newline appended)
  void println(const char* msg);

  // Print formatted output (like printf)
  void printf(const char* fmt, ...);
};

// Create a global instance so that the macros can use it.
extern RemoteDebug Debug;

// Define debug macros for informational and warning messages.
#define debugI(fmt, ...) Debug.printf((fmt), ##__VA_ARGS__)
#define debugW(fmt, ...) Debug.printf((fmt), ##__VA_ARGS__)

#endif // REMOTE_DEBUG_H
