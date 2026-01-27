#pragma once

#include <Arduino.h>

#define LOG_TAG(tag, message)            \
  do {                                   \
    Serial.print("# ");                 \
    Serial.print(tag);                   \
    Serial.print(" ");                  \
    Serial.println(message);             \
  } while (0)

#define LOG_TAG_VALUE(tag, label, value) \
  do {                                   \
    Serial.print("# ");                 \
    Serial.print(tag);                   \
    Serial.print(" ");                  \
    Serial.print(label);                 \
    Serial.println(value);               \
  } while (0)
