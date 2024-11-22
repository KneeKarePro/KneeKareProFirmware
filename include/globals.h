#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <RTClib.h>
#include <SD.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// Global Instances
extern RTC_PCF8523 rtc;
extern SemaphoreHandle_t fileMutex;
extern QueueHandle_t potDataQueue;

// Constants
extern const int POT_PIN;
extern const char *DATA_FILE;
extern const int QUEUE_SIZE;

// Data Structures
struct KneeData {
  uint32_t time;
  uint16_t millis;
  uint16_t angle;
};

#endif // GLOBALS_H