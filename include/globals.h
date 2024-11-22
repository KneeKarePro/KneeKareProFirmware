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

/**
 * @brief The RTC module used for timekeeping.
 * 
 */
extern RTC_PCF8523 rtc;

/**
 * @brief Mutex for managing access to the SD card file.
 * 
 */
extern SemaphoreHandle_t fileMutex;

/**
 * @brief Queue for storing potentiometer data.
 * 
 */
extern QueueHandle_t potDataQueue;

/**
 * @brief Pin connected to the potentiometer.
 * 
 */
extern const int POT_PIN;

/**
 * @brief The name of the data file on the SD card.
 * 
 */
extern const char *DATA_FILE;

/**
 * @brief The size of the queue for storing potentiometer data.
 * 
 */
extern const int QUEUE_SIZE;

/**
 * @brief KneeData is a data structure to store the time, angle, and millis of the knee.
 * ```cpp
 * KneeData data;
 * data.time = 1636761600;
 * data.millis = 500;
 * data.angle = 90;
 * ```
 */
struct KneeData {
  uint32_t time;
  uint16_t millis;
  uint16_t angle;
};

#endif // GLOBALS_H