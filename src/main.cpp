/**
 * @file main.cpp
 * @author Cole Rottenberg (cole.rottenberg@gmail.com)
 * @brief The goal of this project is to enable our Knee Rehab device to store
 * data on an SD card and when a user connects to the device via Bluetooth, the
 * device will send the data to the user's Laptop. The project will use an ESP32
 * microcontroller and the Arduino IDE. The project will make use of FreeRTOS
 * tasks to handle the Bluetooth and SD card functionality concurrently.
 * @version 0.1
 * @date 2024-11-13
 *
 * @copyright Copyright (c) 2024
 *
 */

// Include Libraries
#include <Arduino.h>
// For FreeRTOS tasks
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
// For BL Classic communication
#include <BluetoothSerial.h>
// For the SD card
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>

// Debug Definitions
#define DEBUG 1

// Global Instances
RTC_PCF8523 rtc;
BluetoothSerial SerialBT;
SemaphoreHandle_t fileMutex;
QueueHandle_t potDataQueue;
bool bluetoothConnected = false;
bool dataTransferred = false;
const int QUEUE_SIZE = 100;
const int POT_PIN = 34; // Adjust pin as needed
const char *DATA_FILE = "/pot_data.txt";

struct KneeData {
  uint32_t time;
  uint16_t angle;
};

uint16_t convertToAngle(uint16_t rawValue) {
  return map(rawValue, 0, 4095, 0, 180);
};

// Task Prototypes
void bluetoothTask(void *pvParameters);
void sdCardTask(void *pvParameters);
void readDataFromSDCardTask(void *pvParameters);
void readPotentiometerTask(void *pvParameters);

void setup() {
  SerialBT.begin("KneeKare Pro Device");

// if DEBUG is defined, print to Serial Monitor
#if DEBUG
  Serial.begin(115200);
  while (!Serial.available())
    ;
  Serial.println("Serial Monitor is ready");
#endif

  // SPI.begin(5, 19, 18, 33); // Initialize SPI for SD card | 5: CLK, 19: MISO,
  // 18: MOSI, 33: CS
  if (!SD.begin(33)) {
#if DEBUG
    Serial.println("SD Card failed to initialize");
#endif
  } else {
#if DEBUG
    Serial.println("SD Card initialized");
#endif
  }

  // Initialize the RTC
  if (!rtc.initialized() || rtc.lostPower()) {
    rtc.begin();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  rtc.start();
#if DEBUG
  Serial.println("RTC initialized");
#endif

  // put your setup code here, to run once:
  fileMutex = xSemaphoreCreateMutex();
  potDataQueue = xQueueCreate(QUEUE_SIZE, sizeof(KneeData));

  // Create tasks with appropriate priorities
  xTaskCreate(readPotentiometerTask, "PotTask", 2048, NULL, 1, NULL);
  xTaskCreate(sdCardTask, "SDTask", 4096, NULL, 2, NULL);
  xTaskCreate(bluetoothTask, "BTTask", 4096, NULL, 3, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// Implement the potentiometer reading task
void readPotentiometerTask(void *pvParameters) {
  KneeData data;
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    data.time = rtc.now().unixtime();
    data.angle = convertToAngle(analogRead(POT_PIN));

    xQueueSend(potDataQueue, &data, 0);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10)); // 100Hz sampling
  }
}

// Implement the SD card writing task
void sdCardTask(void *pvParameters) {
  KneeData data;
  File dataFile;

  for (;;) {
    if (xQueueReceive(potDataQueue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        dataFile = SD.open(DATA_FILE, FILE_APPEND);
        if (dataFile) {
          dataFile.printf("%d,%d\n", data.time, data.angle);
          dataFile.close();
        }
        xSemaphoreGive(fileMutex);
      }
    }
    taskYIELD();
  }
}

// Implement the Bluetooth task
void bluetoothTask(void *pvParameters) {
  File dataFile;

  for (;;) {
    if (SerialBT.connected()) {
      if (!bluetoothConnected) {
        bluetoothConnected = true;
#if DEBUG
        Serial.println("Bluetooth Connected");
#endif
      }

      // Transfer data when connected
      if (!dataTransferred &&
          xSemaphoreTake(fileMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        dataFile = SD.open(DATA_FILE);
        if (dataFile) {
          while (dataFile.available()) {
            SerialBT.write(dataFile.read());
          }
          dataFile.close();

          // Delete file after successful transfer
          SD.remove(DATA_FILE);
          dataTransferred = true;
        }
        xSemaphoreGive(fileMutex);
      }
    } else {
      bluetoothConnected = false;
      dataTransferred = false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
