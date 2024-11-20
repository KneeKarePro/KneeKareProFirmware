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
#include "esp32-hal-spi.h"
#include <Arduino.h>
// For FreeRTOS tasks
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
// Web Server Imports
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiAP.h>
// For the SD card
#include <FS.h>
#include <RTClib.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>

// Debug Definitions
#define DEBUG 0

// SD Error Codes
enum SDError {
  SD_OK = 0,
  SD_FAILED_TO_INIT = 1,
  SD_FAILED_TO_OPEN_FILE = 2,
  SD_FAILED_TO_WRITE_FILE = 3,
  SD_FAILED_TO_READ_FILE = 4,
  SD_FAILED_TO_DELETE_FILE = 5
};

// RTC Error Codes
enum RTCError {
  RTC_OK = 0,
  RTC_FAILED_TO_INIT = 1,
  RTC_FAILED_TO_SET_TIME = 2,
  RTC_FAILED_TO_READ_TIME = 3
};

enum NetworkError {
  NETWORK_OK = 0,
  NETWORK_FAILED_TO_INIT = 1,
  NETWORK_FAILED_TO_SWITCH_MODE = 2
};

// Web Server Error Codes
enum WebServerError {
  WEBSERVER_OK = 0,
  WEBSERVER_FAILED_TO_INIT = 1,
  WEBSERVER_FAILED_TO_START = 2
};

// Global Instances
RTC_PCF8523 rtc;
SemaphoreHandle_t fileMutex;
QueueHandle_t potDataQueue;
bool bluetoothConnected = false;
bool dataTransferred = false;
const int QUEUE_SIZE = 100;
const int POT_PIN = 34; // Adjust pin as needed
const char *DATA_FILE = "/pot_data.txt";

struct KneeData {
  uint32_t time;
  uint16_t millis;
  uint16_t angle;
};

/**
 * @brief Convert the raw value from the potentiometer to an angle
 * @param rawValue The raw value from the potentiometer
 * @return uint16_t The angle in degrees
 */
uint16_t convertToAngle(uint16_t rawValue) {
  return map(rawValue, 0, 4095, 0, 180);
};

/**
 * @brief Initialize the SD card
 * @param csPin The chip select pin for the SD card
 */
uint8_t initSDCard(uint8_t csPin = 33); // Initialize the SD card

/**
 * @brief Initialize the RTC
 */
uint8_t initRTC();

uint8_t initNetwork();

uint8_t initWebServer();

// Task Prototypes
void bluetoothTask(void *pvParameters);
void sdCardTask(void *pvParameters);
void readDataFromSDCardTask(void *pvParameters);
void readPotentiometerTask(void *pvParameters);
void serverTask(void *pvParameters);

// Helper Functions
void handleDataRequest(WebServer &server);

void setup() {

  // if DEBUG is defined, print to Serial Monitor
  Serial.begin(115200);
#if DEBUG
  while (!Serial.available())
    ;
  Serial.println(F("Serial Monitor is ready"));
#endif

  uint8_t net_code = initNetwork();
  if (net_code == NetworkError::NETWORK_OK) {
    Serial.println("Network initialized");
  } else if (net_code == NetworkError::NETWORK_FAILED_TO_INIT) {
    Serial.println("Failed to initialize network");
  }

  uint8_t ws_code = initWebServer();
  if (ws_code == WebServerError::WEBSERVER_OK) {
    Serial.println("Web Server initialized");
  } else if (ws_code == WebServerError::WEBSERVER_FAILED_TO_INIT) {
    Serial.println("Failed to initialize Web Server");
  }

  uint8_t sd_code = initSDCard(33);
  if (sd_code == SDError::SD_OK) {
    Serial.println(F("SD Card initialized"));
    Serial.println("Data will be stored in " + String(DATA_FILE));
  } else if (sd_code == SDError::SD_FAILED_TO_INIT) {
    Serial.println(F("Failed to initialize SD Card"));
  } else if (sd_code == SDError::SD_FAILED_TO_OPEN_FILE) {
    Serial.println(F("Failed to open file"));
  }

  uint8_t rtc_code = initRTC();
  if (rtc_code == RTCError::RTC_OK) {
    Serial.println(F("RTC initialized"));
  } else if (rtc_code == RTCError::RTC_FAILED_TO_INIT) {
    Serial.println(F("Failed to initialize RTC"));
  } else if (rtc_code == RTCError::RTC_FAILED_TO_SET_TIME) {
    Serial.println(F("Failed to set time"));
  }

  // put your setup code here, to run once:
  fileMutex = xSemaphoreCreateMutex();
  potDataQueue = xQueueCreate(QUEUE_SIZE, sizeof(KneeData));

  // Create tasks with appropriate priorities
  xTaskCreate(readPotentiometerTask, "PotTask", 2048, NULL, 1, NULL);
  xTaskCreate(sdCardTask, "SDTask", 4096, NULL, 2, NULL);
  xTaskCreate(serverTask, "Server", 4096, NULL, 2, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}

uint8_t initSDCard(uint8_t csPin) {
  // Initialize SPI for SD card | 5: CLK, 19: MISO, 18: MOSI, 33: CS
  SPI.begin(5, 19, 18, csPin);
  if (!SD.begin(csPin))
    return SDError::SD_FAILED_TO_INIT;
  else {
    if (!SD.exists(DATA_FILE)) {
      File dataFile = SD.open(DATA_FILE, FILE_WRITE);
      // write header to file
      if (dataFile) {
        dataFile.println("Time,Millis,Angle");
        dataFile.close();
        return SDError::SD_OK;
      }
      return SDError::SD_FAILED_TO_OPEN_FILE;
    }
  }
  return SDError::SD_OK;
}

uint8_t initRTC() {
  if (!rtc.begin())
    return RTCError::RTC_FAILED_TO_INIT;
  else {
    if (!rtc.initialized() || rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      return RTCError::RTC_FAILED_TO_SET_TIME;
    } else {
      return RTCError::RTC_OK;
    }
  }
}

uint8_t initNetwork() {
  if (WiFi.mode(WIFI_AP)) {
    Serial.println("Switched to AP mode");
    if (WiFi.softAP("KneeRehab", "password")) {
      Serial.println("Access Point started");
      Serial.println("IP Address: " + WiFi.softAPIP().toString());
      return NetworkError::NETWORK_OK;
    } else {
      Serial.println("Failed to start Access Point");
      return NetworkError::NETWORK_FAILED_TO_INIT;
    }
  } else {
    Serial.println("Failed to switch to AP mode");
    return NetworkError::NETWORK_FAILED_TO_INIT;
  }
};

uint8_t initWebServer() {
  // settings up dns
  if (!MDNS.begin("knee-rehab")) {
    Serial.println("Failed to start mDNS");
    return WebServerError::WEBSERVER_FAILED_TO_INIT;
  } else {
    Serial.println("mDNS started as knee-rehab.local");
  }
  WebServer server(80);
  server.on("/", HTTP_GET,
            [&server]() { server.send(200, "text/plain", "Hello World"); });

  server.begin();
  return WebServerError::WEBSERVER_OK;
}

// Implement the potentiometer reading task
void readPotentiometerTask(void *pvParameters) {
  KneeData data;
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    data.time = rtc.now().unixtime();
    data.millis = uint16_t(
        millis() %
        1000); // milliseconds cast to uint16_t instead of unsigned long
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
          dataFile.printf("%d,%d,%d\n", data.time, data.millis, data.angle);
          dataFile.close();
        }
        xSemaphoreGive(fileMutex);
      }
    }
    taskYIELD();
  }
}

void serverTask(void *pvParameters) {
  WebServer server(80);
  server.on("/", HTTP_GET,
            [&server]() { server.send(200, "text/plain", "Hello World"); });

  server.on("/data", HTTP_GET, [&server]() { handleDataRequest(server); });

  server.begin();
  for (;;) {
    server.handleClient();
    taskYIELD();
  }
}

// Handle and serve the data
void handleDataRequest(WebServer &server) {
  if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    File dataFile = SD.open(DATA_FILE, FILE_READ);
    if (dataFile) {
      // Set download headers
      server.sendHeader("Content-Type", "text/csv");
      server.sendHeader("Content-Disposition",
                        "attachment; filename=knee_data.csv");
      server.sendHeader("Connection", "keep-alive");
      server.setContentLength(dataFile.size());
      server.send(200, "text/csv", "");

      // Use a larger transfer buffer
      static uint8_t buffer[2048];
      size_t bytesRead;
      while ((bytesRead = dataFile.read(buffer, sizeof(buffer))) > 0) {
        server.client().write(buffer, bytesRead);
        taskYIELD();
      }

      dataFile.close();
    } else {
      server.send(404, "text/plain", "File not found");
    }
    xSemaphoreGive(fileMutex);
  } else {
    server.send(503, "text/plain", "Server busy");
  }
}
