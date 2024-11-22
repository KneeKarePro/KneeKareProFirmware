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

#include <Arduino.h>
#include "globals.h"
#include "init.h"
#include "tasks.h"

void setup() {
  Serial.begin(115200);

  initSDCard(33);
  initRTC();
  initNetwork();
  initWebServer();


  fileMutex = xSemaphoreCreateMutex();
  potDataQueue = xQueueCreate(QUEUE_SIZE, sizeof(KneeData));

  // Create tasks with appropriate priorities
  xTaskCreate(readPotentiometerTask, "PotTask", 2048, NULL, 1, NULL);
  xTaskCreate(sdCardTask, "SDTask", 4096, NULL, 2, NULL);
  xTaskCreate(serverTask, "Server", 4096, NULL, 2, NULL);
}

void loop() {}
