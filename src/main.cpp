/**
 * @file main.cpp
 * @author Cole Rottenberg (cole.rottenberg@gmail.com) 
 * @brief The goal of this project is to enable our Knee Rehab device to store data on an SD card and when a user connects to the device via Bluetooth, the device will send the data to the user's Laptop. The project will use an ESP32 microcontroller and the Arduino IDE. The project will make use of FreeRTOS tasks to handle the Bluetooth and SD card functionality concurrently.
 * @version 0.1
 * @date 2024-11-13
 * 
 * @copyright Copyright (c) 2024
 * 
 */

// Include Libraries
#include <Arduino.h>
// For FreeRTOS tasks
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
// For BL Classic communication
#include <BluetoothSerial.h>
// For the SD card
#include <SD.h>
#include <SPI.h>

// Debug Definitions
#define DEBUG 1

// Global Instances
BluetoothSerial SerialBT;
SemaphoreHandle_t fileMutex;

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
    Serial.println("Serial Monitor is ready");
  #else
    // Open the SD card
    if (!SD.begin()) {
      Serial.begin(115200);
      Serial.println("SD Card failed to initialize, please check the card");
      return;
    }
  #endif

  // put your setup code here, to run once:
  fileMutex = xSemaphoreCreateMutex();
}

void loop() {
  // put your main code here, to run repeatedly:
}

void bluetoothTask(void *pvParameters) {
  for(;;) {
    // put your code here
  }
}

void sdCardTask(void *pvParameters) {
  for(;;) {
    // put your code here
  }
}

void readDataFromSDCardTask(void *pvParameters) {
  for(;;) {
    // put your code here
  }
}

void readPotentiometerTask(void *pvParameters) {
  for(;;) {
    // put your code here
  }
}
