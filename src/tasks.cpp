#include "tasks.h"
#include "globals.h"
#include "utils.h"
#include <Arduino.h>

void readPotentiometerTask(void *pvParameters){
  KneeData data;
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    data.time = rtc.now().unixtime();
    data.millis = millis() % 1000;
    data.angle = convertToAngle(analogRead(POT_PIN));
    xQueueSend(potDataQueue, &data, 0);
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
  }
}

void sdCardTask(void *pvParameters){
  KneeData data;
  File dataFile;

  for (;;)
  {
    if (xQueueReceive(potDataQueue, &data, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        dataFile = SD.open(DATA_FILE, FILE_APPEND);
        if (dataFile)
        {
          dataFile.printf("%d,%d,%d\n", data.time, data.millis, data.angle);
          dataFile.close();
        }
        xSemaphoreGive(fileMutex);
      }
    }
    taskYIELD();
  }
}

void serverTask(void *pvParameters){
  WebServer server(80);
  server.on("/", HTTP_GET, [&server]() { server.send(200, "text/plain", "Hello World"); });
  server.on("/data", HTTP_GET, [&server]() { handleDataRequest(server); });
  server.begin();

  for (;;)
  {
    server.handleClient();
    taskYIELD();
  }
}

void handleDataRequest(WebServer &server){
  if(xSemaphoreTake(fileMutex, pdMS_TO_TICKS(100)) == pdTRUE){
    File dataFile = SD.open(DATA_FILE, FILE_READ);
    if(dataFile){
      server.sendHeader("Content-Type", "text/csv");
      server.sendHeader("Content-Disposition", "attachment; filename=knee_data.csv");
      server.send(200, "text/csv", "");

      static uint8_t buffer[2048];
      size_t bytesRead = 0;
      while((bytesRead = dataFile.read(buffer, sizeof(buffer))) > 0){
        server.client().write(buffer, bytesRead);
        taskYIELD();
      }
      dataFile.close();
    }
    else {
      server.send(503, "text/plain", "Server Busy");
    }
    }
}