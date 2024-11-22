
#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
#include <globals.h>

// Function Prototypes
void readPotentiometerTask(void *pvParameters);
void sdCardTask(void *pvParameters);
void serverTask(void *pvParameters);
void handleDataRequest(WebServer &server);

#endif // TASKS_H