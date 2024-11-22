#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h>
#include "globals.h"
#include <WebServer.h> // Include WebServer for handleDataRequest

/**
 * @brief Task to read data from the potentiometer sensor.
 *
 * This FreeRTOS task reads analog values from the potentiometer at regular intervals,
 * converts them to angles, and sends the data to a queue for processing by other tasks.
 *
 * @param pvParameters Pointer to task parameters (unused in this task).
 *
 * @note Ensure that the potentiometer is connected to the correct analog pin defined by `POT_PIN`.
 *
 * @example
 * ```cpp
 * // Create the potentiometer reading task
 * xTaskCreate(readPotentiometerTask, "PotTask", 2048, NULL, 1, NULL);
 * ```
 */
void readPotentiometerTask(void *pvParameters);

/**
 * @brief Task to write queued potentiometer data to the SD card.
 *
 * This FreeRTOS task receives `KneeData` items from the `potDataQueue` and writes them
 * to the SD card file specified by `DATA_FILE`. It uses a mutex to manage concurrent
 * access to the SD card.
 *
 * @param pvParameters Pointer to task parameters (unused in this task).
 *
 * @note Ensure that the SD card is initialized and `fileMutex` is created before starting this task.
 *
 * @example
 * ```cpp
 * // Create the SD card writing task
 * xTaskCreate(sdCardTask, "SDTask", 4096, NULL, 2, NULL);
 * ```
 */
void sdCardTask(void *pvParameters);

/**
 * @brief Task to handle web server operations and client requests.
 *
 * This FreeRTOS task sets up the web server, defines request handlers,
 * and processes incoming client connections. It allows clients to retrieve
 * data or interact with the device through defined endpoints.
 *
 * @param pvParameters Pointer to task parameters (unused in this task).
 *
 * @note Ensure that the network is initialized before starting this task.
 *
 * @example
 * ```cpp
 * // Create the web server task
 * xTaskCreate(serverTask, "Server", 4096, NULL, 2, NULL);
 * ```
 */
void serverTask(void *pvParameters);

/**
 * @brief Handles client requests for potentiometer data.
 *
 * This function responds to HTTP GET requests on the `/data` endpoint.
 * It reads the potentiometer data from the SD card and sends it to the client
 * as a CSV file for download.
 *
 * @param server Reference to the `WebServer` instance handling the request.
 *
 * @note This function should be registered with the web server's route handlers.
 *
 * @example
 * ```cpp
 * // Register the data handler with the server
 * server.on("/data", HTTP_GET, [&server]() { handleDataRequest(server); });
 * ```
 */
void handleDataRequest(WebServer &server);

#endif // TASKS_H