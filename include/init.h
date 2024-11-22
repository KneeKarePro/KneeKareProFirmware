#ifndef INIT_H
#define INIT_H

#include <Arduino.h>
#include <globals.h>


/**
 * @enum SDError
 * @brief Enumeration for SD Card error codes.
 *
 * This enumeration defines various error codes that can be returned by functions
 * interacting with the SD Card.
 *
 * @var SDError::SD_OK
 * Indicates that the SD Card operation was successful.
 *
 * @var SDError::SD_FAILED_TO_INIT
 * Indicates that the SD Card failed to initialize.
 *
 * @var SDError::SD_FAILED_TO_OPEN_FILE
 * Indicates that the SD Card failed to open the file.
 *
 * @var SDError::SD_FAILED_TO_WRITE_FILE
 * Indicates that the SD Card failed to write to the file.
 *
 * @var SDError::SD_FAILED_TO_READ_FILE
 * Indicates that the SD Card failed to read from the file.
 *
 * @var SDError::SD_FAILED_TO_DELETE_FILE
 * Indicates that the SD Card failed to delete the file.
 */
enum SDError {
  SD_OK = 0,
  SD_FAILED_TO_INIT = 1,
  SD_FAILED_TO_OPEN_FILE = 2,
  SD_FAILED_TO_WRITE_FILE = 3,
  SD_FAILED_TO_READ_FILE = 4,
  SD_FAILED_TO_DELETE_FILE = 5
};


/**
 * @enum RTCError
 * @brief Enumeration for Real-Time Clock (RTC) error codes.
 * 
 * This enumeration defines various error codes that can be returned by functions interacting with the RTC module.
 * 
 * @var RTCError::RTC_OK
 * Indicates that the RTC operation was successful.
 * 
 * @var RTCError::RTC_FAILED_TO_INIT
 * Indicates that the RTC module failed to initialize.
 * 
 * @var RTCError::RTC_FAILED_TO_SET_TIME
 * Indicates that the RTC module failed to set the time.
 * 
 * @var RTCError::RTC_FAILED_TO_READ_TIME
 * Indicates that the RTC module failed to read the time.
 * 
 * @note Ensure that the RTC module is properly connected before calling this function.
 * 
 */
enum RTCError {
  RTC_OK = 0,
  RTC_FAILED_TO_INIT = 1,
  RTC_FAILED_TO_SET_TIME = 2,
  RTC_FAILED_TO_READ_TIME = 3
};

/**
 * @enum NetworkError
 * @brief Enumeration for network error codes.
 *
 * This enumeration defines various error codes that can be returned by functions
 * interacting with the Wi-Fi network.
 *
 * @var NetworkError::NETWORK_OK
 * Indicates that the network operation was successful.
 *
 * @var NetworkError::NETWORK_FAILED_TO_INIT
 * Indicates that the Wi-Fi network failed to initialize.
 *
 * @var NetworkError::NETWORK_FAILED_TO_SWITCH_MODE
 * Indicates that the Wi-Fi network failed to switch modes.
 */
enum NetworkError {
  NETWORK_OK = 0,
  NETWORK_FAILED_TO_INIT = 1,
  NETWORK_FAILED_TO_SWITCH_MODE = 2
};

/**
 * @enum WebServerError
 * @brief Enumeration for web server error codes.
 *
 * This enumeration defines various error codes that can be returned by functions
 * interacting with the web server.
 *
 * @var WebServerError::WEBSERVER_OK
 * Indicates that the web server operation was successful.
 *
 * @var WebServerError::WEBSERVER_FAILED_TO_INIT
 * Indicates that the web server failed to initialize.
 *
 * @var WebServerError::WEBSERVER_FAILED_TO_START
 * Indicates that the web server failed to start.
 */
enum WebServerError {
  WEBSERVER_OK = 0,
  WEBSERVER_FAILED_TO_INIT = 1,
  WEBSERVER_FAILED_TO_START = 2
};

// Function Prototypes

/**
 * @brief Initialize the SD Card, by first creating a connection to the SD Card. 
 * After starting a SPI connectino, the function will attempt to open the data file. If the file does not exist, it will be created.
 * 
 * @param csPin The chip select pin for the SD Card
 * @return `SDError` The error code for the SD Card initialization
 * - `SD_OK` - The SD Card was initialized successfully
 * - `SD_FAILED_TO_INIT` - The SD Card failed to initialize
 * - `SD_FAILED_TO_OPEN_FILE` - The data file failed to open
 * - `SD_FAILED_TO_WRITE_FILE` - The data file failed to write
 * - `SD_FAILED_TO_READ_FILE` - The data file failed to read
 * - `SD_FAILED_TO_DELETE_FILE` - The data file failed to delete
 * 
 * @note Ensure that the SD Card is formatted as FAT32... there have been issues with the **Sandisk Ultra 32GB** SD Card
 * 
 * @example
 * ```cpp
 * SDError sdError = initSDCard(5);
 * if (sdError != SDError::SD_OK) {
 *  Serial.println("Failed to initialize SD Card");
 * }
 */
SDError initSDCard(uint8_t csPin);

/**
 * @brief Initializes the Real-Time Clock (RTC) module.
 *
 * This function initializes the RTC. If the RTC has lost power or is uninitialized,
 * it sets the RTC time to the compile time.
 *
 * @return `RTCError` indicating the status of the initialization:
 * - `RTCError::RTC_OK` - RTC initialized successfully.
 * - `RTCError::RTC_FAILED_TO_INIT` - Failed to initialize the RTC module.
 * - `RTCError::RTC_FAILED_TO_SET_TIME` - Failed to set the RTC time.
 *
 * @note Ensure that the RTC module is properly connected before calling this function.
 *
 * @example
 * ```cpp
 * RTCError rtcError = initRTC();
 * if (rtcError != RTCError::RTC_OK) {
 *   Serial.println("Failed to initialize RTC");
 * }
 * ```
 */
RTCError initRTC();

/**
 * @brief Initializes the Wi-Fi network as an Access Point (AP).
 *
 * This function sets up the ESP32 as a Wi-Fi Access Point with the specified
 * SSID and password, allowing devices to connect directly to it.
 *
 * @return `NetworkError` indicating the status of the initialization:
 * - `NetworkError::NETWORK_OK` - Network initialized successfully.
 * - `NetworkError::NETWORK_FAILED_TO_INIT` - Failed to initialize the Wi-Fi network.
 *
 * @note Adjust the SSID and password in the `initNetwork` function if necessary.
 *
 * @example
 * ```cpp
 * NetworkError netError = initNetwork();
 * if (netError != NetworkError::NETWORK_OK) {
 *   Serial.println("Failed to initialize network");
 * }
 * ```
 */
NetworkError initNetwork();

/**
 * @brief Initializes the Web Server and mDNS responder.
 *
 * This function starts the mDNS service and sets up the web server to handle
 * incoming HTTP requests. It defines the necessary routes and begins listening
 * for client connections.
 *
 * @return `WebServerError` indicating the status of the initialization:
 * - `WebServerError::WEBSERVER_OK` - Web server initialized successfully.
 * - `WebServerError::WEBSERVER_FAILED_TO_INIT` - Failed to initialize the web server.
 *
 * @note Ensure that the network is initialized before calling this function.
 *
 * @example
 * ```cpp
 * WebServerError wsError = initWebServer();
 * if (wsError != WebServerError::WEBSERVER_OK) {
 *   Serial.println("Failed to initialize web server");
 * }
 * ```
 */
WebServerError initWebServer();

#endif // INIT_H