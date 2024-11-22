#ifndef INIT_H
#define INIT_H

#include <Arduino.h>
#include <globals.h>

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

// Function Prototypes
SDError initSDCard(uint8_t csPin);
RTCError initRTC();
NetworkError initNetwork();
WebServerError initWebServer();

#endif // INIT_H