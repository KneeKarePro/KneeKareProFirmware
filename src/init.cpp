
#include "init.h"
#include <WiFi.h>
#include <WebServer.h>

SDError initSDCard(uint8_t csPin) {
  if (!SD.begin(csPin)) {
    return SDError::SD_FAILED_TO_INIT;
  }
  else {
    File dataFile = SD.open(DATA_FILE, FILE_APPEND);
    if (dataFile) {
      dataFile.close();
      return SDError::SD_OK;
    }
    return SDError::SD_FAILED_TO_OPEN_FILE;
  }
};

RTCError initRTC() {
  if (!rtc.begin()) {
    return RTCError::RTC_FAILED_TO_INIT;
  }
  else {
    if (!rtc.initialized() || rtc.lostPower()) {
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      return RTCError::RTC_FAILED_TO_SET_TIME;
    }
    return RTCError::RTC_OK;
  }
};

NetworkError initNetwork() {
  if (WiFi.mode(WIFI_AP)) {
    Serial.println("Switched to AP mode");
    if (WiFi.softAP("KneeRehab", "password")) {
      Serial.println("Access Point started");
      Serial.println("IP Address: " + WiFi.softAPIP().toString());
      return NetworkError::NETWORK_OK;
    }
    else {
      Serial.println("Failed to start Access Point");
      return NetworkError::NETWORK_FAILED_TO_INIT;
    }
  }
  else {
    Serial.println("Failed to switch to AP mode");
    return NetworkError::NETWORK_FAILED_TO_INIT;
  }
};

WebServerError initWebServer() {
  if (!MDNS.begin("knee-rehab")) {
    Serial.println("Failed to start mDNS");
    return WebServerError::WEBSERVER_FAILED_TO_INIT;
  }
  else {
    Serial.println("mDNS started as knee-rehab.local");
  }
  WebServer server(80);
  server.on("/", HTTP_GET,
            [&server]() { server.send(200, "text/plain", "Hello World"); });

  server.begin();
  return WebServerError::WEBSERVER_OK;
}