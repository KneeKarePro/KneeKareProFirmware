
#include "globals.h"

RTC_PCF8523 rtc;
SemaphoreHandle_t fileMutex = NULL;
QueueHandle_t potDataQueue = NULL;

const int POT_PIN = 34;
const char *DATA_FILE = "/pot_data.txt";
const int QUEUE_SIZE = 100;