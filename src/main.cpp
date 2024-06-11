#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

// ESP32 Bulit-in LED pin at pin 2
const int LED_BUILTIN = 2;

int i = 0;

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial.available())
  {
    /* Wait til connect is established */
  }
  Serial.println("Hello World");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(i++);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}