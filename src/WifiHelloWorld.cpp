/*
#include <Arduino.h>
#include <WiFi.h>

#define WIFI_SSID "Mr.WashiWashi 24"
#define WIFI_PASSWORD "NLxub8pYp4r4"

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(921600);
  Serial.println("Hello From the setup");
  pinMode(LED_BUILTIN, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

bool isConnected = false;
void loop()
{
  if (WiFi.status() == WL_CONNECTED && !isConnected)
  {
    Serial.println("Connected");
    digitalWrite(LED_BUILTIN, HIGH);
    isConnected = true;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(1000);
    isConnected = false;
  }
}
*/