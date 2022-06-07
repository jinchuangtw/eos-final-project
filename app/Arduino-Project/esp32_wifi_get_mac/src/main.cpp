#include <Arduino.h>
#include <WiFi.h>

void setup()
{
    Serial.begin(115200);
    Serial.println();
#ifdef ESP8266
    Serial.print("ESP8266 Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
#elif defined ESP32
    WiFi.mode(WIFI_MODE_STA);
    Serial.print("ESP32 Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
#endif
}

void loop()
{
}