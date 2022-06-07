/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-05-02
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
/*
    Ref.: https://karta146831.pixnet.net/blog/post/334779135-esp32--%E5%AF%A6%E7%8F%BE%E5%A4%9A%E5%B0%8D%E4%B8%80%E5%82%B3%E8%BC%B8%21%21%21--esp-now
*/

#define NUM_OF_DATA 3
#define SERIAL_DATA_BINARY

// Data
typedef double WIFI_DATA_t;
typedef union
{
    WIFI_DATA_t Value[NUM_OF_DATA];
    unsigned char Bytes[sizeof(WIFI_DATA_t) * NUM_OF_DATA];
} DataUnion_t;
DataUnion_t WiFi_Data_Package;

// 數據接收 Callback function
void WiFi_Receive_Callback(const uint8_t *mac, const uint8_t *incomingData, int len);

void setup()
{
    /* Initialize Serial Ports */
    Serial.begin(115200);
    delay(1000);

    /* Pin Mode Setting */
    pinMode(LED_BUILTIN, OUTPUT);

    // 初始化 ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != 0)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // 設置數據接收 callback function (接受到數據時會觸發)
    esp_now_register_recv_cb(WiFi_Receive_Callback);
}

void loop()
{
}

void WiFi_Receive_Callback(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    static int Counter;

    // Copy WiFi Data to 'WiFi_Data_Package.Bytes'
    memcpy(&WiFi_Data_Package.Bytes, incomingData, sizeof(WiFi_Data_Package.Bytes));

    for (int i = 0; i < 3; i++)
    {
        WiFi_Data_Package.Value[i] *= RAD_TO_DEG;
    }
#ifdef SERIAL_DATA_BINARY
    // Print Binary Data to Software Serial
    Serial.write('s');
    Serial.write(WiFi_Data_Package.Bytes, sizeof(WiFi_Data_Package.Bytes));
    Serial.write('f');
    Serial.flush();
#else
    // Print String Data to Software Serial static const int NUM_OF_DIGITS = 3;
    for (int i = 0; i < NUM_OF_DATA; i++)
    {
        Serial.print(WiFi_Data_Package.Value[i] * RAD_TO_DEG, NUM_OF_DIGITS);
        Serial.print(" ");
    }
    // Serial.print("(deg)");
    Serial.println();
#endif

    // LED Control
    if (Counter++ == 50)
    {
        Counter = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}