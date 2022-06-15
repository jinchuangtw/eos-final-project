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
#define SERIAL_DATA_BINARY 1

// UART
typedef struct DataPackage
{
    double qua[4];
    int status;
} Data_t;
Data_t Data_Rx;
const size_t DATA_SIZE = sizeof(Data_t) + 2;
const char start_byte = 's';
const char finish_byte = 'f';

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
    while (esp_now_init() != 0)
    {
        Serial.println("Error initializing ESP-NOW");
        delay(1000);
    }

    // 設置數據接收 callback function (接受到數據時會觸發)
    esp_now_register_recv_cb(WiFi_Receive_Callback);
}

void loop()
{
    delay(1);
}

void WiFi_Receive_Callback(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    static int Counter;

    // Copy binary data
    memcpy((uint8_t *) &Data_Rx, incomingData, sizeof(Data_t));

#if SERIAL_DATA_BINARY == 1
    // Print Binary Data to Software Serial
    Serial.write('s');
    Serial.write((uint8_t *) &Data_Rx, sizeof(Data_Rx));
    Serial.write('f');
    Serial.flush();
#else

    // Print String Data to Software Serial
    Serial.print(Data_Rx.status);
    Serial.print(" ");
    for (int i = 0; i < 4; i++)
    {
        Serial.print(Data_Rx.qua[i], 2);
        Serial.print(" ");
    }
    Serial.println();
    Serial.flush();
#endif

    // LED Control
    if (Counter++ == 50)
    {
        Counter = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}