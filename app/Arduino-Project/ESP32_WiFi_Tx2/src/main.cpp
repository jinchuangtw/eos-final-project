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

#include "Serial_Data_Transfer.hpp"

/*
    Ref.: https://karta146831.pixnet.net/blog/post/334779135-esp32--%E5%AF%A6%E7%8F%BE%E5%A4%9A%E5%B0%8D%E4%B8%80%E5%82%B3%E8%BC%B8%21%21%21--esp-now
*/

/* USER SETTINGS */
#define DATA_SERIAL Serial2
#define DATA_SERIAL_BAUD_RATE 115200
typedef struct DataPackage
{
    double qua[4];
    int status;
} Data_t;
/* END OF USER SETTINGS */

// UART
Data_t UART_Rx;
const size_t DATA_SIZE = sizeof(Data_t) + 2;
const char start_byte = 's';
const char finish_byte = 'f';

// 接收設備的 MAC 地址
const uint8_t broadcastAddress[] = {0x30, 0xAE, 0xA4, 0xF7, 0x89, 0x0C};

// 數據發送 Call back function
void WiFi_Transmit_Callback(const uint8_t *mac_addr, esp_now_send_status_t status);

// 主程式
void setup()
{
    /* Initialize Serial Ports */
    Serial.begin(115200);
    DATA_SERIAL.begin(DATA_SERIAL_BAUD_RATE);
    delay(1000);

    /* Pin Mode Setting */
    pinMode(LED_BUILTIN, OUTPUT);

    // 初始化 ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // 設置數據發送 Call back function
    esp_now_register_send_cb(WiFi_Transmit_Callback);

    // 绑定數據接收端
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // 檢查設備是否配對成功
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        while (1)
        {
            Serial.println("Failed to add peer");
            delay(1000);
        }
    }
}

void loop()
{
    static int IsUpdated;
    static int Counter;

    // Reading and decoding the serial data bytes
    IsUpdated = 0;
    while (DATA_SERIAL.available() >= DATA_SIZE)
    {
        if ((char)(DATA_SERIAL.read()) == start_byte) // check for the start byte
        {
            DATA_SERIAL.readBytes((uint8_t *) &UART_Rx, sizeof(Data_t));
            // DATA_SERIAL.flush();

            if ((char)(DATA_SERIAL.read()) == finish_byte) // check for the finish byte
            {
                IsUpdated = 1;
            }
            else
            {
                IsUpdated = -2;
            }
        }
        else
        {
            IsUpdated = -1;
        }
    }

    /* Print Data */
    if (IsUpdated == 1)
    {
        Serial.print(UART_Rx.status);
        Serial.print(" ");
        for (int i = 0; i < 4; i++)
        {
            Serial.print(UART_Rx.qua[i], 2);
            Serial.print(" ");
        }
        Serial.println();
        Serial.flush();
    }

    if (IsUpdated == 1)
    {
        // 發送數據 (執行這行後會呼叫 call back function 'WiFi_Transmit_Callback()' )
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &UART_Rx, sizeof(Data_t));

        // 檢查數據是否發送成功
        if (result == ESP_OK)
        {
            // Serial.println("Sent with success");

            if (Counter++ == 50)
            {
                Counter = 0;
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            }
        }
        else
        {
            // Serial.println("Error sending the data");
        }
    }
}

void WiFi_Transmit_Callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    // char macStr[18];
    // String SSD;

    // Serial.print("Packet to: ");
    // snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
    //          mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    // Serial.println(macStr);
    // Serial.print("Send status: ");
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? SSD = "Delivery Success" : SSD = "Delivery Fail");
    // //
    // if (SSD == "Delivery Success")
    // {
    //     Serial.println("OK");
    // }
    // else
    // {
    //     Serial.println("NO");
    // }
    // //
    // Serial.println();
}