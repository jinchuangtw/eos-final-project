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

#define DATA_SERIAL Serial1
#define NUM_OF_DATA 3
#define BAUD_RATE 115200
SERIAL_DATA_TRANSFER<typeof(DATA_SERIAL)> UART_Rx;
double Data_Array_Recev[NUM_OF_DATA];

// 接收設備的 MAC 地址
const uint8_t broadcastAddress[] = {0x3C, 0x71, 0xBF, 0x18, 0x8B, 0xD0};

// Data
typedef double WIFI_DATA_t;
typedef union
{
    WIFI_DATA_t Value[NUM_OF_DATA];
    unsigned char Bytes[sizeof(WIFI_DATA_t) * NUM_OF_DATA];
} DataUnion_t;

DataUnion_t WiFi_Data_Package;

// 數據發送 Call back function
void WiFi_Transmit_Callback(const uint8_t *mac_addr, esp_now_send_status_t status);

// 主程式 
void setup()
{
    /* Initialize Serial Ports */
    Serial.begin(115200);
    UART_Rx.Init(&DATA_SERIAL, NUM_OF_DATA, BAUD_RATE);
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
        Serial.println("Failed to add peer");
        return;
    }
}

void loop()
{
    static int UART_Flag;
    static int Counter;
    UART_Flag = UART_Rx.Receive(Data_Array_Recev);

    // Serial.print(UART_Flag);
    // Serial.print(" ");
    // for (int i = 0; i < 3; i++)
    // {
    //     Serial.print(Data_Array_Recev[i] * RAD_TO_DEG);
    //     Serial.print(" ");
    // }
    // Serial.flush();
    // Serial.println();

    if (UART_Flag == 1)
    {
        memcpy(WiFi_Data_Package.Value, Data_Array_Recev, sizeof(WiFi_Data_Package));

        // 發送數據
        esp_err_t result = esp_now_send(broadcastAddress, WiFi_Data_Package.Bytes, sizeof(WiFi_Data_Package.Bytes));

        // 檢查數據是否發送成功
        if (result == ESP_OK)
        {
            Serial.println("Sent with success");

            if (Counter++ == 50)
            {
                Counter = 0;
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            }
        }
        else
        {
            Serial.println("Error sending the data");
        }
    }
}

void WiFi_Transmit_Callback(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    char macStr[18];
    String SSD;

    Serial.print("Packet to: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.println(macStr);
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? SSD = "Delivery Success" : SSD = "Delivery Fail");
    //
    if (SSD == "Delivery Success")
    {
        Serial.println("OK");
    }
    else
    {
        Serial.println("NO");
    }
    //
    Serial.println();
}