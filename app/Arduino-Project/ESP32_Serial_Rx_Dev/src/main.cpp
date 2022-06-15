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

/* USER SETTINGS */
#define DATA_SERIAL Serial2
#define DATA_SERIAL_BAUD_RATE 115200
#define BUF_SIZE 1024
/* END OF USER SETTINGS */

// 主程式
void setup()
{
    Serial.begin(115200);
    DATA_SERIAL.begin(DATA_SERIAL_BAUD_RATE);

    delay(500);

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    static int i;                       // Counter of serial data
    static unsigned char buf[BUF_SIZE]; // Buffer of serial data

    for (i = 0; DATA_SERIAL.available() > 0; i++)
    {
        buf[i] = DATA_SERIAL.read(); // Read one byte from serial
    }

    // Print data
    if (i > 0)
    {
        // Serial.print("Received " + String(i) + " Bytes: ");
        Serial.write((char *)buf, i); // Write i bytes to serial (buf[0], buf[1], ..., buf[i] <-- i+1 binary array data)
    }
    memset(buf, 0, i); // Reset buffer (clean i bytes of 'buf')

    static int counter;
    if (counter++ == 50)
    {
        counter = 0;
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    delay(10);
}