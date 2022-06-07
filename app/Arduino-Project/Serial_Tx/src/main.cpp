#include <Arduino.h>

#define NUM_OF_DATA 3
#define LOOP_DELAY_TIME_MS 100
#define LED_TOGGLE_PERIOD_MS 500

typedef double dataType;
typedef union 
{
    dataType Value[NUM_OF_DATA];
    unsigned char Bytes[NUM_OF_DATA*sizeof(dataType)];
} UART_Data_t;
const int UART_DataSize = sizeof(UART_Data_t);
const char startByte = 's';
const char finishByte = 'f';

UART_Data_t DataTrans;

void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 0);
}

void loop()
{
    static int loop_counter;

    DataTrans.Value[0] = (dataType) loop_counter;
    DataTrans.Value[1] = (dataType) 0;
    DataTrans.Value[2] = (dataType) 1;

    Serial.write((unsigned char *) &startByte, 1);
    Serial.write(DataTrans.Bytes, UART_DataSize);
    Serial.write((unsigned char *) &finishByte, 1);

    if (loop_counter*LOOP_DELAY_TIME_MS == LED_TOGGLE_PERIOD_MS)
    {
        loop_counter = 0;
        digitalToggle(LED_BUILTIN);
    }
    loop_counter++;
    
    delay(LOOP_DELAY_TIME_MS);
}