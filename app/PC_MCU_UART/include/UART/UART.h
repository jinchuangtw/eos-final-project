#pragma once

#include "./RS232/rs232.h"

#define BUF_SIZE 4096

/* TODO: Dynamic Array */
#define NUM_OF_DATA 3
typedef double data_type;
typedef union
{
    data_type Value[NUM_OF_DATA];
    char Bytes[sizeof(data_type) * NUM_OF_DATA];
} UART_Data_t;
/*------------------------ */

/* UART Handler Class */
class UART_Handler
{
public:
    UART_Data_t RxData;

    void Set_COM_Port(int COMPort_Number)
    {
        cport_nr = COMPort_Number - 1; /* /dev/ttyS0 (COM1 on windows) */
    }

    void Set_Baud_Rate(int _baud_rate)
    {
        baud_rate = _baud_rate;
    }

    int Init(int _COM_Port, int _Baud_Rate = 115200)
    {
        Set_COM_Port(_COM_Port);
        Set_Baud_Rate(_Baud_Rate);

        mode[0] = '8';
        mode[1] = 'N';
        mode[2] = '1';
        mode[3] = '\0';

        if (RS232_OpenComport(cport_nr, baud_rate, mode, 0))
        {
            return -1;
        }
        return 1;
    }

    int Receive()
    {
        n = RS232_PollComport(cport_nr, buf, BUF_SIZE - 1);

        if (n > Package_Size+1)
        {
            for (i = 0; i < n; i++)
            {
                if (buf[i] == 's') // Check if the 'Start_Byte' is correct
                {
                    if (buf[i + Package_Size + 1] == 'f') // Check if the 'Finish_Byte' is correct
                    {
                        memcpy(RxData.Bytes, &buf[i + 1], Package_Size); // Copy data bytes

                        return 1;
                    }

                    return -2;
                }
            }
        }
        else
        {
            return -1;
        }
    }
  
protected:
    int i, n;
    int cport_nr;
    int baud_rate; // Baud rate of UART
    unsigned char buf[BUF_SIZE];
    char mode[4];

    size_t Package_Size = sizeof(data_type) * NUM_OF_DATA;
};