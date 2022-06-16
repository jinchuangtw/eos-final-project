#pragma once

//#include "./RS232/rs232.h"
#include "rs232.h"

#define BUF_SIZE 4096

typedef struct DataPackage
{
    double qua[4];
    double acc[3];
    int status;
} Data_t;

#define DATA_LENGTH sizeof(Data_t)

/* UART Handler Class */
class UART_Handler
{
public:
    Data_t RxData;

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

    int Receive(int* _n)
    {
        n = RS232_PollComport(cport_nr, buf, BUF_SIZE - 1);
        *_n = n;
        if (n >= DATA_LENGTH + 2)
        {
            for (i = 0; i < n; i++)
            {
                if (buf[i] == 's') // Check if the 'Start_Byte' is correct
                {
                    if (buf[i + DATA_LENGTH + 1] == 'f') // Check if the 'Finish_Byte' is correct
                    {
                        memcpy(&RxData, &buf[i + 1], DATA_LENGTH); // Copy data bytes

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
    void Close()
    {
        RS232_CloseComport(cport_nr);
    }

protected:
    int i, n;
    int cport_nr;
    int baud_rate; // Baud rate of UART
    unsigned char buf[BUF_SIZE];
    char mode[4];
};