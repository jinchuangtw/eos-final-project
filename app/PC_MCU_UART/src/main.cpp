#include <iostream>
#include "../include/UART/UART.h"
#include "../include/Attitude_Kinematics/Attitude_Kinematics.h"
#include <string>

/* USER SETTINGS */
#define COM_PORT_NUMBER 37
#define BAUD_RATE 115200
/* END OF USER SETTINGS */

using namespace std;
double qua[4];
double eul[3];
int status = 0;

/* Main Program Enter Point */
int main()
{
    UART_Handler UART;

    /* UART Initialization */
    if (UART.Init(COM_PORT_NUMBER, BAUD_RATE) < 0)
    {
        cout << "Can not open COM Port" << endl;

        system("pause");
        return 0;
    }
    else
    {
        cout << "Open COM Port " << COM_PORT_NUMBER << " Successfully" << endl;
    }

    int _num;
    while (1)
    {
        /* Receiving and Decoding the ppUART Data */
        int n = UART.Receive(&_num);
        
        if (n == 1)
        {
            memcpy(qua, UART.RxData.qua, sizeof(qua));
            status = UART.RxData.status;

            cout << status + " " << eul[0] << endl;

            cout << endl;
        }
        else
        {
            cout << n << " " << _num << endl;
        }

#ifdef _WIN32
        Sleep(5);
#else
        usleep(100000); /* sleep for 100 milliSeconds */
#endif
    }

    system("pause");
    return 0;
}