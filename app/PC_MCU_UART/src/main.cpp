#include <iostream>
#include "../include/UART/UART.h"
#include "../include/Attitude_Kinematics/Attitude_Kinematics.h"
#include <string>

/* USER SETTINGS */
#define COM_PORT_NUMBER 27
#define BAUD_RATE 115200
/* END OF USER SETTINGS */

using namespace std;

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

    while (1)
    {
        /* Receiving and Decoding the ppUART Data */
        int n = UART.Receive();
        if (n == 1)
        {
            for (int i = 0; i < 3; i++)
            {
                cout << UART.RxData.Value[i] << " ";
            }
            cout << endl;
        }

#ifdef _WIN32
        Sleep(1);
#else
        usleep(100000); /* sleep for 100 milliSeconds */
#endif
    }

    system("pause");
    return 0;
}