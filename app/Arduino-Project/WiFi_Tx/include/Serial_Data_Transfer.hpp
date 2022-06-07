/**
 * @file Serial_Data_Transfer.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-03-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <Arduino.h>

typedef double data_t;

template <class T>
class SERIAL_DATA_TRANSFER
{
public:
    void Init(T *_Serial_Ptr, int _Number_of_Data, uint32_t Baud_Rate)
    {
        Serial_Ptr = _Serial_Ptr;
        Number_of_Data = _Number_of_Data;
        Data_Size = sizeof(data_t) * _Number_of_Data;
        Serial_Ptr->begin(Baud_Rate);
        delay(500);

        Data_Recev.Value = new data_t[_Number_of_Data];
        Data_Recev.Bytes = new char[Data_Size];

        Data_Trans.Value = new data_t[_Number_of_Data];
        Data_Trans.Bytes = new char[Data_Size];

        Start_Byte = 's';
        Finish_Byte = 'f';

        Test = 0.0f;
    }

    /* Serial Data Receiver */
    int Receive(data_t *Data_Array_Recev);
    void Transmit(data_t *Data_Array_Trans);
    unsigned long Get_Time_Receive() { return (t_receive); }
    unsigned long Get_Time_Transmit() { return (t_transmit); }

    unsigned long Dt, Dt_pre;

protected:
    union UnionData
    {
        data_t *Value;
        char *Bytes;
    };
    UnionData Data_Recev, Data_Trans;
    float Test;
    T *Serial_Ptr;

    int Data_Size;
    int Number_of_Data;
    char Start_Byte, Finish_Byte;
    char Start_Byte_Check, Finish_Byte_Check;
    int Data_Flag;
    unsigned long t_receive, t_transmit;
};

/* -------------------------------------------------------------------------------------- */
template <class T>
int SERIAL_DATA_TRANSFER<T>::Receive(data_t *Data_Array_Recev)
{
    if (Serial_Ptr->available() >= Data_Size + 2)
    {
        Dt = micros() - Dt_pre;
        Dt_pre = micros();
        while (Serial_Ptr->available() > 0)
        {
            Start_Byte_Check = Serial_Ptr->read();
            if (Start_Byte_Check == 's')
            {
                break;
            }
        }

        if (Start_Byte_Check == 's')
        {
            t_receive = micros();

            Serial_Ptr->readBytes(Data_Recev.Bytes, Data_Size);
            Finish_Byte_Check = Serial_Ptr->read();

            if (Finish_Byte_Check == 'f')
            {
                // For Vicon: 2022-03-11
                if (Data_Recev.Value[0] == 0.0f && Data_Recev.Value[1] == 0.0f && Data_Recev.Value[2] == 0.0f)
                {
                    Data_Flag = -3; // Vicon loss the tracking of object 
                }
                else
                {
                    Data_Flag = 1; // Data is updated successfully.
                    memcpy(Data_Array_Recev, Data_Recev.Value, Data_Size);
                }

                // For Serial Data Transfer Only:
                // Data_Flag = 1; // Data updated successfully
                // for (int i = 0; i < Number_of_Data; i++)
                // {
                //     Data_Array_Recev[i] = Data_Recev.Value[i];
                // }
                // memcpy(Data_Array_Recev.Bytes, Data_Recev.Bytes);
            }
            else
            {
                Data_Flag = -2; // The finish byte check is failed.
            }

            t_receive = micros() - t_receive;
        }
        else
        {
            Data_Flag = -1; // The start byte check is failed.
        }
    }
    else
    {
        Data_Flag = 0; // The data is not updated yet.
    }                  // TODO: Repeat Check

    return Data_Flag;
}

/* -------------------------------------------------------------------------------------- */
template <class T>
void SERIAL_DATA_TRANSFER<T>::Transmit(data_t *Data_Array_Trans)
{
    t_transmit = micros();

    // for (int i = 0; i < Number_of_Data; i++)
    // {
    //     Data_Trans.Value[i] = Data_Array_Trans[i];
    // }
    memcpy(Data_Trans.Value, Data_Array_Trans, Data_Size);
    Serial_Ptr->write(&Start_Byte, 1);
    Serial_Ptr->write(Data_Trans.Bytes, Data_Size);
    Serial_Ptr->write(&Finish_Byte, 1);
    // Serial_Ptr->flush(); // !!!

    t_transmit = micros() - t_transmit;
}