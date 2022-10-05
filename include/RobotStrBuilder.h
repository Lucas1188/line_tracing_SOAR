#include <Arduino.h>

#pragma once

enum
{
    PID,
    EDIF,
    SENSOR_HAL,
    SENSOR_P
};

class RobotStrBuilder
{
    public:
        RobotStrBuilder()
        {

        };
        void pushString(String str);
        void print();
    private:
        char stringBuff[255];
        byte index=0;
        const char linebreak ='\n';

};