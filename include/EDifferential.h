#include <Arduino.h>
#include <vectors_line.h>
#pragma once
static double Clamp(double x, double min,double max)
{
    if(x<min)
    {
        return min;
    }
    if(x>max)
    {
        return max;
    }
    return x;
}

struct motorCommand
{
    int LMotor;
    int RMotor;
    //double targetSpeed;
    //magnitude is the inst velocity of the robot
    Vector2 angle_command;
    void resolveMotorSpeed()
    {
        LMotor = (int)angle_command.x;
        RMotor = (int)angle_command.y;
    }
};


class EDifferential
{
    public:
        EDifferential()
        {
           pMotorCommand.angle_command = Vector2(200,200);

        };
        void motorEControlLoop(double correctionRequest);
        void motorBrake(byte pivot = 0b00000000);
        motorCommand pMotorCommand;
    private:
        const double maxDecel = 10;
        const double maxAccel = 20;
        //margin in rad
        const double marginAccel = 0.0167;
        const double marginDecel = 0.167;
        const double minInstSpeed = 141.421356237;
        const double maxInstSpeed = 360.624458405;
        const int motorMin=100;
        const int motorMax=255;
        //int straightSpeed = 220;
        //int maxSpeedDiff=0;
        //const int stepChange = 10;

};