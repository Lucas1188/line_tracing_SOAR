#include <EDifferential.h>

void EDifferential::motorEControlLoop(double correctionRequest)//Assumes -ve Correction to slow L Wheel and vice versa
//CorrectionRequest must be <= motorspeedDiff(155)
//Correction request is anolamous to rate request
//tangentianal speed is the magnitude of this vector
{   
    //check if theres allowance to speed up or needs to back down or just keep the same 
    double corMargin = abs(correctionRequest);
    if(corMargin<marginAccel)
    {
        pMotorCommand.angle_command.magnitude += maxAccel;        
    }
    else if(corMargin>marginDecel)
    {
        pMotorCommand.angle_command.magnitude += maxDecel;
    }
    pMotorCommand.angle_command.magnitude = Clamp(pMotorCommand.angle_command.magnitude,minInstSpeed,maxInstSpeed);
    
    pMotorCommand.angle_command.angle_rad += correctionRequest;
    pMotorCommand.angle_command.resolvePolar();

    pMotorCommand.resolveMotorSpeed();
}

void EDifferential::motorBrake(byte pivot)
{
    switch(pivot)
    {
        case 0b00000010://Left
        {
            pMotorCommand.angle_command = Vector2(motorMin,motorMax);
        }
        break;
        case 0b00000001://Right
        {
            pMotorCommand.angle_command = Vector2(motorMax,motorMin);
        }
        break;
        case 0b00000000:
        {
            pMotorCommand.angle_command = Vector2(motorMin,motorMin);
        }
        break;
        default:
        break;
    }
    
}