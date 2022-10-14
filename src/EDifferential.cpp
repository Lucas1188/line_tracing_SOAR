#include <EDifferential.h>

void EDifferential::motorEControlLoop(double correctionRequest)//Assumes -ve Correction to slow L Wheel and vice versa
//CorrectionRequest must be <= motorspeedDiff(155)
//Correction request is anolamous to rate request
//tangentianal speed is the magnitude of this vector
{   

    //control map linear equations
    //1) y=155+(100/255)x
    //2) x = 255
    //tan t = opp / adj opp = 100
    //check if theres allowance to speed up or needs to back down or just keep the same 
    double powerlimit = 155;
    double corMargin = Clamp(abs(correctionRequest),0,PI/4-0.1);
    //Serial.println(corMargin);
    double diff = powerlimit/tan(PI/4-corMargin);
    if (correctionRequest<0)
    {
        pMotorCommand.LMotor = diff+motorMin;
        pMotorCommand.RMotor = powerlimit+motorMin;
        /* code */
    }
    else
    {
        pMotorCommand.RMotor = diff+motorMin;
        pMotorCommand.LMotor = powerlimit+motorMin;
    }
    
    // if(corMargin<marginAccel)
    // {
    //     pMotorCommand.angle_command.magnitude += maxAccel;        
    // }
    // else if(corMargin>marginDecel)
    // {
    //     pMotorCommand.angle_command.magnitude += maxDecel;
    // }
    // pMotorCommand.angle_command.magnitude = Clamp(pMotorCommand.angle_command.magnitude,minInstSpeed,maxInstSpeed);
    
    // pMotorCommand.angle_command.angle_rad += correctionRequest;
    // pMotorCommand.angle_command.resolvePolar();

    // pMotorCommand.resolveMotorSpeed();
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