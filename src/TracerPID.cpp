#include<TracerPID.h>
#include <Arduino.h>
double PIDClass::getCorrectionTerm(long Dt,double error)//if loop is constant time pass 1 into Dt but tune to it appropriately
{
    double dErr = error-lastError;
    pOut = error*Dt;
    iOut += error*Dt;
    dOut = -dErr/Dt;
    lastError = error;
    lastCorrection = (Kp*pOut+Ki*iOut+Kd*dOut)*divisor;
    if(dErr!=0.0)
    {
        Serial.println(dErr);
    }
    return lastCorrection;
}