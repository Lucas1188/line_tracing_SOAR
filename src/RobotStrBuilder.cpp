#include <RobotStrBuilder.h>

void RobotStrBuilder::pushString(String str)
{
    int len = str.length();
    if(len+index>sizeof(stringBuff))
    {
        return;
    }
    memcpy(&stringBuff[index],str.c_str(),len);
    len += index;
    stringBuff[len] = linebreak;
    index = len+1;
}

void RobotStrBuilder::print()
{
    stringBuff[index] = '\0';
    Serial.print(stringBuff);
    index = 0;
}