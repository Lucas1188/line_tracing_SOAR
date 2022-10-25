//#include <AFMotor.h>
#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <SPI.h>
#include <TracerPID.h>
#include <vectors_line.h>
#include <EDifferential.h>
//JHELLO
#define DEBUGMSG 1
enum MOVESTRAT
{
  PCONTROL_L=1,
  PCONTROL_R=-1,
  BESTACCEL=0,
  PIVOTL=4,
  PIVOTR=-4,
  BESTDECEL=-99
};
enum RUN_STATE
{
  START,
  B_CHECK,
  PIVOT,
  INTERCEPT,
  ALIGN,
  GRAB
};
long lastMillis;

class sensor_buff
{
  public:
    sensor_buff(){};
    //int read;
    int indexPos;
    int pin;  
    //int bias;
    int thresholdBlack=900;
    bool outofline;
    virtual int readSensor()
    {
      //read = analogRead(pin);
      if(analogRead(pin)<thresholdBlack)
      {  
        return 1; //tune something here
      }
      else
      {
        return 0; //tune something here
      }
    }  
};

class linetrackingSensor
{
  public:
  linetrackingSensor(int _checkPointThreshold, int _bReadThreshold)
  {
    checkPointThreshold = _checkPointThreshold;
    bReadThreshold = _bReadThreshold;
    bufferThreshold = _checkPointThreshold;
  };

  int checkPointThreshold=0;
  int bReadThreshold = 0;
  int bufferThreshold = 0;
  byte BinaryRead  = 0b00000000;
  bool ReadingA = false;
  bool writingB = false;
  int bitCount = 0;
    double frameRead(sensor_buff* sensearray,RUN_STATE run_state)
    {
      auto senseResult = pollLayer(sensearray);
#ifdef DEBUGMSG
      //Serial.println(senseResult,BIN);
      
#endif
      switch (run_state)
      {
      case START:
      {
        if(senseResult == 0b00000100)
        {
          bufferThreshold-=1;
        }
        if(bufferThreshold<=0)
        {
          run_State = B_CHECK;
          bufferThreshold = bReadThreshold;
          ReadingA = true;
        }
      }
        break;
      case B_CHECK:
      {
        if(ReadingA)
        {
          if(senseResult!=0b00000100)
          {
            ReadingA = false;
          }
        }

        if(!ReadingA)
        {
          if(senseResult == 0b00000100 && writingB)
          {
            ReadingA = true;
            writingB = false;
          }
          else
          {
            bReadThreshold -=1;
          }

          if(bReadThreshold <=0 && !writingB)
          {
            writingB = true;
            bitCount +=1;
            if(senseResult == 0b00001110)
            {
              BinaryRead = BinaryRead <<1;
              BinaryRead +=1;
            }
            if(senseResult == 0b00001100)
            {
              BinaryRead = BinaryRead <<1;
            }
          }
        }
        if(bitCount==6)
        {
          
        }
        
      }
      break;
      default:
        break;
      }
      if(senseResult==0b00001110)
      {
        //Check Point
      }

      if(senseResult == 0b00000100)
      {
        //Serial.println("On Line");
        return 0; //In this system it means hold your rate
      }
      else
      {
        double sense = (double)(-(senseResult>>3) + (((senseResult>>1 & 0b00000001)| senseResult<<1) & 0b00000011));
        // Serial.print("a: ");
        //Serial.println((((senseResult>>1 & 0b00000001)| senseResult<<1) & 0b00000011),BIN);
         return sense;
      }
      //Serial.println("Nothing");
      return 0;
    }
    
    
  private:
    MOVESTRAT pStrat;
    uint32_t stratSwitch=0;
    byte pollLayer(sensor_buff* sensearray)
    {
      byte poll = 0b00000000;
      for(int i =0;i<5;i++)
      {
        poll |= (sensearray[i].readSensor()<<(i));
        
      }
      poll &= 0b00011111;
      return poll;
    }
};

sensor_buff sensorArray[5];
linetrackingSensor ltsA(10,10);

// AF_DCMotor motor_left(1);
// AF_DCMotor motor_right(4);
//WTF are the pins? IN1234, 2345, PWM 6,9
#define IN1_PIN 2
#define IN2_PIN 3
#define IN3_PIN 4
#define IN4_PIN 5

#define ENA 9
#define ENB 6

#define PGAIN 0.9
#define IGAIN 0.1
#define DGAIN 12.0

PIDClass softCon(PGAIN,IGAIN,DGAIN);

EDifferential eDif;

char debugmessage[32];

//IR_1 is on the left
const int IR_1 = A0;
const int IR_2 = A1;
const int IR_3 = A2;
const int IR_4 = A3;
const int IR_5 = A4;

//#define MAX_STRAIGHTSPEED 200
//#define MAX_ACCEL 1;

int threshold = 400;
int currentSpeedL = 200;
int currentSpeedR = 200;
bool isLocked = true;



const byte SecretCode[]= 
{
  0b11010011,
  0b01111101,
  0b11011110,
  0b11110011
};

int ReadCount = 0;
RUN_STATE run_State = START;
void setup() 
{
  pinMode(IR_1,INPUT);
  pinMode(IR_2,INPUT);
  pinMode(IR_3,INPUT);
  pinMode(IR_4,INPUT);
  pinMode(IR_5,INPUT);

  pinMode(IN1_PIN,OUTPUT);
  pinMode(IN2_PIN,OUTPUT);
  pinMode(IN3_PIN,OUTPUT);
  pinMode(IN4_PIN,OUTPUT);

  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  Serial.begin(115200);

  sensorArray[0].pin = IR_1;
  sensorArray[1].pin = IR_2;
  sensorArray[2].pin = IR_3;
  sensorArray[3].pin = IR_4;
  sensorArray[4].pin = IR_5;
  
  digitalWrite(IN1_PIN,LOW);
  digitalWrite(IN2_PIN,HIGH);
  digitalWrite(IN3_PIN,LOW);
  digitalWrite(IN4_PIN,HIGH);

  lastMillis = millis();
}

#define LOOP_MINTIME 12L

void loop() 
{
  #pragma region  LEGACY

  switch(run_State)
  {
    case START:
    {

    }
    break;
    case B_CHECK:
    {

    }
    break;
  }
  
#pragma endregion
  auto startTime = millis();
  auto deltaTime = startTime-lastMillis;
  auto line_errors = true;

  if(deltaTime>LOOP_MINTIME)
  {
    //Instantiate a PID Class pls
    auto fRead = ltsA.frameRead(sensorArray,run_State);
    // Pivot Control
    //Serial.println(fRead);
    auto dRead = softCon.getCorrectionTerm(deltaTime, fRead);// Store this into an array next time for more processing
    Serial.println(dRead);
    eDif.motorEControlLoop(dRead);
    lastMillis = millis()-startTime;
    
    
    //Always FORWARD!!!!
    

    analogWrite(ENA,eDif.pMotorCommand.LMotor);
    analogWrite(ENB,eDif.pMotorCommand.RMotor);
  }
  else
  {
    if(line_errors)
    {
      //Serial.println(deltaTime);
    }
  }
}

