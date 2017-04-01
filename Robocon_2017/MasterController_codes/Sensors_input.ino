/**
 * SENSORS USED:
 * *************
 * 1.RAZOR-IMU:
 * -------------
 *    functions:
 *        ->imuSetup()
 *        ->getImuValue()
 * 2.LSA-08:
 * ---------
 *    functions:
 *        ->lsa08Setup()
 *        ->getRawValue()
 *        ->getValue()
 */


//***************************
//1. CODE FOR IMU
//***************************
#include<string.h>

#define BUFF_SIZE 40
char buff[BUFF_SIZE];
String yaw_string, pitch_string, roll_string;


void imuSetup()
{
  delay(1000);    //wait for controler on IMU to start (2000ms is taken arbitrarily)
  Serial1.begin(57600);
  Serial1.write("#o0");   //stoping continuous data streaming

  ref_yaw=getImuValue();
}

float getImuValue()
{

  int i = 0;

  for(int j=0;j<40;j++)
{
  buff[j]=0;

}

//token request
Serial1.write("#f");
 delay(90);   //waiting for arbitrary time of around 90ms after which imu sends data packet

int f=1; //flag variable 

while(1)
{ 
while(Serial1.available())
{
  char c=Serial1.read();
  buff[i++]=c;
  f=0;
}

if(f==0)
{
  break;
}
}


String str=buff;
//Serial.print("imu data: ");
//Serial.println(str);
 yaw_string=str.substring(5,str.indexOf(","));
  pitch_string=str.substring(str.indexOf(",")+1,str.lastIndexOf(","));
  roll_string=str.substring(str.lastIndexOf(",")+1);

float yaw=::atof(yaw_string.c_str());
float pitch=::atof(pitch_string.c_str());
float roll=::atof(roll_string.c_str());

Serial.println();
//Serial.print("yaw : ");
//Serial.println(yaw);
//Serial.print("pitch : ");
//Serial.println(pitch);
//Serial.print("roll : ");
//Serial.println(roll);

 return yaw;
 }



 //*******************
 //2. CODE FOR LSA08
 //********************
int lsa08Analog = A9; //analog pin
int lsa08Junction = 53;// junction pin

  void lsa08Setup()
  {
    pinMode(lsa08Analog, INPUT);
    pinMode(lsa08Junction, INPUT);
  }

  int getRawLsa08AnalogValue()
  {
    int val=analogRead(lsa08Analog);;
    Serial.print("lsa08 raw value: "); Serial.println(val);
      return val;
     
  }
  
  float getLsa08AnalogValue()
  {
     float val=analogRead(lsa08Analog);
     val=(val*70)/1023.0;
     Serial.print("lsa08 value  "); Serial.println(val);
     return val;
  }
  
  bool junction()
  {
    return digitalRead(lsa08Junction);
    
  }
  
