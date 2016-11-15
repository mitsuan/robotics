#include <PS4USB.h>
#include <math.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#define BUFF_SIZE 40
 char buff[BUFF_SIZE];

//Trasnformation matrix 
float m[4][3]={{-0.35,0.35,0.25},{-0.35,-0.35,0.25},{0.35,-0.35,0.25},{0.35,0.35,0.25}};
double x=0,y=0;

//motor pins
int frp=11,flp=12,brp=9,blp=8;//pwm pins
int frd=47,fld=49,brd=43,bld=41;//direction pins

double sp=100;//speed
double k,s,s1,s2,s3,s4;
double y1=0.7,y2=1.5,w=0;
int yaw;
int scale_factor=200;
bool D1,D2,D3,D4;

//PS4
USB Usb;
PS4USB PS4(&Usb);

//reference orientaion angle, initialised in the setup().
float refYaw=0;


void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 USB Library Started"));
    pinMode(frp,OUTPUT);
  pinMode(frd,OUTPUT);
  pinMode(flp,OUTPUT);
  pinMode(fld,OUTPUT);
  pinMode(brp,OUTPUT);
  pinMode(brd,OUTPUT);
  pinMode(blp,OUTPUT);
  pinMode(bld,OUTPUT);

   Serial1.begin(57600);
 
 //turning off continuous streaming 
  Serial1.write("#o0");
  delay(1000);
  refYaw=getYawValue();
}

void motion(double frs,double fls,double bls,double brs,bool d1,bool d2,bool d3,bool d4)
{
  digitalWrite(frd,d1);//front right wheel direction d1
  digitalWrite(fld,d2);//front left wheel direction d2
  digitalWrite(brd,d4);//back right wheel direction d4
  digitalWrite(bld,d3);//back left wheel direction d3
  analogWrite(brp,brs);//back right wheel speed brs
  analogWrite(flp,fls);//front left wheel speed fls
  analogWrite(frp,frs);//front right wheel speed frs
  analogWrite(blp,bls);//back left wheel speed bls
}


void stp()
{
  analogWrite(brp,0);//back right wheel speed brs
  analogWrite(flp,0);//front left wheel speed fls
  analogWrite(frp,0);//front right wheel speed frs
  analogWrite(blp,0);//back left wheel speed bls
  
}
void loop() 
{
  Usb.Task();

    if (PS4.connected()) 
  {
    PS4.setLed(Red);
    PS4.setLedFlash(10, 10);
    if (PS4.getAnalogHat(RightHatX) > 197 || PS4.getAnalogHat(RightHatX) < 57 || PS4.getAnalogHat(RightHatY) > 197 || PS4.getAnalogHat(RightHatY) < 57) 
  
    {
      // Converting the values of 'x' and 'y' recieved from PS4: (0 to 255) to (-128 to 127)
      x=PS4.getAnalogHat(RightHatX)-128;
      y=PS4.getAnalogHat(RightHatY)-128;
      y=-y;
     
      Serial.print(F("\tY: "));
      Serial.print(y);
      Serial.print(F("\tX: "));
      Serial.print(x);
     
     //The angle w.r.t field in which the robot is desired to move
      float theta_f=atan2(y,x);
      float yaw_imu=getYawValue();
      float yawdiff=refYaw-yaw_imu;
      yawdiff=yawdiff*(3.14/180);
      float phi_r=theta_f-yawdiff;
      Serial.print(F("\ttheta_f: "));
      Serial.print(theta_f);
      
      Serial.print(F("\tref_yaw: "));
      Serial.print(refYaw);
      Serial.print(F("\tyaw_imu: "));
      Serial.print(yaw_imu);
      
      Serial.print(F("\tyaw_diff: "));
      Serial.print(yawdiff);
      
      Serial.print(F("\tphir_r: "));
      Serial.print(phi_r);
      x=cos(phi_r);
      y=sin(phi_r);
      yaw=PS4.getAnalogHat(LeftHatX);
      if(yaw>195)
        {
          w=-1.3;//right
        }
      else if(yaw<57) 
        {
          w=1.3;
        }
      else  w=0;
      Serial.print(F("\tY: "));
      Serial.print(y);
      Serial.print(F("\tX: "));
      Serial.print(x);
      
      Serial.print(F("\tW: "));
      Serial.print(w);
      s1=( (m[0][0]*x) + (m[0][1]*y) + (m[0][2]*w) );
      s2=( (m[1][0]*x) + (m[1][1]*y) + (m[1][2]*w) );
      s3=( (m[2][0]*x) + (m[2][1]*y) + (m[2][2]*w) );
      s4=( (m[3][0]*x) + (m[3][1]*y) + (m[3][2]*w) );

      
      
      
      D1=s1>0?1:0;
      D2=s2>0?0:1;
      D3=s3>0?0:1;
      D4=s4>0?1:0;
   
      s1=scale_factor*abs(s1);
      s2=scale_factor*abs(s2);
      s3=scale_factor*abs(s3);
      s4=abs(s4)*scale_factor;

      Serial.print(F("\ts1: "));
      Serial.print(s1);
      Serial.print(F("\ts2: "));
      Serial.print(s2);
      Serial.print(F("\ts3: "));
      Serial.print(s3);
      Serial.print(F("\ts4: "));
      Serial.print(s4);
      Serial.println();
      motion(s1,s2,s3,s4,D1,D2,D3,D4);
      
    }
   else if(PS4.getAnalogHat(LeftHatX) > 197 || PS4.getAnalogHat(LeftHatX) < 57) 
      {
        x=PS4.getAnalogHat(LeftHatX);
        if(x<57)
        motion(50,50,50,50,1,0,0,1);
        else if(x>197)
        motion(50,50,50,50,0,1,1,0);
        else 
        motion(0,0,0,0,1,0,1,0);
        
      }
   
   else
   motion(0,0,0,0,D1,D2,D3,D4);
  }
 else
   motion(0,0,0,0,D1,D2,D3,D4);
}


float getYawValue()
{
 // Serial.println("inside getYawValue");
  int i = 0;

  for(int j=0;j<40;j++)
{
  buff[j]=0;

}

//requesting #ypr values
Serial1.write("#f");
 delay(50);
 
int f=1;
while(1)
{Serial.println("Inside loop1");
 
while(Serial1.available())
{
  //Serial.println("Inside loop2");

  char c=Serial1.read();
  buff[i++]=c;
  f=0;
}

if(f==0)
{
  //Serial.println("---\nbreaking out of loop!");
  break;
}
}
//Serial.print("recieved: ");
//Serial.println(buff);
//delay(500);


String str=buff; 
String st=str.substring(5,str.indexOf(","));
float yaw=::atof(st.c_str());
//Serial.print("\nyaw: ");
//Serial.println(yaw);
 
  return yaw;
 
 }

