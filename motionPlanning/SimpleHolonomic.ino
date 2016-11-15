#include <PS4USB.h>
#include <math.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

//Transformation matrix for inverse kinematics equation
float m[4][3]={{-0.35,0.35,0.25},{-0.35,-0.35,0.25},{0.35,-0.35,0.25},{0.35,0.35,0.25}};

int x=0,y=0;

//Motor pins
int frp=11,flp=12,brp=9,blp=8;//pwm pins
int frd=47,fld=49,brd=43,bld=41;//direction pins
double sp=100;//speed

double k,s,s1,s2,s3,s4;
double y1=0.7,y2=1.5,w=0;
float scale=1.5;
int yaw;
bool D1,D2,D3,D4;
USB Usb;
PS4USB PS4(&Usb);

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

      
       if (PS4.getButtonClick(L1))  //Increasing the speed of motion
        {
          scale=scale+0.2;    
        }
     if (PS4.getButtonClick(R1))  //Decreasing the speed of motion
          {
            scale=scale-0.2;
          }
    PS4.setLed(Red);
    PS4.setLedFlash(10, 10);
      
    if (PS4.getAnalogHat(RightHatX) > 197 || PS4.getAnalogHat(RightHatX) < 57 || PS4.getAnalogHat(RightHatY) > 197 || PS4.getAnalogHat(RightHatY) < 57) 
  
    {
      // Converting the values of 'x' and 'y' recieved from PS4: (0 to 255) to (-128 to 127)
      x=PS4.getAnalogHat(RightHatX)-128;
      y=PS4.getAnalogHat(RightHatY)-128;
      y=-y;
      
      //For rotation of about its axis with constant angular speed
      yaw=PS4.getAnalogHat(LeftHatX);
      if(yaw>195) 
        w=-100;//right
      else if(yaw<57) 
        w=100;
      else  
        w=0;
      
      //Displaying values of 'x' , 'y' and 'w'(omega)
      Serial.print(F("\tY: "));
      Serial.print(y);
      Serial.print(F("\tX: "));
      Serial.print(x);
      Serial.print(F("\tW: "));
      Serial.print(w);
      
      //Solving Inverse Kinematics equation for obtaining required motor speed
      s1=( (m[0][0]*x) + (m[0][1]*y) + (m[0][2]*w) );
      s2=( (m[1][0]*x) + (m[1][1]*y) + (m[1][2]*w) );
      s3=( (m[2][0]*x) + (m[2][1]*y) + (m[2][2]*w) );
      s4=( (m[3][0]*x) + (m[3][1]*y) + (m[3][2]*w) );

      //Displaying Motor speeds
      Serial.print(F("\ts1: "));
      Serial.print(s1);
      Serial.print(F("\ts2: "));
      Serial.print(s2);
      Serial.print(F("\ts3: "));
      Serial.print(s3);
      Serial.print(F("\ts4: "));
      Serial.print(s4);
      Serial.println();
      
      //Specifying direction of rotation of motors 
      D1=s1>0?1:0;
      D2=s2>0?0:1;
      D3=s3>0?0:1;
      D4=s4>0?1:0;
   
      //Multilpying scale factor with the values obtained from inverse kinematics equation
      //to obtain speed value within the range 0-255.
      s1=scale*abs(s1);
      s2=scale*abs(s2);
      s3=scale*abs(s3);
      s4=scale*abs(s4);
      
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

