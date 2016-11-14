#include <PS4USB.h>
#include <math.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif


//Storing IMU values
#define BUFF_SIZE 40
 char buff[BUFF_SIZE];


//inverse matrix for Inverse Kinematics calculation
float m[4][3]={{-0.35,0.35,0.25},{-0.35,-0.35,0.25},{0.35,-0.35,0.25},{0.35,0.35,0.25}};

//inverse Kinematics parameters
double x=0,y=0,w=0;

//Analog Hat values of PS4
double rhy=0,rhx=0;
double lhy=0,lhx=0;

//output of inverse Kinematics calculation  
double s1,s2,s3,s4; //Motor speeds: FR,FL,BL,BR
bool D1,D2,D3,D4; //Motor directions: FR,FL,BL,BR
int scale_factor=200;

//motor pins
int frp=11,flp=12,brp=9,blp=8;//pwm pins
int frd=47,fld=49,brd=43,bld=41;//direction pins

//unused
double sp=100;//speed
double k,s;
double y1=0.7,y2=1.5;

int yaw;




float max_pid_output=0;

float lastProp=0;
float p=0;
float i=0;
float d=0;
float kp=1.55;
float ki=0;
float kd=0.4;
char v='l',kk='w';
char pidin='l';
int kkk;


USB Usb;
PS4USB PS4(&Usb);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;
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
  for(int ii=0;ii<5;ii++)
  refYaw=(getYawValue());
  //refYaw=map(refYaw,-180,180,0,360);
 
}

void motion()
{
  digitalWrite(frd,D1);//front right wheel direction d1
  digitalWrite(fld,D2);//front left wheel direction d2
  digitalWrite(brd,D4);//back right wheel direction d4
  digitalWrite(bld,D3);//back left wheel direction d3
  analogWrite(brp,s4);//back right wheel speed brs
  analogWrite(flp,s2);//front left wheel speed fls
  analogWrite(frp,s1);//front right wheel speed frs
  analogWrite(blp,s3);//back left wheel speed bls
}


void stp()
{
  analogWrite(brp,0);//back right wheel speed brs
  analogWrite(flp,0);//front left wheel speed fls
  analogWrite(frp,0);//front right wheel speed frs
  analogWrite(blp,0);//back left wheel speed bls
  
}



float pid(float err)
{
  p=err;
  i+=p;
  d=p-lastProp;

  lastProp=p;  
  float out=kp*p+ki*i+kd*d;

      //  Serial.print(F("\tout: "));
      //Serial.print(out);

  if(out>0)
  out=(out*(2.0119)/(max_pid_output)+0.1);
  else
   out=(out*(-2.0119)/(-max_pid_output)-0.1);
  return out;
}


void max_pid_out()
{
    max_pid_output=kp*3.14;
  
}

void invKinematics()
{
  //Inverse Kinematics calculation for motor speeds
      s1=( (m[0][0]*x) + (m[0][1]*y) + (m[0][2]*w) );   //front right
      s2=( (m[1][0]*x) + (m[1][1]*y) + (m[1][2]*w) );   //front left
      s3=( (m[2][0]*x) + (m[2][1]*y) + (m[2][2]*w) );   //back left
      s4=( (m[3][0]*x) + (m[3][1]*y) + (m[3][2]*w) );   //back right

      
      
      //motor direction
      D1=s1>0?1:0;
      D2=s2>0?0:1;
      D3=s3>0?0:1;
      D4=s4>0?1:0;
   
      s1=scale_factor*abs(s1);
      s2=scale_factor*abs(s2);
      s3=scale_factor*abs(s3);
      s4=scale_factor*abs(s4);

      Serial.print(F("\ts1: "));
      Serial.print(s1);
      Serial.print(F("\ts2: "));
      Serial.print(s2);
      Serial.print(F("\ts3: "));
      Serial.print(s3);
      Serial.print(F("\ts4: "));
      Serial.print(s4);
      Serial.println();

}





void pidinput()
        {
          pidin=Serial.read();
           if(pidin=='p')
          {
            Serial.println("  enter value for kp.....    ");
            kkk=0;
            while(Serial.available() > 0)
                  {
                    kkk=kkk*10+Serial.read()-'0';
                  }
            kp=kkk;
          }

          else if(pidin=='i')
          {
            Serial.println("  enter value for ki.....    ");
            kkk=0;
            while(Serial.available() > 0)
                  {
                    kkk=kkk*10+Serial.read()-'0';
                  }
            ki=kkk;
          }
          else if(pidin=='d')
          {
            Serial.println("  enter value for kd.....    ");
            kkk=0;
            while(Serial.available() > 0)
                  {
                    kkk=kkk*10+Serial.read()-'0';
                  }
            kd=kkk;
          }

          else if(pidin=='a')
          {
            Serial.println("  enter value for kp.....    ");
            kkk=0;
            while(Serial.available() > 0)
                  {
                    kk=Serial.read();
                    if(kk=='/') break;
                    kkk=kkk*10+kk-'0';
                  }
            kp=kkk;

            Serial.println("  enter value for ki.....    ");
            kkk=0;
            while(Serial.available() > 0)
                  {
                    
                    kk=Serial.read();
                    if(kk=='/') break;
                    kkk=kkk*10+kk-'0';
                  }
            ki=kkk;

            Serial.println("  enter value for kd.....    ");
            kkk=0;
            while(Serial.available() > 0)
                  {
                    
                    kk=Serial.read();
                    if(kk=='/') break;
                    kkk=kkk*10+kk-'0';
                  }
            kd=kkk;
          }
          else  Serial.println("  entered invalid choice...   ");
          
          Serial.print("  kp = ");Serial.print(kp);Serial.print("  ki = ");Serial.print(ki);Serial.print("  kd = ");Serial.println(kd);
        delay(1000);

          
        }
 

void setInvKinematicsParam(int RHX,int RHY,int LHX,int LHY)
{
        /*
       * CALCULATIONS FOR FIELD-ORIENTED KILLOUGH-DRIVE
       * 
       * param.: theta_f
       * descrp.: The angle made by the vector(in the desired direction of motion) from the positive x-axis of the reference axes of the FIELD.
       * calc:  Calculated as the inverse tan of the y and x values recieved from PS4
       * 
       * param.: yaw_imu
       * descrp.: yaw value recieved from IMU
       * calc:  As calculated in the function getYawValue();
       * 
       * param.: yaw_diff
       * descrp.: The angle rotated by the BOT's reference axes from the reference axes of the FIELD.
       * calc:  Calculated as the difference of refYaw and yaw_imu
       * 
       * param.: phi_r
       * descrp.: The angle made by the vector(in the direction along which the bot should move) from the positive x-axis of the refernce axis of the BOT.
       * calc:  Calculated as the difference of theta_f and yaw_diff
       */         
       
      double theta_f=atan2(RHY,RHX);
      double yaw_imu=getYawValue();
      double yawdiff=refYaw-yaw_imu;
      yawdiff=yawdiff*(PI/180);
      double phi_r=theta_f-yawdiff;
      
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


      //parameters of Inverse Kinematics equation
      x=cos(phi_r);
      y=sin(phi_r);


      if(LHX!=0||LHY!=0)
      {
      double reqYaw=atan2(LHY,LHX);
      w=-pid(reqYaw-yawdiff);
      }
     /* yaw=PS4.getAnalogHat(LeftHatX);
      if(yaw>195)
        {
          w=-1.3;//right
        }
      else if(yaw<57) 
        {
          w=1.3;
        }*/
      else  w=0;
      
      Serial.print(F("\tY: "));
      Serial.print(y);
      Serial.print(F("\tX: "));
      Serial.print(x);
      
      Serial.print(F("\tW: "));
      Serial.print(w);


}


void loop() 
{

  max_pid_out();
while ((Serial.available() > 0)&& pidin!='e')
        {
          pidinput();
        }

  
  Usb.Task();




    //Serial.println("loop");
    if (PS4.connected()) 
  {
    PS4.setLed(Green);
    PS4.setLedFlash(10, 10);
    
    if(PS4.getAnalogHat(LeftHatX) > 210 || PS4.getAnalogHat(LeftHatX) < 40 || PS4.getAnalogHat(LeftHatY) > 210 || PS4.getAnalogHat(LeftHatY) < 40) 
      {
        lhx=PS4.getAnalogHat(LeftHatX)-128;
        lhy=PS4.getAnalogHat(LeftHatY)-128;

      //for getting positive  and negative y-values for upward and downward motion of the right analogHat respectively
      lhy=-lhy;
      
      Serial.print(F("\tLHY: "));
      Serial.print(lhy);
      Serial.print(F("\tLHX: "));
      Serial.print(lhx);
        double yaw_imu=getYawValue();
      double yawdiff=yaw_imu-refYaw;

      yawdiff=(refYaw>0 && yawdiff<-180) ? (yawdiff+360):yawdiff;
      yawdiff=(refYaw<0 && yawdiff>180) ? (yawdiff-360):yawdiff;
      double reqYaw=atan2(lhy,lhx)*180/PI;////D1=s1>0?1:0;
      reqYaw=reqYaw>0?(map(reqYaw,0,180,90,-90)):(map(reqYaw,0,-180,-90,90)+180);
      reqYaw=reqYaw>180?(map(reqYaw,180,270,-90,0)-90):(reqYaw);////to shift the joystick values by 90 degree clockwise 
        double error=(reqYaw-yawdiff)*PI/180;
      w=-pid(error);

        Serial.print(F("\trefYaw: "));
      Serial.print(refYaw);
       Serial.print(F("\tyaw_imu: "));
      Serial.print(yaw_imu);
      Serial.print(F("\tyawdiff: "));
      Serial.print(yawdiff);
      Serial.print(F("\treqYaw: "));
      Serial.print(reqYaw);
      Serial.print(F("\terror: "));
      Serial.print(error);
      Serial.print(F("\tw: "));
      Serial.print(w);

      invKinematics();
      motion();    
      }
   
   else  
   stp();
//   motion(0,0,0,0,D1,D2,D3,D4);
  }
 else
 stp();
//   motion(0,0,0,0,D1,D2,D3,D4);
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
{//Serial.print("loop1");
 
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

