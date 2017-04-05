#include<SoftwareSerial.h>
SoftwareSerial xbee(10,11); //Rx, Tx
//*******GLOBAL VARIABLES******************************
/**
 * Input to inverse kinematics equation (in PWM)
 * x: speed in x direction
 * y: speed in y direction
 * w: angular velocity
 */
float x=0,y=0,w=0;

/*****************************
 * Output to motors(in PWM)
 * ***************************
 * s[0]: front right motor
 * s[1]: front left motor
 * s[2]: back motor
 */
int s[3]={0,0,0};

/********************************
 * Direction of motors(in PWM) **
 ********************************
 * d[0]: front right motor
 * d[1]: front left motor
 * d[2]: back motor
 */
bool d[3]={1,1,1};

/*****************************
 * PWM pins of motors
 *****************************
 * s_p[0]: front right motor
 * s_p[1]: front left motor
 * s_p[2]: back motor
 */
int s_p[3]={3,6,9};

/********************************
 * Direction pins of motors
 ********************************
 * d_p[0]: front right motor
 * d_p[1]: front left motor
 * d_p[2]: back motor
 */
int d_p[3]={2,7,8};


//************************************************

void setup() {
  // put your setup code here, to run once:

  //arbitrary setup time for camera
  delay(2000);
  
  Serial.begin(115200);
  xbee.begin(9600);
  
  pinMode_setup();
  

}
void pinMode_setup()
{
  for(int i=0;i<3;i++)
  {
    pinMode(s_p[i],OUTPUT);
    pinMode(d_p[i],OUTPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  //update x,y,w values:
  if(xbee.available())
  {
    String s="";
  while(xbee.available())
  {
     char ch=xbee.read();
     s+=ch;  
  }
  Serial.print("ReceivedString : ");
  Serial.println(s);
  
  x=atof(s.substring(0,s.indexOf(",")).c_str());
  y=atof(s.substring(s.indexOf(",")+1,s.lastIndexOf(",")).c_str());
  w=atof(s.substring(s.lastIndexOf(",")+1).c_str());
  
  
  Serial.println();
  }


  inverseKinematics();
  motion();
  display_motion_values();

}

void inverseKinematics()
{
  float m[3][3]={{-0.33, 0.58, 0.33}, {-0.33, -0.58, 0.33}, {0.67, 0, 0.33}};

  s[0]=m[0][0]*x + m[0][1]*y + m[0][2]*w;
  s[1]=m[1][0]*x + m[1][1]*y + m[1][2]*w;
  s[2]=m[2][0]*x + m[2][1]*y + m[2][2]*w;

  for(int i=0;i<3;i++)
  {
    //Determining direction of motors
    if(s[i]>0)
    d[i]=1;
    else
    d[i]=0;

    //Taking absolute values of motor outputs
    s[i]=abs(s[i]);
  }
}

void motion()
{
  //pwm write to motors
  for(int i=0;i<3;i++)
  {
    analogWrite(s_p[i],s[i]);
  }

  //direction to motors
  for(int i=0;i<3;i++)
  {
    digitalWrite(d_p[i],d[i]);
  }
}

void display_motion_values()
{
  Serial.println("-------Motor Values-------");
  for(int i=0;i<3;i++)
  {
    Serial.print("\t s");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(s[i]);
    Serial.print(" , ");
    Serial.print(d[i]);
    
  }
  Serial.println();

    Serial.print("x: ");
    Serial.print(x);
    Serial.print("y: ");
    Serial.print(y);
    Serial.print("w: ");
    Serial.println(w);
}
