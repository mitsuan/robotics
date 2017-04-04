#include <math.h>

float m[3][3]={{-0.33, 0.58, 0.33}, {-0.33, -0.58, 0.33}, {0.67, 0, 0.33}};

float x=0,y=0;

//motor pins
int frg=2 ,flg=A2;//bg=2; //ground pins
int frp=3,flp=6,bp=12; //pwm pins
int frd=4,fld=5,bd=11; //direction pins

//double sp=100;//speed
double k,s,s1,s2,s3;
double w=0;
float scale=100;
bool D1,D2,D3;



void setup() {
  Serial.begin(115200);
  pinMode(frp,OUTPUT);
  pinMode(frd,OUTPUT);
  pinMode(frg,OUTPUT);
  
  pinMode(flp,OUTPUT);
  pinMode(fld,OUTPUT);
  pinMode(flg,OUTPUT);
  
  pinMode(bp,OUTPUT);
  pinMode(bd,OUTPUT);
//  pinMode(bg,OUTPUT);

  //for ground pin
  digitalWrite(frg,0);
  digitalWrite(flg,0);
//  digitalWrite(bg,0);
}

void motion(double frs,double fls,double bs,bool d1,bool d2,bool d3)
{
  //direction
  digitalWrite(frd,d1);//front right wheel direction d1
  digitalWrite(fld,d2);//front left wheel direction d2
  digitalWrite(bd,d3);//back wheel direction d3

  //speed
  analogWrite(flp,fls);//front left wheel speed fls
  analogWrite(frp,frs);//front right wheel speed frs
  analogWrite(bp,bs);//back wheel speed bs
}


void stp()
{
  
  analogWrite(flp,0);//front left wheel speed fls
  analogWrite(frp,0);//front right wheel speed frs
  analogWrite(bp,0);//back wheel speed bs
  
}

int v=0;
void loop() 
{
v+=1;
 //***********************************  

    //direction input from PS4
//      x=PS4.getAnalogHat(RightHatX)-128;
//      y=PS4.getAnalogHat(RightHatY)-128;
//      y=-y;

        x=sin((3.14*v)/180);
        y=-1;
   //**************************************

        w=0;
    //**************************************

      //Printing x, y, w:
      
      Serial.print(F("\tY: "));
      Serial.print(y);
      Serial.print(F("\tX: "));
      Serial.print(x);
      
      Serial.print(F("\tW: "));
      Serial.print(w);
  //**************************************

  //Inverse kinematics equation
      
      s1=( (m[0][0]*x) + (m[0][1]*y) + (m[0][2]*w) );
      s2=( (m[1][0]*x) + (m[1][1]*y) + (m[1][2]*w) );
      s3=( (m[2][0]*x) + (m[2][1]*y) + (m[2][2]*w) );

  //**************************************


      //printing motor speed from above equation
      Serial.print(F("\ts1: "));
      Serial.print(s1);
      Serial.print(F("\ts2: "));
      Serial.print(s2);
      Serial.print(F("\ts3: "));
      Serial.print(s3);
      
      Serial.println();

      //**************************************

      //setting direction of rotation of motor
      D1=s1>0?0:1;
      D2=s2>0?0:1;
      D3=s3>0?1:0;

      //**************************************
      
      //multiplying scale factor
      s1=scale*abs(s1);
      s2=scale*abs(s2);
      s3=scale*abs(s3);
      
      
      //printing motor speed and direction after scaling
      Serial.print(F("\ts1: "));
      Serial.print(s1);
      Serial.print(F("\ts2: "));
      Serial.print(s2);
      Serial.print(F("\ts3: "));
      Serial.print(s3);


      Serial.print(F("\td1: "));
      Serial.print(D1);
      Serial.print(F("\td2: "));
      Serial.print(D2);
      Serial.print(F("\td3: "));
      Serial.print(D3);
      
      Serial.println();


      //**************************************
      
      motion(s1,s2,s3,D1,D2,D3);
      
    }
 



