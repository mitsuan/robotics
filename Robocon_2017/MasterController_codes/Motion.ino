//motor pins
byte frd = 47, fld = 49, bld = 35, brd = 43, fra = 11, fla = 12, bla = 5, bra = 9;

void motorSetup()
{
  pinMode(fra, OUTPUT);
  pinMode(frd, OUTPUT);
  pinMode(fla, OUTPUT);
  pinMode(fld, OUTPUT);
  pinMode(bra, OUTPUT);
  pinMode(brd, OUTPUT);
  pinMode(bla, OUTPUT);
  pinMode(bld, OUTPUT);
}

// inverse kinematics matrix 
float a11 = -0.35, a12 = 0.35, a13 = 0.25;
float a21 = -0.35, a22 = -0.35, a23 = 0.25;
float a31 = 0.35, a32 = -0.35, a33 = 0.25;
float a41 = 0.35, a42 = 0.35, a43 = 0.25;

void invKinematics()
{
      s1 = sp * ((a11 * x) + (a12 * y) + (a13 * w));
      s2 = sp * ((a21 * x) + (a22 * y) + (a23 * w));
      s3 = sp * ((a31 * x) + (a32 * y) + (a33 * w));
      s4 = sp * ((a41 * x) + (a42 * y) + (a43 * w));
}

void pwmAndDirectionCalc()
{
  //direction calculations   
    d1=s1<0?0:1;
    d2=s2<0?0:1;
    d3=s3<0?0:1;
    d4=s4<0?0:1;
    
  //magnitude 
    s1=abs(s1);
    s2=abs(s2);
    s3=abs(s3);
    s4=abs(s4);
     
}

void motion()/// commands the motors to drive
{

  digitalWrite(frd, d1);
  digitalWrite(fld, d2);
  digitalWrite(brd, d4);
  digitalWrite(bld, d3);
  analogWrite(fra, s1);
  analogWrite(fla, s2);
  analogWrite(bla, s3);
  analogWrite(bra, s4);

}
void stopp()
{
  digitalWrite(frd, d1);
  digitalWrite(fld, d2);
  digitalWrite(brd, d4);
  digitalWrite(bld, d3);
  analogWrite(fra, 0);
  analogWrite(fla, 0);
  analogWrite(bla, 0);
  analogWrite(bra, 0);

}

void displayMotionValues()
{
      Serial.print("x=");
      Serial.print(x);
      Serial.print(" y=");
      Serial.print(y);
      Serial.print(" w=");
      Serial.print(w);
      Serial.print(" s1=");
      Serial.print(s1);
      Serial.print(" s2=");
      Serial.print(s2);
      Serial.print(" s3=");
      Serial.print(s3);
      Serial.print(" s4=");
      Serial.print(s4);
      Serial.print(" d1=");
      Serial.print(d1);
      Serial.print(" d2=");
      Serial.print(d2);
      Serial.print(" d3=");
      Serial.print(d3);
      Serial.print(" d4=");
      Serial.println(d4);
}
