

int  d1, d2, d3, d4;
float s1, s2, s3, s4; // speed variables of motor
float w=0, x=0, y=0; //inverse kinemaatics variables
float sp = 1;//60; // scale pitch

float ref_yaw;


//       motor configuration:
//---------------------------------------------
//     | motor2(fld,fla)   |   motor1(frd,fra) |
//---------------------------------------------
//     | motor3(bld,bla)   |   motor4(brd,bra) |
//---------------------------------------------






void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  imuSetup();
  lsa08Setup();
  motorSetup();
  
}


void loop() {
  // put your main code here, to run repeatedly:

      //motion input       
//       x = pidX();
       displayPidXValues();
       
       w=pidW();
       w=-w;
       displayPidWValues();


       y=300;

      //motion output
      invKinematics();
      pwmAndDirectionCalc();
      motion();

      displayMotionValues();
}
