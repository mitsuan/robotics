
#include <sevensegmentATC2.h>
#include<string.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <math.h>
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf


//*******************************KALMAN*****************************************************
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
uint32_t t1,t2;


// TODO: Make calibration routine
const int pinA = 4,pinB = 5,pinC = 8,pinD = 9;
//************************************************************************************


int node=0;
int rpm=0;
float pitch=0;
float roll=0;
float yaw=0;


//****display board pins**************
int pitchDisp[]={49,47,45,43};
int rollDisp[]={39,37,35,33};
int nodeDisp[]={29,27,25,23};
int rpmDisp[]={A11,A10,A9,A8};
int segPins[]={3,11,9,7,5,12,10};



void setup() {

Serial.begin(57600);           // start serial for output
  
//********************I2C SETUP****************************************  
//  i2cSetup();
 //********************************************************************

//********************KALMAN_SETUP*************************************
// kalmanSetup();
//*********************************************************************

//*************display configuration***************************

  pin_init('p',pitchDisp);
  pin_init('r',rollDisp);
  pin_init('n',nodeDisp);
  pin_init('s',rpmDisp);
  pin_init('d',segPins);

  segment_delay(500);
  digit_begin();    //defining pinModes

 //************************************************************

 //************Wire(I2C) setup****************************************
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  
  //*************************************************************


}



void loop() {



  //calculates and stores filtered pitch and roll values in :  kalAngleX, kalAngleY.
  //It also prints in Serial monitor
  kalman();
  Serial.println();

  pitch=kalAngleX;
  roll=kalAngleY;
  
  print_string('s',String(rpm).c_str());
  print_string('n',String(node).c_str());
  print_string('p',String((int)pitch).c_str());
  print_string('r',String((int)roll).c_str());
  print_string('y',String((int)yaw).c_str());

}

void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all but the last

      
          String s="";
          char ch=Wire.read();
          
          for(int i=0;i<howMany-1;i++)
              s=s+(char)Wire.read(); 
          
          if(ch=='s')
           {
              rpm=atof(s);
           }
           
            else if(ch=='y')
           {
              yaw=atof(s);
           }

            else if(ch=='n')
           {
              
            node=Wire.read()-48;
            
           }
           
  }
  
}
