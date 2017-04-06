/*
Receive Acelerometer data from phone over Bluetooth.
Process the data and accordingly drive the motors to get the desired motion
  
*/
 
#include <MeetAndroid.h>


MeetAndroid meetAndroid();

//Defining motor pins
byte lmp=3;
byte lmn=5;
byte rmp=6;
byte rmn=9;

void setup()  
{
  // use the baud rate your bluetooth module is configured to 
  // not all baud rates are working well, i.e. ATMEGA168 works best with 57600
  Serial.begin(57600); 
  
  // register callback functions, which will be called when an associated event occurs.
  // - the first parameter is the name of your function (see below)
  // - match the second parameter ('A', 'B', 'a', etc...) with the flag on your Android application
  meetAndroid.registerFunction(accelerometer, 'A');  

  motor_setup();
  
}

void motor_setup()
{
    //Defining pin modes for motors
    pinMode(lmp,OUTPUT);  //left motor positive
    pinMode(lmn,OUTPUT);  //left motor negative
    pinMode(rmp,OUTPUT);  //right motor positive
    pinMode(rmn,OUTPUT);  //right motor negative
    
}

void loop()
{
  meetAndroid.receive(); // you need to keep this in your loop() to receive events
}

/*
 * This method is called constantly.
 * Compass events are sent several times a second.
 *
 * note: flag is in this case 'A' and numOfValues is 1 
 * since compass event sends exactly one single int value for heading
 */
void accelerometer(byte flag, byte numOfValues)
{
  // we use getInt(), since we know only data between 0 and 360 will be sent
  int imuData []= meetAndroid.getInt(); 
  
  
  
  

}

void turn_right(byte speed)
{
  analogWrite(lmp,speed);
  analogWrite(lmn,0);
  analogWrite(rmp,0);
  analogWrite(rmn,speed);
  
}

void turn_left()
{
  analogWrite(lmp,0);
  analogWrite(lmn,speed);
  analogWrite(rmp,speed);
  analogWrite(rmn,0);
}

void move_forward()
{
  analogWrite(lmp,speed);
  analogWrite(lmn,0);
  analogWrite(rmp,speed);
  analogWrite(rmn,0);
}

void stop()
{
  analogWrite(lmp,0);
  analogWrite(lmn,0);
  analogWrite(rmp,0);
  analogWrite(rmn,0);
}

void move_backward()
{
  analogWrite(lmp,0);
  analogWrite(lmn,speed);
  analogWrite(rmp,0);
  analogWrite(rmn,speed);
}

