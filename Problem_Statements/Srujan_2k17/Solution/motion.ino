// MOTOR PINS..................
int mrp = 9; //right motor pwm
int mrd = 8; //right motor direction
int mlp = 11; //left motor pwm
int mld = 12; //left motor direction
//***********************************

//SPEED VARIABLE..............
float speed_max =100; //MAX SPEED OF MOTTORS
float speed_right = 0; //RIGHT MOTOR SPEED
float speed_left = 0; //LEFT MOTOR SPEED
bool dir_right = 1;
bool dir_left = 1;

//............................

void motor_setup()
{
  pinMode(mrp, OUTPUT);
  pinMode(mrd, OUTPUT);
  pinMode(mlp, OUTPUT);
  pinMode(mld, OUTPUT);

  analogWrite(mlp,0);
  analogWrite(mrp,0);
}


//motion function..............
void move_forward()//function to command motors
{
  pid_calculate();

  if (pid_val < 0)
  {
    speed_left = speed_max + pid_val;
    speed_right = speed_max;
    if (speed_left < 0)
      speed_left = 5;
  }
  else
  {
    speed_left = speed_max;
    speed_right = speed_max - pid_val;
    if (speed_right < 0)
      speed_right = 5;
  }


  analogWrite(mrp, speed_right);
  analogWrite(mlp, speed_left);
  digitalWrite(mrd, dir_right);
  digitalWrite(mld, dir_left);
}

//....................................


void forward()
{ 
  analogWrite(mrp, speed_max);
  analogWrite(mlp, speed_max);
  digitalWrite(mrd, dir_right);
  digitalWrite(mld, dir_left);
  
}
void right()
{
 
  analogWrite(mrp, 0);//speed_max/2);
  analogWrite(mlp, speed_max/2);
  digitalWrite(mrd, !dir_right);
  digitalWrite(mld, dir_left);


}

void sharp_right()
{
 
  analogWrite(mrp, speed_max/2);
  analogWrite(mlp, speed_max/2);
  digitalWrite(mrd, 1);//!dir_right);
  digitalWrite(mld, dir_left);

}

void turn_right()
{
  
  while(sensorAnalogVal()<60)
right();
    stop1();
    
  while(!(sensorAnalogVal()>32&&sensorAnalogVal()<35))
    right();

   stop1();
   
}

void turn_right_sharp()
{
    while(sensorAnalogVal()<60)
sharp_right();
    stop1();
    
  while(!(sensorAnalogVal()>32&&sensorAnalogVal()<35))
    sharp_right();

   stop1();

}
void turn_left()
{
  
  while(sensorAnalogVal()<60)
//  right();
    left();
  while(!(sensorAnalogVal()>32&&sensorAnalogVal()<35))
//    right();
    left();
  
  stop1();

}
void left()
{
  
  analogWrite(mrp, speed_max/2);
  analogWrite(mlp, 0);
  digitalWrite(mrd, dir_right);
  digitalWrite(mld, !dir_left);
 
}

void turn_back()
{
  turn_right_sharp();
  stop1();
  delay(500);
  turn_right_sharp();  
}
void stop1()
{
  delay(500);
   while(junction())
          {
            forward();
          }
  analogWrite(mrp,0 );//speed_right);
  analogWrite(mlp, 0);//speed_left);
  digitalWrite(mrd, 0);
  digitalWrite(mld, 0);
}
