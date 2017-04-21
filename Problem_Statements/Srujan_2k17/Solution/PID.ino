// PID VARIABLE...............
float kp = 6; // CONSTANT KP
float ki = 0.031; // CONSTANT KI
float kd = 50; // CONSTANT KD
double pid_val;//TOTAL pid_val=kp*error+ki*integral+kd*prop
float error, prop, last_error, integral;
float limit = 2000;

//...........................

// pid calculation function
void pid_calculate()//function to calculate pid values
{
  error = sensorAnalogVal() - 34;
  prop = error - last_error;
  integral = integral + error;
  integral = integral > limit ? limit : integral;
  integral = (integral < -limit && integral < 0) ? -limit : integral;
  last_error = error;
  pid_val = kp * error + ki * integral + kd * prop;
  Serial.print("error  "); Serial.print(error);
  Serial.print("last_error  "); Serial.print(last_error);
  Serial.print("prop  "); Serial.print(prop);
  Serial.print("integral  "); Serial.print(integral);
  Serial.print("pid_val  "); Serial.println(pid_val);

}

//.....................................
