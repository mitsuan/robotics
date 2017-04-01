// pid variables for X control

float pid_valX;//TOTAL pid_val=kp*error+ki*integral+kd*prop
float errorX, last_errorX, propX, integralX,derX;
float limitX = 2000;

// pid calculation function
float pidX()//function to calculate pid values
{
  // pid variables
  float kp = 1; // CONSTANT KP
  float ki = 0;//.0031; // CONSTANT KI
  float kd = 0;//.00123; // CONSTANT KD
  
  float sensor_val=getRawLsa08AnalogValue();
  
  errorX = sensor_val - 450;
  propX = errorX;
  integralX = integralX + errorX;
  integralX = integralX > limitX ? limitX : integralX;
  integralX = (integralX < -limitX && integralX < 0) ? -limitX : integralX;
  derX=errorX-last_errorX;
  last_errorX = errorX;
  pid_valX = kp * propX + ki * integralX + kd * derX;
  return pid_valX;
}

void displayPidXValues()
{
  Serial.print("errorX  :"); Serial.print(errorX);
  Serial.print("last_errorX  :"); Serial.print(last_errorX);
  Serial.print("proportionalX  :"); Serial.print(propX);
  Serial.print("integralX  :"); Serial.print(integralX);
  Serial.print("derivativeX  :"); Serial.print(derX);
  Serial.print("pid_valX  "); Serial.println(pid_valX);
}


// pid variables for yaw control

float pid_valW;//TOTAL pid_val=kp*error+ki*integral+kd*prop
float errorW, last_errorW, propW, integralW,derW;
float limitW = 200;

// pid calculation function
float pidW()//function to calculate pid values
{
  // pid variables
  float kp = 34; // CONSTANT KP
  float ki = 0;//.0031; // CONSTANT KI
  float kd = 6;//.0123; // CONSTANT KD
  
  float yaw_val=getImuValue();
  
  errorW = ref_yaw-yaw_val;
  propW = errorW;
  integralW = integralW + errorW;
  integralW = integralW > limitW ? limitW : integralW;
  integralW = (integralW < -limitW && integralW < 0) ? -limitW : integralW;
  derW=errorW-last_errorW;
  last_errorW = errorW;
  pid_valW = kp * propW + ki * integralW + kd * derW;
  return (pid_valW);
}

void displayPidWValues()
{
  Serial.print("errorW  :"); Serial.print(errorW);
  Serial.print("last_errorW  :"); Serial.print(last_errorW);
  Serial.print("proportionalW  :"); Serial.print(propW);
  Serial.print("integralW  :"); Serial.print(integralW);
  Serial.print("derivativeW  :"); Serial.print(derW);
  Serial.print("pid_valW  "); Serial.println(pid_valW);
}
