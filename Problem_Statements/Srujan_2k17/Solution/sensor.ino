//LSA08 LINE SENSOR PINS......
int sensor_analog = A3; //analog read
int sensor_junction = 10; //junction read
int sen_jn=0;

//............................

void sensor_setup()
{
  pinMode(sensor_analog, INPUT);
  pinMode(sensor_junction, INPUT);
  
}

int sensorAnalogVal()
{

  int value_raw = (analogRead(sensor_analog));
  int value = map(value_raw, 0, 480, 0, 70);
  Serial.print("raw value: ");
  Serial.println(value_raw);
  Serial.print("value: ");
  Serial.println(value);
  return value;
  
}

bool junction()
{
  sen_jn=(digitalRead(sensor_junction));
return sen_jn;
}

