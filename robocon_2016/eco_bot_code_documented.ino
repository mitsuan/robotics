

#include <Servo.h>

Servo ecobot;                //OBJECT for servo

int sensor[500];            //STORING ANALOG VALUE
const byte analogPin = 0;   // Connect AN output of LSA08 to analog pin 0
const byte junctionPulse = 33;
unsigned int junctionCount = 0;
int readVal,pv,ppev;   


//MAXTURN VALUES
int maxturn_right=64;  //70
int maxturn_right1=55;            //MAXTURN FOR JUNCTION
int maxturn_left=64;  //42


//VARIABLES FOR PID CALCULATION
double integral=0,derivative=0,proportional=0;
float error_val;
double kp=0.85510;
double ki=0.003;
double kd=0.03555;

//Variables for Digital Mode
int x=0;                      //FOR CONTROLLING THE ROW NUMBER OF prevValue[] array
int dsensor[8];               //FOR STORING INSTANTANEOUS DIGITAL VALUES OF THE SENSOR
int prevValue[10][8];         //FOR STORING  LAST 10 DIGITAL VALUES
int avg[8];                   //STORING THE SUM OF LAST 10 DIGITAL VALUES WHEN SENSOR GOES OUT OF TRACK
int sum=0;                    //STORING SUM OF INSTANTANEOUS DIGITAL VALUES


//VARIABLES FOR SERVOCONTROL
int servospeed;           //ANGLE OF THE SERVO
int servo_set=95; //110   //SET POINT OF THE SERVO
int setpoint=35;          //SET POINT FOR ERROR CALCULATION
int buzzer=12;


                                                          //FUNCTION DEFINITION OF setup() function

      void setup() 
      {
            Serial.println("Inside setup()");
            
            ecobot.attach(2);
            Serial.begin(115200); 
            pinMode(junctionPulse,INPUT);
            pinMode(A13,INPUT);
            pinMode(buzzer,OUTPUT);
            
            Serial.println("Exiting setup()");
      }

                                                              //END OF setup()



                                                              //FUNCTION DEFINITION of sensor_read() FOR READING ANALOG VALUES
void sensor_read()
{
  Serial.println("Inside sensor_read()");
  
 sensor[5]=analogRead(A13);
 pv=((float)sensor[5]/921)*70;
 Serial.print( "pv=");
 Serial.print( pv);
 Serial.print("\t");
 // if(pv>=74 && ppev<=35 && ppev>=0)
  //{
   // pv=0;
  //}

  Serial.println("Exiting sensor_read()");
}

                                                                //END OF sensor_read()



                                                                  //FUNCTION DEFINITION FOR PID
void pid_calc()
{
  Serial.println("Inside pid_calc()");
  
  int postion;
  //int setpoint;
  int last_proportional=0;
  postion=pv;
  Serial.print("postion=");
  Serial.print(postion);
  proportional=postion-setpoint;
  Serial.print("    proportional=");
  Serial.print(proportional);
  if(integral>=450&&proportional>0)
  {
    integral=450;
  }
  
  else if(integral<=-450&&proportional<0)
  {
    integral=-450;
  }
  else
  {
  integral=integral+ proportional;
   }
    Serial.print("  integral=");
  Serial.print(integral);
  derivative=proportional-last_proportional;
  Serial.print("    derivative=");
  Serial.print(derivative);
  last_proportional=proportional;
  Serial.print("    last_proportional=");
  Serial.print(last_proportional);
  error_val=int(proportional*kp+integral*ki+derivative*kd);
  Serial.print("    error_val1=");
  Serial.println(error_val);

    Serial.println("Exiting pid_calc()");
}

                                                                            //END OF pid_calc()


                                         // FUNCTION DEFINITION OF calc_turn() for calculating servospeed according to calculated error_val
void calc_turn()
{
  if(error_val<-maxturn_left)
  {
    error_val= -maxturn_left;
        Serial.print("  error_value2=");
    Serial.print(error_val);
    Serial.print("\t");
  }
  if(error_val>maxturn_right)
  {
    error_val= +maxturn_right;
    Serial.print(" error_value23=");
    Serial.println(error_val);
        Serial.print("\t");
  }
   if (error_val<0)
   {
    servospeed=servo_set+error_val;
    Serial.print("servo_speed=");
    Serial.println(servospeed);
    
   }
   
   if (error_val>=0)
   {
    servospeed=servo_set+error_val;
    Serial.print("servo_speed=");
    Serial.println(servospeed);
    
   }
}

                                                                      //END OF calc_turn()


                                                                  //FUNCTION DEFINITION of servo_speed() FOR SERVO_WRITE
void servo_speed()
{
  ecobot.write(servospeed); 
}

                                                                  //END OF servo_speed()




                                                                  //FUNCTION DEFINITION FOR JUNCTION
void junction()
{
      while(pv<50)
      {
        
      sensor_read();
      /*for(int i=0;i<8;i++){
      Serial.print(dsensor[i]);
      Serial.print(" ");}
      Serial.println();*/
      servospeed=servo_set+maxturn_right1;
      servo_speed();
      Serial.print("servo speed=");
      Serial.println(servospeed);
      Serial.println("RIGHT");
     
        Serial.println("Junction");
    // Increment junction count by 1 after the junction
    // You can do whatever you want with the junction count
    junctionCount++;
      }

      while(pv>65)
      {
        
      sensor_read();
      
      /*for(int i=0;i<8;i++){
      Serial.print(dsensor[i]);
      Serial.print(" ");}
      Serial.println();*/
      servospeed=servo_set+maxturn_right1;
      servo_speed();
      Serial.print("servo speed=");
      Serial.println(servospeed);
      Serial.println("RIGHT");
              Serial.println("Junction");
    // Increment junction count by 1 after the junction
    // You can do whatever you want with the junction count
    junctionCount++;
      }
  }
  
                                                                                  //END OF  junction()

  

void read_digital();     //declaration of the function



                                                                              //loop() function
void loop()
{
      sensor_read();            //reading analog values
      read_digital();           //reading digital values


//1ST CONDITION FOR SENSOR GOING OUT OF TRACK ON COMPLETELY MONOCOLORED BACKGROUND DUE TO OVERSHOOT
//CALCULATES THE AVERAGE VALUES( ACTUALLY ONLY THE SUM) OF LAST 10 DIGITAL VALUES

          if(pv>73 && sum==0)      
        {
      
           //calculating the avg values 
              for(int i=0;i<8;i++)
              {
                  avg[i]=0;
      
                   for(int j=0;j<10;j++)
                      avg[i]+=prevValue[j][i];
              }
      
                            Serial.println("----------------------Avg value-------------------------");
                               for(int i=0;i<8;i++)
                                 { 
                                    Serial.print(avg[i]);
                                    Serial.print(" ");
                                 }

                            Serial.println("--------------------------------------------------------");


    
                            Serial.println();


                        //calculating sum of 3 values of exterme left and extreme right from the array of avg values
                            int lsum=avg[0]+avg[1]+avg[2];//+avg[3];
                            int rsum=avg[6]+avg[7]+avg[5];//+avg[5];


                       //printing the sum of left 3 avg values and sum of right 3 avg values
                               Serial.print("LSUM = ");
                               Serial.print(lsum);
                               Serial.print("      RSUM = ");
                               Serial.println(rsum);

                        
                        
                        if(lsum>rsum)
                       {
                                //condition  for remaining in the left position if sensor values ={0} till bot senses the line
                                while(pv<10||pv>65)
                                {                      
                                 
                                    servospeed=servo_set-maxturn_left;
                                    Serial.println("LEFT due to avg");
                                    servo_speed();
                                    sensor_read();
                                }
                       }



                     
                       else if(rsum>lsum)
                       {
                                 //condition  for remaining in the right position if sensor values ={0} till bot senses the line
                               while(pv<10||pv>65)                     
                               {
                                    servospeed=servo_set+maxturn_right;
                                    Serial.println("RIGHT due to avg");
                                 
                                    servo_speed();
                                    sensor_read();
                                    
                               }

                      }
    
     }
  
//END OF 1ST CONDITION


//2ND CONDITION FOR JUNCTION
/* else  if(digitalRead(junctionPulse)&&pv<37)
   {
    digitalWrite(buzzer,LOW);
    junction();
    Serial.println(servospeed);
    servo_speed();
    //delay(1250);
    
   }*/
//END OF 2ND CONDITION
  


//3RD CONDITION FOR PID CALCULATION
     else
   {
          Serial.print("pv in else condition= ");
          Serial.print(pv);
          Serial.print(" ");
          Serial.print("sum =");
          Serial.println(sum );
          digitalWrite(buzzer,LOW);
         pid_calc();
         calc_turn();
   
         ppev=pv;
   }
//END OF 3RD CONDITION

   servo_speed();

   
}

                                                                            //END OF loop()


void read_digital()                                    //DIGITAL READ
{ 
  //READING INSTANTANEOUS DIGITAL VALUES FROM SENSOR AND STORING IN dsensor[] array 
  //AND, CALCULATING SUM OF ALL THE INSTANTANEOUS VALUES AND STORING IN VARIABLE sum
  sum=0;
  dsensor[0]=digitalRead(51);//51
  sum+=dsensor[0];
 dsensor[1]=digitalRead(37);//37
  sum+=dsensor[1];
 dsensor[2]=digitalRead(49);//49
 sum+=dsensor[2];
 dsensor[3]=digitalRead(39);//39
  sum+=dsensor[3];
 dsensor[4]=digitalRead(47);//47
  sum+=dsensor[4];
 dsensor[5]=digitalRead(41);//41
  sum+=dsensor[5];
 dsensor[6]=digitalRead(45);//45
  sum+=dsensor[6];
 dsensor[7]=digitalRead(43);//43
 sum+=dsensor[7];


              //PRINTING SUM OF ALL THE INSTANTANEOUS DIGITAL VALUES FROM VARIABLE sum
              //AND PRINTING ALL THE INSTANTANEOUS DIGITAL VALUES FROM dsensor[] array
               Serial.print("sum in digital read=");
               Serial.println(sum );
  
                for(int i=0;i<8;i++)
                {
                      Serial.print(dsensor[i]);
                      Serial.print(" ");
                }
                  Serial.println();
 
  
  //STORING INSTANTANEOUS  DIGITAL SENSOR VALUES FROM dsensor[] array
  for(int i=0;i<8;i++)
 {   
  prevValue[x][i]=dsensor[i];
 
 }

      //INCREMENTING THE ROW NUMBER OF THE prevValue[] array FOR STORING NEXT SENSOR VALUES
       x++; 



      // UPDATING x=0 TO STORE FROM ROW 1 AGAIN AFTER COMPLETION OF 10 VALUES 
      //AND PRINTING LAST 10 VALUES
    if(x==10) 
    {
      Serial.println();
      Serial.println("----------------------Prev 10 Values-------------------");
      x=0;
      for(int i=0;i<10;i++)
      {for(int j=0;j<8;j++)
      {
        Serial.print(prevValue[i][j]);
        Serial.print(" ");
      }
        Serial.println("-------------------------------------------------");
      }
    }
    
}
