/*
   this code is written to trace line for curve path for sharp turn u can change the as per ur needs
   for the bot with 2 wheels and line sensor LSA08
   it uses only the analog pin of the sensor and the junction
   sensor gives the value from 0 to 900 and above 980 when no line is detected
   line sensor can detect color diffenence between any two color
   sensor can be calibrated to trace either white or the black line
   kp ki kd value used in the code for 200 rpm motor for any change in configuration u can set new values for the same
   limit 2000 is used at random to limit the integral value u can choose of ur choice

*/


//*************************************
#define GRIDS 6
int x=-1,y=0;
int orient=0;   //+x:0, -x:1 
//............................

int a=1,b=5;


//VOID SETUP
void setup()
{
  Serial.begin(115200);
  
  motor_setup();
  sensor_setup();

  delay(2000);

  solve();
}

//.........................


void solve()
{
  for(int i=a;i<=b;i++)
  {
      int x1=i%GRIDS;
      int y1=phi(x1)%GRIDS;

      if((x-x1)>0)
      {
          while((orient-2)!=0)
          {
            if((orient-2)<0)
            {
                turn_left();
                if(orient==3)
                  orient=0;
                else 
                orient++;
            }
            else
            {
                turn_right();
                if(orient==0)
                orient=3;
                else
                orient--;
              
            }
            
          }
            
      }
      else if((x-x1)<0)
      {
         while((orient-0)!=0)
          {
            if((orient-0)<0)
            {
                turn_left();
                if(orient==3)
                  orient=0;
                else 
                orient++;
            }
            else
            {
                turn_right();
                if(orient==0)
                orient=3;
                else
                orient--;
              
            }
            
          }
        
      }
          
      while((x-x1)!=0)
      {
          move_forward();
          
          if(junction())
          {
           
            stop1();
            update_coord();
            
          }
        
      }
//------------------------------move towards y-----------------------------
      if((y-y1)>0)
      {
          while((orient-3)!=0)
          {
            if((orient-3)<0)
            {
                turn_left();
                if(orient==3)
                  orient=0;
                else 
                orient++;
            }
            else
            {
                turn_right();
                if(orient==0)
                orient=3;
                else
                orient--;
              
            }
            
          }
            
      }
      else if((y-y1)<0)
      {
         while((orient-1)!=0)
          {
            if((orient-1)<0)
            {
                turn_left();
                if(orient==3)
                  orient=0;
                else 
                orient++;
            }
            else
            {
                turn_right();
                if(orient==0)
                orient=3;
                else
                orient--;
              
            }
            
          }
        
      }
          
      while((y-y1)!=0)
      {
          
          move_forward();
          if(junction())
          {
           
            stop1();
            update_coord();
            
          }
        
      }

      stop1();
      delay(5000);
  }
  
}


//loop function
void loop()
{
stop1();
}

