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
int i=0,j=-1;
int x=4;
int orient=0;   //+x:0, -x:1 
//............................

int a[6][6]=
{{1,2,3,4,5,6},
  {2,4,3,5,1,6},
  {5,4,6,2,3,1},
  {1,5,2,3,4,6},
  {6,2,3,4,1,5},
  {4,3,1,6,2,5}
  };


//VOID SETUP
void setup()
{
  Serial.begin(115200);
  
  motor_setup();
  sensor_setup();

  delay(2000);
}

//.........................


//loop function

void loop()
{
//  forward();

   if(junction())         //NODE DETECT
      {
//         jn++; //NODE VAL INCREMENT
      
          
//          Serial.print("jn: ");
//          Serial.println(jn);
//          
          stop1();
          delay(300);

//          turn_back();
//          stop1();
//          delay(5000);

          switch(orient)
          {

            case 0: j++; break;
            case 1: i--; break;
            case 2: j--; break;
            case 3: i++; break;
            
          } 

          if(orient==3)
          {

            if(a[i][j]==x)
          {
            change_row();
            x=j+1;
          }

          else{
            
            //if(i==2&& j==2)
            int turn =search_x();
            

            if(turn==0)
           {
            turn_left();
            orient=0;
           }
           else if(turn==1)
           {
            turn_right();
            orient=2;
           }
           
          }
            
          }

          else
          {
            if(a[i][j]==x)
          {
            change_row();
            x=j+1;
          }
          
          }
          
       
  }
     
    
    move_forward();

//    displayOnBoard();

}


