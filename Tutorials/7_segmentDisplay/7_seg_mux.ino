

/**
 * Library for 4 DIGIT MULTIPLEXED 7-SEGMENT DISPLAY
 * Author: Sumit Kumar Pradhan
 */


//Pins for segments 
//int segment[]={a,b,c,d,e,f,g,dec};
int segment[]={22,24,26,28,30,32,34,36};

//Pins for decimal place
//int  dn[]={d0,d1,d2,d3};
int  dn[]={40,41,42,43};


//input data for the 4 digits
int data[]={0,0,0,0};
int data_ind=3;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  
  for(int i=0;i<8;i++)
  {
    pinMode(segment[i],OUTPUT);
  }
  
  for(int i=0;i<4;i++)
  {
    pinMode(dn[i],OUTPUT);
  }
  
  allOff();
}

void pattern(int n)
{
//  String sym[]={0,1,2,3,4,5,6,7,8,9,dec};
  String sym[]={"abcdef","bc","abged", "abgcd", "fgbc", "afgcd","afgcde", "abc", "abcdefg", "abcdfg", "h"};

  for(int i=0;i<sym[n].length();i++)
  {
    digitalWrite(segment[ sym[n].charAt(i)-97 ],LOW);
  }
  
}

void allOff()
{
  for(int i=0;i<8;i++)
  {
    digitalWrite(segment[i],HIGH);
  }
}


void d3Out(int n)
{
  for(int i=0;i<4;i++)
  {
    if(i==3)
    digitalWrite(dn[i],HIGH);
    else
    digitalWrite(dn[i],LOW);
  }
  
  allOff();
  
  pattern(n);
  
  
}

void d2Out(int n)
{
  for(int i=0;i<4;i++)
  {
    if(i==2)
    digitalWrite(dn[i],HIGH);
    else
    digitalWrite(dn[i],LOW);
  }
  
  allOff();
  
  pattern(n);
  
}


void d1Out(int n)
{
 for(int i=0;i<4;i++)
  {
    if(i==1)
    digitalWrite(dn[i],HIGH);
    else
    digitalWrite(dn[i],LOW);
  }
  
  allOff();
  
  pattern(n);
  
  
}

void d0Out(int n)
{
  for(int i=0;i<4;i++)
  {
    if(i==0)
    digitalWrite(dn[i],HIGH);
    else
    digitalWrite(dn[i],LOW);
  }
  
  allOff();
  
  pattern(n);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  while(Serial.available())
  {
    char serialChar=Serial.read();
    data[data_ind]=serialChar-48;
    data_ind--;
    if(data_ind==-1)
    data_ind=3;
  }

  
  for(int i=3;i>=0;i--)
  {
    Serial.print(data[i]); 
    Serial.print(" ");
  }
  Serial.println();
  
  d3Out(data[3]);
  delay(5);
  d2Out(data[2]);
  delay(5);
  d1Out(data[1]);
  delay(5);
  d0Out(data[0]);
  delay(3);

}
