/**
 *LIBRARY FILE FOR MULTIPLE DISPLAY BOARDS USED IN ROBOCON-2017
 *
 * Display boards used for:
 *      -speed
 *      -node
 *      -yaw 
 *      -pitch
 *      -roll
 *
 *  AUTHORS:
 *      ATUL RAM
 *      TAPAS MAHANTA
 *      CHINMAYA MOHAPATRA
 *      SUMIT KUMAR PRADHAN
 *
 */
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"

int print_string(char s[]);
int print_alphabet(char z);
int digit_begin();
void pin_init(char disp,int pins[]);

char s[50];

//  seg[]={a,b,c,d,e,f,g,dp}; 
int seg[]={22,23,24,25,26,27,28,20};
int seg_delay=500;

int ds[]={8,9,10,11};
int dn[]={4,5,6,7};
int dy[]={0,0,0,0};
int dp[]={30,31,32,33};
int dr[]={0,0,0,0};


int digit_begin()
{
  for(int i=0;i<8;i++)
  {
	pinMode(seg[i],OUTPUT);
  }
	
  for(int i=0;i<4;i++)
  {
	pinMode(ds[i],OUTPUT);
	pinMode(dn[i],OUTPUT);
	pinMode(dp[i],OUTPUT);
	pinMode(dr[i],OUTPUT);
	pinMode(dy[i],OUTPUT);
  }

}

void allOff()
{
	for(int i=0;i<8;i++)
	{
		digitalWrite(seg[i],HIGH);
	}
}

//set the delay time between two display units
void segment_delay(int del)
{
	seg_delay=del;
}

void pin_init(char disp,int pins[])
{
	switch(disp)
	{
		case 's'://motor speed
			for(int i=0;i<4;i++)
			{
				ds[i]=pins[i];
			}
			break;

		case 'n'://node
			for(int i=0;i<4;i++)
			{
				dn[i]=pins[i];
			}
			break;

		case 'p'://pitch
			for(int i=0;i<4;i++)
			{
				dp[i]=pins[i];
			}
			break;

		case 'r'://roll
			for(int i=0;i<4;i++)
			{
				dr[i]=pins[i];
			}
			break;

		case 'y'://yaw
			for(int i=0;i<4;i++)
			{
				dy[i]=pins[i];
			}
			break;

		case 'd'://segment pins
			for(int i=0;i<8;i++)
			{
				seg[i]=pins[i];
			}
			break;

	}	

}




int print_string(char disp,char s[])
{
	
	switch(disp)
	{
		case 's'://motor speed
			for(int i=0;i<4;i++)
			{
				for(int j=0;j<4;j++)
				{
					if(i==j)
					{
					  digitalWrite(ds[j] , HIGH );
					}	
					else
					{
					  digitalWrite(ds[j] , LOW );
					}
				}
				print_alphabet(s[i]);
				delayMicroseconds(seg_delay);
 				allOff();
                                digitalWrite(ds[i] , LOW);
			}
			break;
                        
		case 'n'://node
			for(int i=0;i<4;i++)
			{
				for(int j=0;j<4;j++)
				{
					if(i==j)
					{
					  digitalWrite(dn[j] , HIGH );
					}	
					else
					{
					  digitalWrite(dn[j] , LOW );
					}
					
				}
				print_alphabet(s[i]);
				delayMicroseconds(seg_delay);
                                allOff();
                                digitalWrite(dn[i] , LOW);
			}
			break;

		case 'p'://pitch
			for(int i=0;i<4;i++)
			{
				for(int j=0;j<4;j++)
				{
					if(i==j)
					{
					  digitalWrite(dp[j] , HIGH );
					}	
					else
					{
					  digitalWrite(dp[j] , LOW );
					}
				}
				print_alphabet(s[i]);
				delayMicroseconds(seg_delay);
                                allOff();
                                digitalWrite(dp[i] , LOW);
			}
			break;

		case 'r'://roll
			for(int i=0;i<4;i++)
			{
				for(int j=0;j<4;j++)
				{
					if(i==j)
					{
					  digitalWrite(dr[j] , HIGH );
					}	
					else
					{
					  digitalWrite(dr[j] , LOW );
					}
					
				}
				print_alphabet(s[i]);
				delayMicroseconds(seg_delay);
                                allOff();
                                digitalWrite(dr[i] , LOW);				
			}
			break;

		case 'y'://yaw
			for(int i=0;i<4;i++)
			{
				for(int j=0;j<4;j++)
				{
					if(i==j)
					{
					  digitalWrite(dy[j] , HIGH );
					}	
					else
					{
					  digitalWrite(dy[j] , LOW );
					}
					
				}
				print_alphabet(s[i]);
      				delayMicroseconds(seg_delay);
	                        allOff();
                                digitalWrite(dy[i] , LOW);				
			}
			break;
             
	
}
	
	
}



int print_alphabet(char z)
	{
		String cl[]={"abc0efg","abcdefg","a00def0","abcdef0"};  //segment patterns for a,b,c,d
		String num[]={"abcdef0","0bc0000", "ab0de0g", "abcd00g","0bc00fg", "a0cd0fg", "a0cdefg", "abc0000", "abcdefg", "abcd0fg"}; //segment patterns for 0 to 9
			if(z>=65 && z<=90)
			{
				allOff();
				for(int i=0;i<7;i++)
				{
					if(cl[z-65].charAt(i)=='0')
					digitalWrite(seg[i],HIGH);
					else
					digitalWrite(seg[i],LOW);
					
				}
			}

			else if(z>=48 && z<=57)
			{
				allOff();
				for(int i=0;i<7;i++)
				{
					if(num[z-48].charAt(i)=='0')
					digitalWrite(seg[i],HIGH);
					else
					digitalWrite(seg[i],LOW);
					
				}
			}
			
			else
			{
				allOff();
				
			}

}
			
	

