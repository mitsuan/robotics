

/****************************************************************************************************
Team Id:          eYRC-CC#2755
Author List:      Sumit Kumar Pradhan, Ronak Bansal.
Filename:         main_code.c
Theme:            Cross_A_Crater
Functions:        void port_init(),
					void timer5_init(),
					void velocity(unsigned char, unsigned char),
					void motors_delay(),
					void lcd_port_config (void),
					void motion_pin_config (void), 
					void adc_pin_config (void),
					void motion_pin_config (void),
					void buzzer_pin_config (void),
					void servo1_pin_config (void),
					void servo2_pin_config (void),
					void servo3_pin_config (void),
					void left_encoder_pin_config (void),
					void right_encoder_pin_config (void),
					void timer1_init(void),
					void adc_init(),
					unsigned char ADC_Conversion(unsigned char Ch), 
					void print_sensor(char row, char coloumn,unsigned char channel),
					void velocity (unsigned char left_motor, unsigned char right_motor),
					void motion_set (unsigned char Direction),
					void forward (void) ,
					void left_1(void),
					void right_1(void),
					void stop (void),
					void left_position_encoder_interrupt_init (void),
					void right_position_encoder_interrupt_init (void),
					void uart0_init(void),
					void init_devices (void),
					int filter(int val,int thresh),
					int get_sum(),
					void get_sensor_val(),
					void move_till_node(),
					void reach_base_station(),
					void get_boulders(int bn),
					void reach_bridge(),
					void drop_boulder(int crater_pos),
					void reach_start(int crater_pos),
					void buzzer_on (void),
					void buzzer_off (void),
					void follow_line(),
					void turn_left(int o),
					void turn_right(int o),
					void turn_back(),
					ISR(INT5_vect),
					ISR(INT4_vect),
					void angle_rotate(unsigned int Degrees),
					void linear_distance_mm(unsigned int DistanceInMM),
					void forward_mm(unsigned int DistanceInMM),
					void left_degrees(unsigned int Degrees),
					void right_degrees(unsigned int Degrees),
					void servo_1(unsigned char degrees),
					void servo_2(unsigned char degrees),
					void servo_3(unsigned char degrees),
					SIGNAL(SIG_USART0_RECV),
					void servo_set(),
					void hand_down(),
					void hand_up(),
					void grab(),
					void release(),
					int main()

Global Variables:  THRESHOLD, tem, Left_white_line, Center_white_line, Right_white_line, left, center, right, mean_speed, right_motor_speed, 
					left_motor_speed, kp, ki, kd, error,derivative,integration,last_error,pid, ShaftCountRight,ShaftCountLeft,data,
					data_arr[100],data_count,data_flag,node_nmbr, craters_1,craters_2, craters_pos_1[4],craters_pos_2[3][2],obstacles, obs_pos[4][2],
					 boulders,bz[4],bz_to_pick[4],bridge=1,boulder_dir[4][7]
*********************************************************************************************************/



#define F_CPU 14745600

#define		THRESHOLD	13    //sensor adc threshold

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"



void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char tem = 0;
//****************line follwoing *************
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
int left=0,center=0,right=0;  //after threholding


//***************motor parameters************
#define 	mean_speed  160

unsigned int right_motor_speed=mean_speed;
unsigned int left_motor_speed=mean_speed;


//************pid parameters****************
const float kp = 20.0;
const float ki = 0.8;
const float kd = 0.05;
int error=0,derivative=0,integration=0,last_error=0,pid=0;

//********************position encoder parameters*******************
unsigned long int ShaftCountRight; //right shaft position count
unsigned long int ShaftCountLeft; //left shaft position count

//*******ZIGBEE COMM VARIABLES******************************
unsigned char data;			 // receiving each character in the ISR
unsigned char data_arr[100]; //an array taking input data from xbee
unsigned char data_count=0;
unsigned char data_flag=1;		//to check and wait till all data has been received
//***********variables*********************************
int node_nmbr=0;

unsigned char craters_1; //no. of craters in bridge 1
unsigned char craters_2; //no. of craters in bridge 2
unsigned char craters_pos_1[4]={0,0,0,0};
unsigned char craters_pos_2[3][2]={{0,0},{0,0},{0,0}};
unsigned char obstacles; //no. of obstacles
unsigned char obs_pos[4][2]={{0,0},{0,0},{0,0},{0,0}};
unsigned char boulders;//no. of boulders
unsigned char bz[4]={-1,-1,-1,-1};
unsigned char bz_to_pick[4]={0,0,0,0};
unsigned char bridge=1; //bridge to be chosen
unsigned short int boulder_dir[4][7]={
											{6,'f','r','b','l','f','s'},
											{4,'r','b','l','s',0,0},
											{4,'l','b','r','s',0,0},
											{6,'f','l','b','r','f','s'}
											};




//******************FUNCTION DEFINITIONS***************************************
//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to initialize Buzzer 
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}


//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE = DDRE & 0xEF; //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}
//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE = DDRE & 0xDF; //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}



//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();	
	buzzer_pin_config ();
	servo1_pin_config();
	servo2_pin_config();
	servo3_pin_config();
	right_encoder_pin_config();
	left_encoder_pin_config();
}

//*************************************************************************

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}
//TIMER1 initialization in 10 bit fast PWM mode
//prescale:256
// WGM: 7) PWM 10bit fast, TOP=0x03FF
// actual value: 52.25Hz
void timer1_init(void)
{
TCCR1B = 0x00; //stop
TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
TCNT1L = 0x01; //Counter low value to which OCR1xH value is to be compared with
OCR1AH = 0x03; //Output compare Register high value for servo 1
OCR1AL = 0xFF; //Output Compare Register low Value For servo 1
OCR1BH = 0x03; //Output compare Register high value for servo 2
OCR1BL = 0xFF; //Output Compare Register low Value For servo 2
OCR1CH = 0x03; //Output compare Register high value for servo 3
OCR1CL = 0xFF; //Output Compare Register low Value For servo 3
ICR1H = 0x03;
ICR1L = 0xFF;
TCCR1A = 0xAB;
//COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0
//For Overriding normal port functionality to OCRnA outputs. WGM11=1, WGM10=1. Along With //WGM12 in
//TCCR1B for Selecting FAST PWM Mode TCCR1C = 0x00;
TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

/*******************************************************************************************
Function Name: adc_init()
Input:         
Output:        
Logic:         
Example Call: adc_init()
********************************************************************************************/

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/********************************************************************************************
Function Name: ADC_Conversion(unsigned char Ch)
Input:         
Output:        
Logic:         
Example Call: ADC_Conversion(unsigned char Ch)
********************************************************************************************/

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}


/********************************************************************************************
Function Name: print_sensor(char row, char coloumn,unsigned char channel)
Input:         
Output:        
Logic:         
Example Call: print_sensor(char row, char coloumn,unsigned char channel)
********************************************************************************************/


//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/********************************************************************************************
Function Name: velocity (unsigned char left_motor, unsigned char right_motor)
Input:         
Output:        
Logic:         
Example Call: velocity (unsigned char left_motor, unsigned char right_motor)
********************************************************************************************/

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	if(right_motor>35)
	right_motor-=35;
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}


/********************************************************************************************
Function Name: motion_set (unsigned char Direction) 
Input:         
Output:        
Logic:         
Example Call: motion_set (unsigned char Direction) 
********************************************************************************************/


//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

/********************************************************************************************
Function Name: forward (void) 
Input:         
Output:        
Logic:         
Example Call: forward (void) 
********************************************************************************************/

void forward (void) 
{
  motion_set (0x06);
}

/********************************************************************************************
Function Name: left_1(void)
Input:         
Output:        
Logic:         
Example Call: left_1(void)
********************************************************************************************/

void left_1(void)
{
  motion_set (0x05);
}

/********************************************************************************************
Function Name: right_1(void)
Input:         
Output:        
Logic:         
Example Call: right_1(void)
********************************************************************************************/

void right_1(void)
{
  motion_set (0x0A);
}

/********************************************************************************************
Function Name: stop (void)
Input:         
Output:        
Logic:         
Example Call: stop (void)
********************************************************************************************/
void stop (void)
{
  motion_set (0x00);
}

//************encoders interrupt init******************************************

/********************************************************************************************
Function Name: left_position_encoder_interrupt_init (void)
Input:         
Output:        
Logic:         
Example Call: left_position_encoder_interrupt_init (void)
********************************************************************************************/


//Functions for configuring external interrupts for position encoders
void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei(); // Enables the global interrupt
}
/********************************************************************************************
Function Name: right_position_encoder_interrupt_init (void)
Input:         
Output:        
Logic:         
Example Call: right_position_encoder_interrupt_init (void)
********************************************************************************************/
void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei(); // Enables the global interrupt
}

//************zigbee uart port init************************************
//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled


/********************************************************************************************
Function Name: uart0_init(void)
Input:         
Output:        
Logic:         
Example Call: uart0_init(void)
********************************************************************************************/

void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
// UBRR0L = 0x47; //11059200 Hz
 UBRR0L = 0x5F; // 14745600 Hzset baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

/********************************************************************************************
Function Name: init_devices (void)
Input:         
Output:        
Logic:         
Example Call: init_devices (void)
********************************************************************************************/
void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	timer1_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	uart0_init();
	sei();   //Enables the global interrupts
}

//***********************************************************************************************
//*********************************OUR FUNCTIONS***************************************************
/********************************************************************************************
Function Name: filter(int val,int thresh)
Input:         
Output:        
Logic:         
Example Call: filter(int val,int thresh)
********************************************************************************************/

//Fiter sends 1 when higher than the given threshhold else 0
int filter(int val,int thresh)
{
	return (val>thresh);
}
/********************************************************************************************
Function Name: get_sum()
Input:         
Output:        
Logic:         
Example Call: get_sum()
********************************************************************************************/
int get_sum()
{
	get_sensor_val();
	return (left+center+right);
}
/********************************************************************************************
Function Name: get_sensor_val()
Input:         
Output:        
Logic:         
Example Call: get_sensor_val()
********************************************************************************************/
void get_sensor_val()
{
		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor



		//print_sensor(1,1,3);	//Prints value of White Line Sensor1
		//print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		//print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		left = filter(Left_white_line,THRESHOLD);
		center = filter(Center_white_line,THRESHOLD);
		right = filter(Right_white_line,THRESHOLD);

		##lcd_print(1,1,left,1);
		#lcd_print(1,5,center,1);
		#lcd_print(1,9,right,1);
}
/********************************************************************************************
Function Name: move_till_node()
Input:         
Output:        
Logic:         
Example Call: move_till_node()
********************************************************************************************/

void move_till_node()
{

	while(!(get_sum()>=2))
		{	
			forward();
	  		velocity(mean_speed,mean_speed);
		}
}

/********************************************************************************************
Function Name: reach_base_station()
Input:         
Output:        
Logic:         
Example Call: reach_base_station()
********************************************************************************************/
void reach_base_station()
{
	int nodeNmbr=0;
	if(bridge==1)
	{
		forward_mm(190);
		left_degrees(100);
		forward_mm(200);
		stop();
		buzzer_on();
		_delay_ms(5000);
		buzzer_off();
	}
	else
	{
		forward_mm(200);
		right_degrees(100);
		forward_mm(200);
		stop();
		buzzer_on();
		_delay_ms(5000);
		buzzer_off();
	}

	while(1);

		
}

/********************************************************************************************
Function Name: get_boulders(int bn)
Input:         
Output:        
Logic:         
Example Call: get_boulders(int bn)
********************************************************************************************/
void get_boulders(int bn)
{
	stop();
	//buzzer_on();
	//_delay_ms(1000);
	//buzzer_off();
	forward();
	velocity(mean_speed,mean_speed);
	int nodeNmbr=0;
	
		/*when reaches respective last node it will incrmnt nodeNmbr ,operate the cmd 
		and then this condition will get checked
		*/

	while(nodeNmbr<boulder_dir[bn-1][0]) 
	{
		get_sensor_val();

		if(get_sum()>=2)
		{
			nodeNmbr++;
			
			switch(boulder_dir[bn-1][nodeNmbr])
			{
				case 'f':forward_mm(30);		break;	//move away from the node
				case 'l':forward_mm(50);left_degrees(85); get_sum();while(center<1) {get_sum();turn_left(1);	}break;
				case 'r':forward_mm(50);right_degrees(85);get_sum(); while(center<1){get_sum(); turn_right(1);}	break;
				case 'b':
							stop();
							hand_down(); 
							forward_mm(20);
						 	_delay_ms(100); 
							grab();
						 hand_up();
						 forward_mm(20);
						 right_degrees(160);
						 get_sum();	
						  while(center<1)
						  {get_sum();
						   turn_right(1);} break;
				case 's':forward_mm(20);stop();

			}

			buzzer_off();

		}
		else
		{
			follow_line();
		}

		
		
	}
	
	
}


/********************************************************************************************
Function Name: reach_bridge()
Input:         
Output:        
Logic:         
Example Call: reach_bridge()
********************************************************************************************/
void reach_bridge()
{


	int nodeNmbr=0;
	if(bridge==1)
	{
		forward_mm(60);
		right_degrees(95);
	}
	else
	{
		forward_mm(60);
		left_degrees(90);
	}
	while(get_sum()<=1)
	{
		follow_line();
	}
	if(bridge==1)
	{
		forward_mm(60);
		left_degrees(95);
	}
	else
	{
		forward_mm(60);
		right_degrees(90);
	}

/*	while(get_sum()<=1)
	{
		follow_line();
	}*/

	stop(); 



}
/********************************************************************************************
Function Name: drop_boulder(int crater_pos)
Input:         
Output:        
Logic:         
Example Call: drop_boulder(int crater_pos)
********************************************************************************************/
void drop_boulder(int crater_pos)
{
	forward_mm((crater_pos-1)*100);
	hand_down();
	_delay_ms(500);
	release();
	_delay_ms(100);
	grab();
	_delay_ms(100);
	hand_up();
	
	if(crater_pos==1)
	{
	right_degrees(160);
	get_sum();
	while(center==0)
	{
		get_sum();
		if(bridge==1)
		turn_right(1);
		else
		turn_left(1);
	}

	}
	else
	{
		right_degrees(180);
		forward_mm((crater_pos-1)*100);
	}

}
/********************************************************************************************
Function Name: reach_start(int crater_pos)
Input:         
Output:        
Logic:         
Example Call: reach_start(int crater_pos)
********************************************************************************************/
void reach_start(int crater_pos)
{
	if(crater_pos!=1)
	{
		while(get_sum()==0)
		{
			forward();
			velocity(mean_speed,mean_speed);
		}
		forward_mm(30);
		while(get_sum()<2)
		{
		follow_line();
		}
		forward_mm(60);
		if(bridge==1)
		right_degrees(90);
		else
		left_degrees(90);
	}

	while(get_sum()<2)
		follow_line();
		forward_mm(60);
		if(bridge==1)
		left_degrees(90);
		else
		right_degrees(90);

		stop();
	
}



//******************************buzzer***********************************************************
/********************************************************************************************
Function Name: buzzer_on (void)
Input:         
Output:        
Logic:         
Example Call:  buzzer_on (void)
********************************************************************************************/
void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

/********************************************************************************************
Function Name: buzzer_off (void)
Input:         
Output:        
Logic:         
Example Call:  buzzer_off (void)
********************************************************************************************/

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

//*****************************MOTION*************************************************************
/********************************************************************************************
Function Name: follow_line()
Input:         
Output:        
Logic:         
Example Call:  follow_line()
********************************************************************************************/
void follow_line()
{
	if((get_sum())==1)
		{
		error = (left*1 + center*2 + right*3) - 2;
		derivative = error - last_error;
		if(integration>100) integration = 100;
		last_error = error;
		}
		
			
			integration += error;
		 pid = kp*error + kd*derivative + ki*integration;
		
		int ls = 0,rs = 0;
		if(error<0)
		{
			rs = abs(pid);
			forward();
	  	velocity(mean_speed-rs,mean_speed+rs);

		}
		else if(error>0)
		{
		 	ls = pid;
			forward();
	  		velocity(mean_speed+ls,mean_speed-ls);
		 }
		else
		{
			forward();
	  		velocity(mean_speed+ls,mean_speed +rs);
		}


}

/********************************************************************************************
Function Name: turn_left(int o)
Input:         
Output:        
Logic:         
Example Call:  turn_left(int o)
********************************************************************************************/
void turn_left(int o)
{
	if(o==0)
	{
	while(get_sum()!=0 )
	{
		get_sensor_val();
		//forward();
		//velocity(0,150);
		left_1();
		velocity(120,120);	
	}
	while(center!=1 )
	{
		get_sensor_val();
		get_sum();
		//forward();
		//velocity(0,150);
		left_1();
		velocity(120,120);	
	}

	forward();
	velocity(0,0);
	}

	else if(o==1)
	{

		left_1();
		velocity(130,130);
		
	}

} 
/********************************************************************************************
Function Name: turn_right(int o)
Input:         
Output:        
Logic:         
Example Call:  turn_right(int o)
********************************************************************************************/
void turn_right(int o)
{
	if(o==0)
	{
	while(get_sum()!=0 )
	{
		get_sensor_val();
		//forward();
		//velocity(150,0);
		right_1();
		velocity(120,120);	
	}
	while(center!=1 )
	{
		get_sensor_val();
		get_sum();
		//forward();
		//velocity(150,0);
		right_1();
		velocity(120,120);	
	}
	forward();
	velocity(0,0);
	}

	else if(o==1)
	{

		right_1();
		velocity(130,130);
	}

}

/********************************************************************************************
Function Name: turn_back()
Input:         
Output:        
Logic:         
Example Call:  turn_back()
*/

void turn_back()
{
	while(get_sum()!=0 )
	{
		get_sensor_val();
		right_1();
		velocity(150,150);	
	}
	while(center!=1 )
	{
		get_sensor_val();
		get_sum();
		right_1();
		velocity(150,150);	
	}
	forward();
	velocity(0,0); 
	
}

//********************motion control using position control*********************************
/********************************************************************************************
Function Name:ISR(INT5_vect)
Input:         
Output:        
Logic:         
Example Call: ISR(INT5_vect)
********************************************************************************************/
//ISR for right position encoder
ISR(INT5_vect)
{
ShaftCountRight++; //increment right shaft position count
}
/*
Function Name:ISR(INT4_vect)
Input:         
Output:        
Logic:         
Example Call: ISR(INT4_vect)
*/
//ISR for left position encoder
ISR(INT4_vect)
{
ShaftCountLeft++; //increment left shaft position count
}
/********************************************************************************************
Function Name: angle_rotate(unsigned int Degrees)
Input:         
Output:        
Logic:         
Example Call:  angle_rotate(unsigned int Degrees)
********************************************************************************************/
//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

/********************************************************************************************
Function Name: linear_distance_mm(unsigned int DistanceInMM)
Input:         
Output:        
Logic:         
Example Call:  linear_distance_mm(unsigned int DistanceInMM)
********************************************************************************************/
//Function used for moving robot forward by specified distance
void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;
	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

/********************************************************************************************
Function Name: forward_mm(unsigned int DistanceInMM)
Input:         
Output:        
Logic:         
Example Call:  forward_mm(unsigned int DistanceInMM)
********************************************************************************************/
void forward_mm(unsigned int DistanceInMM)
{
	forward();
	velocity(mean_speed,mean_speed);
	linear_distance_mm(DistanceInMM);
}
/********************************************************************************************
Function Name: left_degrees(unsigned int Degrees)
Input:         
Output:        
Logic:         
Example Call:  left_degrees(unsigned int Degrees)
********************************************************************************************/
void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	turn_left(1); //Turn left
	angle_rotate(Degrees);
}
/********************************************************************************************
Function Name:  right_degrees(unsigned int Degrees)
Input:         
Output:        
Logic:         
Example Call:  right_degrees(unsigned int Degrees) 
********************************************************************************************/
void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	turn_right(1); //Turn left
	angle_rotate(Degrees);
}

//****************SERVO CONTROL******************************************************
/********************************************************************************************
Function Name:  servo_1(unsigned char degrees)
Input:         
Output:        
Logic:         
Example Call:  servo_1(unsigned char degrees)  
********************************************************************************************/
//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)
{
float PositionPanServo = 0;
PositionPanServo = ((float)degrees / 1.86) + 35.0;
OCR1AH = 0x00;
OCR1AL = (unsigned char) PositionPanServo;
}


/********************************************************************************************
Function Name:  servo_2(unsigned char degrees)
Input:         
Output:        
Logic:         
Example Call:  servo_2(unsigned char degrees)  
********************************************************************************************/

//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
float PositionTiltServo = 0;
PositionTiltServo = ((float)degrees / 1.86) + 35.0;
OCR1BH = 0x00;
OCR1BL = (unsigned char) PositionTiltServo;
}

/********************************************************************************************
Function Name:  servo_3(unsigned char degrees)
Input:         
Output:        
Logic:         
Example Call: servo_3(unsigned char degrees)  
********************************************************************************************/
//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
float PositionServo = 0;
PositionServo = ((float)degrees / 1.86) + 35.0;
OCR1CH = 0x00;
OCR1CL = (unsigned char) PositionServo;
}

//*******************************************************************************************



//**********************ZIGBEE COMM FUNCTION*************************************************
/********************************************************************************************
Function Name: SIGNAL(SIG_USART0_RECV) 
Input:         
Output:        
Logic:         
Example Call: SIGNAL(SIG_USART0_RECV)  
********************************************************************************************/
SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable 

	//UDR0 = data; 				//echo data back to PC

	if(data=='#')
	{
		data_flag=0;
	}
	else
	{
		data_arr[data_count]=data;
		data_count++;	
	}
	
}

//********************************SERVO hand MOTION**************************************
/********************************************************************************************
Function Name: servo_set()
Input:         
Output:        
Logic:         
Example Call: servo_set() 
********************************************************************************************/
void servo_set()
{
	release();
	_delay_ms(1000);
	hand_up();
}
 
 /********************************************************************************************
Function Name: hand_down()
Input:         
Output:        
Logic:         
Example Call: hand_down() 
********************************************************************************************/
 
void hand_down()
{
	_delay_ms(2000);
		for(int i=130;i<=170;i++)
		{
			_delay_ms(15);
			servo_1(i);

		}


}

/********************************************************************************************
Function Name: hand_up()
Input:         
Output:        
Logic:         
Example Call: hand_up() 
********************************************************************************************/

void hand_up()
{
		_delay_ms(1000);
		for(int i=170;i>=130;i--)
		{
			_delay_ms(15);
			servo_1(i);

		}
}

/********************************************************************************************
Function Name: grab()
Input:         
Output:        
Logic:         
Example Call: grab() 
********************************************************************************************/

void grab()
{

	_delay_ms(1000);
		for(int i=30;i<=85;i++)
		{
			_delay_ms(15);
			servo_3(i);

		}
}



/********************************************************************************************
Function Name: release()
Input:         
Output:        
Logic:         
Example Call: release() 
********************************************************************************************/

void release()
{

		_delay_ms(1000);
		for(int i=85;i>=30;i--)
		{
			_delay_ms(15);
			//servo_1(i);
			servo_3(i);

		}
}


//*******************************************************************************************



/********************************************************************************************
Function Name: main()
Input:         
Output:        
Logic:         
Example Call: main()  
********************************************************************************************/


int main()
   {
	init_devices();
	lcd_set_4bit();
	lcd_init();
	servo_set();
	
	
		while(data_flag==1);
		
		int i=0;
		craters_1=data_arr[i]-48;
		for(int j=0;j<craters_1;j++)
		{
			i++;
			craters_pos_1[j]=data_arr[i]-48;
		}
		
		i++;
		craters_2=data_arr[i]-48;
		for(int j=0;j<craters_2;j++)
		{
			i++;
			craters_pos_2[j][0]=data_arr[i]-48;
			i++;
			craters_pos_2[j][1]=data_arr[i]-48;
		}
		
		i++;
		obstacles=data_arr[i]-48;
		for(int j=0;j<obstacles;j++)
		{
			i++;
			obs_pos[j][0]=data_arr[i]-48;
			i++;
			obs_pos[j][1]=data_arr[i]-48;
		}
		
		i++;
		boulders=data_arr[i]-48;
		for(int j=0;j<boulders;j++)
		{
			i++;
			bz[j]=data_arr[i]-48;
		}
		
		i++;
		bridge=data_arr[i]-48;
		for(int j=0;j<((bridge==1)?craters_1:craters_2);j++)
		{
			i++;
			bz_to_pick[j]=data_arr[i]-48;
		}
		
		
		
		
	forward_mm(30);	
	get_sensor_val();
	
			

	follow_line();
	
	int boulders_to_drop;

	if(bridge==1)
	boulders_to_drop=craters_1;
	else
	boulders_to_drop=craters_2;
	
	
		for( i=1;i<=boulders_to_drop;i++)
		{
			get_boulders(bz_to_pick[i-1]);
			reach_bridge();
			
			int crat_pos;
			if(bridge==1)
			crat_pos=craters_pos_1[i-1];
			else
			crat_pos=craters_pos_2[i-1];
			
			drop_boulder(crat_pos);
			if(i!=boulders_to_drop)
			reach_start(crat_pos);
			else
			reach_base_station();
		
		}	 

		
	
}
