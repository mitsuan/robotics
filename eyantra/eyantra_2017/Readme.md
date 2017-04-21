# Problem Statement:
The theme was Cross A Crater and was based on the Mars mission of ISRO. The complete problem statement is given the pdf-'Problem_statement_eyantra_2017'.

## Overview of solution 

   According to the problem statement our robot had gone to a location (the boulder region)away from the base station. While returning it encountered two bridges with cavities. One of the bridge also had some obstacles. The robot had to fill the cavities present in the bridge using some boulders present in the boulder region. And, then cross the bridge to reach the base station. 
  
  *** The complete arena was monitored using an **overhead camera** connected to a pc, which acts as a satellite. 
  
  *** The robot had an Atmega 2560 which was programmed using embedded-C to:
     + sense the black line for traversing different parts of the arena.
     + follow the line using a digital PID 
     + display relevant data on 16x2 LCD 
     + communicate with the pc using serial communication over xbee
     + control the actuators 
         - servo for hand mechanism
         - Motors for motion control
     + buzz the buzzer at the end.

*** A **python** code running on the pc processes the image of the arena using the overhead camera and sends the following information to the robot using an **xbee**:
   + Numbers printed on the top of the boulders 
   + Postions of the boulders
   + Postions of craters 
   + position of obstacles
  
  
*** The robot accordingly travels to the boulders using **line following**, picks them up and fills the cavities. After filling up the cavities the robot reaches the base station, completing the problem statement.
    
