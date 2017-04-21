# ROBOCON 2016 - Eco Robot Design and programing

 ## Problem Statement
      http://www.roboconindia.com/rulebook%202016.pdf
      
 ## Description of Robot:
    According to the problem statement we had to design two robots - Hybrid robot and Eco robot. Here a brief explanation of the Eco robot has been provided.
     
   ### Eco Robot 
     This robot was to be designed with no driving power. The only power it had was used to run the microcontroller and steer the robot's motion. The energy for driving this robot was transferred through the Hybrid robot using a 'Clean Energy' (wind energy, electromagnetic energy, gravitational energy). Our design had a flap at the back of the Eco robot used to push the robot using the wind energy delivered by the Hybrid robot using an EDF.
    
    
 ## Control of the robot
    The robot was an autonomous one which uses the wind energy delivered by the Hybrid robot to drive itself and uses its own power source to control the steering using a servo motor to follow the white line in the arena.
    
 ### Microcontroller
  We had used Arduino Mega for controlling the Eco robot.
   
 ### Sensors
 ->LSA08 (Cytron):
           This line following sensor was used to sense the white line and send the data to the microcontroller to control the servo motor in order to steer the robot's motion.
           
 ### Actuators
 -> Metal geared servo :
      It was connected to the frame on which the front wheels were mounted. 
           
 ### Programming
   
   The robot was programmed using a PID control loop. The position of white line was sensed using the lsa08 sensor and the error was determined. This error was fed to the the digital PID algorithm whose output was used to control the servo to keep the sensor centered with the white line.
      
    Another algorithm was used to store the last 10 values of the sensor in order to determine the position of the robot if the sensor gets off the white line. 
      
    The track on the arena had a zig-zag path (river) to be crossed. Many a times the robot got off the track at the turns and wasn't able to get back to the track. The above algorithm was useful in this case to determine the direction in which the robot should turn to get back to the track.
      
    There was a junction after crossing the first part (hill) of the arena. We had to take a right turn for the left side arena and left turn for the right side arena, at this junction. We had used a toggle switch on the board which was toggled according to the arena assigned during gameplay. The lsa08 sensor had a feature called the 'Junction Pulse' which give a active high signal when a particular number of sensors (can be set in the sensor module) are high, which helps to detect a junction without explicitly writing a code for the same.
      
   
 
