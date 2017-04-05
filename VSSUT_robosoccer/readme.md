# Intro
    This project has been started to enable proper research in the field of:
      -Motion planning and odometry of omnidrive robots
      -Image Processing and Computer Vision
      -Artificial Intelligence
      -Electronics circuit design
      -Mechanical design
      
      And, prepare multiple autonomous soccer playing robots for the Robocup Small Sized League.

------------------------------------------------------------------------------------------------------------------------

# Mechanical
    
  ## Motion
        The base of the robot has been prepared for a kiwi drive holonomic motion i.e a three wheeled robot.
        
  ## Wheels:
            The wheels used are 100mm plastic omni wheels.
        
  ## Motors:
            Three Faulhaber Coreless 17W Encoder Motor 120RPM are used to drive the robot.

------------------------------------------------------------------------------------------------------------------------

# Electronics
  
  ## Microcontroller:
          Arduino UNO is being used as the microcontroller board to control the robot.
      
  ## Communication
          Xbee is used for wireless communication between the robot and PC.
      
  ## Motor Drivers
          3 x Cytron 13 amp DC motor drivers are used to control the three motors coupled to the wheels of the robot.
          
  ## Camera
          Intex IT-306WC is used as an overhead camera to take the complete view of the field of motion of the robot

------------------------------------------------------------------------------------------------------------------------

# Programming

    The complete robot control is done in the following steps:
      -Capture video from camera through PC
      -Use image processing and algorithms to extract required information from video frames
      -Send the data to MCU on robot through Xbee
      -Command the robot to move and orient in the desired way
      
      The complete processing can be categorised into two:
        -Image Processing
        -Robot control
        
 ## Image processing
          
   ### Platform
              We are using OpenCV libraries in Python.
              
   ### Algorithm
              <to be explained>
              
 ## Robot control
           The MCU on the robot receives the data packet about the motion from the PC through the xbee.
           
           The data consists of three parameters:
           -x : The required speed in x direction
           -y : The required speed in x direction
           -w : The required angular speed along z-axis
           
           Using the above three parameters and the inverse kinematics matrix, the speed (PWM) and direction of each motor is calculated and the motors are conrolled accordingly.
             
