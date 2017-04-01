The problem statement of robocon 2017 was:
https://www.roboconindia.com/rule%20book%202017.pdf

Mechanical design:
1. <Solidworks file>
2.Motion
    -We had used an 'X' design for the base to implement 4 wheeled omnidirectional drive called the 'Killough drive'.
    -Wheels used were 100mm double aluminium omni wheels.
    
3.Throwing Platform

    The design of the throwing platform was the most critical part in the robot manufacturing process as this decided the     initial parameters governing the aerodynamics of the frisbee during its flight.
    
    Reference papers for study of aerodynamics of frisbee flight:
    i.  http://scripts.mit.edu/~womens-ult/frisbee_physics.pdf
    ii. http://digitalcommons.csbsju.edu/cgi/viewcontent.cgi?article=1067&context=honors_theses
    
    From the study of the aerodynamics of the frisbee flight it was concluded that the parameters upon which the frisbee's flight depended were:
    -> initial launch angle
    -> initial velocity
    -> initial angular velocity
      
    a.Linear actuators
      -> These were responsible for controlling the first parameter viz initial launch angle of the frisbee
      -> Two linear actuators were used to control the pitch and roll angle of the platform
            -One was fitted at the middle of front side edge of the platform to control the pitch angle of the platform
            -Other was fitted at the middle of left side edge of the platform to control the roll angle of the platform  
      
    b.Driving Wheel
      -> This wheel was used to impart linear velocity and angular velocity to the frisbee

    c.Control wheel
      -> According to the requirement we need to control the initial velocity and angular velocity of the frisbee independently but the driving wheel alone couldn't control these 2 parameters independently.
      -> Therfore, the control wheel was added.
      -> This wheel was placed such that the path of the frisbee was in between the driving wheel and the controlling wheel.
      -> Such a placement enabled us to impart two impulsive tangential forces to the frisbee at two ends of its diameter and conrol linear momentum and angular momentum of the frisbee independently.
    
    d.Stack holder
      ->Used for holding the frisbees.
      ->Had a capacity of holding 20 frisbees at a time.
      
    e.Pneumatic cylinder
        -> This was used for launching the frisbees.
        -> It pushed each frisbee out of the stack to get into the contact of the driving wheel.
      
      
-----------------------------------------------------------------------------------------------------------------------------
Electronics:

I. Motherboard:
      -Two motherboards were used, one for master controller and the other as a slave controller 


        Microcontroller boards:
        ----------------------
            We had designed a robot with two microcontroller boards: 
            1.master board (Arduino Mega 2560)
                -Takes input from sensors and devices:
                    ->LSA08 sensor for line following
                    ->9-DOF Razor IMU for yaw control
                    ->PS4 for taking control commands from the controller

                -Controls the motors:
                    -> 4 Motion control motors
                    -> Motor coupled to:
                            -Driving wheel
                            -Control wheel
                -Actuation of pneumatic for pushing a frisbee out of the stack
                -Control linear actuator to vary the pitch and roll angle of the throwing platform

            2.slave board (Arduino Mega 2560)
                -Takes input from an IMU mounted on the throwing platform to calculate the pitch anf roll angle
                -Used for control of display boards for displaying five parameters
                    ->Speed of driving wheel
                    ->Speed of control wheel
                    ->Node number
                    ->Pitch angle of throwing platform 
                    ->Roll angle of throwing platform

            The communication between the two boards was done through I2C using the 'Wire' library of Arduino.

II. Motor driver circuits
    
      7 Motor driver circuits from cytron were used.
      -Specs:
            -> 30 amp DC motor driver x 5
                4 were used for motion control of the base
                1 was used for controlling the speed of the motor coupled to driving wheel
            -> 13 amp DC motor driver x 2
                1 was used for controlling the speed of the motor coupled to controlling wheel
                1 ws used to control the linear actuator
