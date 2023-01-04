# Servo Controller
This controller runs on a Raspberry Pi Pico microcontroller.
The Pico may optionally be soldered to a main PCB that facilitates
interconnects with up to 16 RC servos, 2 power supplies and a
computer.

This controller is designed to use the "Precision Servo System"
that runs on a Linux computer.   This system includes a GUI app
for servo calibration.  This app allows for the robot's as-built
joint angles to be measured and does a least squares fit of these
measurements to find a function that transforms joint angles in
radians in the joint's frame to microseconds of pulse width for
the servo that drives that joint.  
This calibration corrects for differences between each servomotor,
and inaccuracies in the installation of the servo.
The users can also set soft limits
for joint angles.  This data is stored in a calibrations file.

In addition to setting the position the user can set a rate
in radians per second.  The servo's position will be updated
so as to make it move at the specified rate.  
It will continue to move untill it either hits a soft limit or
the rate and postion is updated.    
The default rate is zero.

This servo controller can store the data from the calibration file
and apply it in real time.  So the Linux computer sends the desired
angles in radians in the joints frame to the Pico that in turns
generates the PWM pulses for up to 16 servos.

The Precision Servo System can also use a PCA9685 breakout board
to drive a set of up to 16 RC servos.  This controller replaces
the PCA9685 board and does several things better:
1) The PWM pulses are much more accurate.
1) An angular rate can be specified so the robot joint can move 
independently of position updates from the Linux computer
2) The PCA9685 uses I2C while the Pico has
a few other options to connect with the Linux PC


### Future Work

I may move this software to a Pico-W or ESP32.
Both of these have WiFi radios that might allow the servos to
be controlled over a wireless link.  This would allow the Linux
computer to be removed from the robot, allowing for a smaller
and lower priced robot. 
(Implementing the above angular rate feature should reduce
WiFi bandwidth and importantly allow for smooth motion if
WiFi packets are dropped.)

I may implement inverse kinematics in this controller so that
the Linux PC only has to send the desired (x, y, z) locations
and optional velocities of the end points.  
My guess is that the ESP32 would be used 
because it has much faster floating point hardware. 

### Author
Chris Albertson
albertson.chris@gmail.com

