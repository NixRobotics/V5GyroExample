V5GyroExample

EXAMPLE CODE USING INERTIAL SENSOR TO TURN ROBOT 360 DEGREES

This example shows several concepts needed to be able to use the inertial sensor effectively. All the steps here will
be needed to successfully integrate the inertial sensor into your robot code. Below are some of the key concepts.

1. Calibrating the inertial sensor

Upon each power-up the inertial sesnor must be calibrated while the robot is stationary and on the surface it will be used on.
This is done in the pre_autonomous() function. The calibration takes around 2 seconds. During this time the robot must not be
moved or the calibration will fail and erractic motion of the robot will be a result. Calibration is a way for the VEX OS to
automatially determine the orientation of the sensor. It does this by using the 3-axis accelerometer to determine which way is
down (gravity). Once this is known the API can then combine the readings from the 3-axis gyroscope (gyro) to provide a heading
that is oriented correctly to the field.
 
Because the robot must not be moved during calibration, there is a global variable ROBOT_INITIALIZED that is used to stop any
commands within user_control() or autonomous() from exeruting. This will introduce a small delay before the robot can move.
This delay is hidden when the robot is connected to a field controller (as in a competition) or if the program is started
using one of the Competition or Timed Run options on the controller. If you just do Program->Run on the controller then
you will see the delay directly.

2. PID control of turns (see TURN_CONSTANT and TIME_FOR_FULL_TURN notes below)

The big difference between using an inertial sensor and not using one is that in the simple case of turning the robot without
a sensor you just turn the motors for a fixed number of revolutions, whereas with the inertial sensor you are constantly
comparing the desired heading with the actual heading and adjusting the motor speeds to correct any difference. The most common
way to do this is with a PID controller and VEX has this built into the SmartDrive class and you will interact with this when
using one of the turn commands such as turn_for(). For expediency we will use the SmartDrive class here, although it is not
recommended when using geared drivetrains as it causes the motors to fight each other. The purpose of this exampe is not to
go into PID control so I will just point out the issue and move on. As autonomus routines are usually limited to 60secs max,
any additional heating due to this effect will be minimal.

Without going into too many details of PID control there are 3 concepts that need to be understood regardless:
 - Overshoot / Undershoot / Oscillation: Most likely when you first try to turn the robot it will either turn too sluggishly
   or oscilate around the desired heading. This is due to the gain used in the PID controller. The SmartDrive class lets you
   "tune" the proportional gain (typically denoted as Kp) using the set_turn_constant() function. The default value is 1.0 and
   in most cases will need adjusting either up or down a bit. If the robot oscilates around the desired heading then this value
   is too high, if the robot turns very slowly or never reaches the desired heading then this value is too low. You want to find
   a value where the robot stops cleanly with just a very small wobble at the end. See the NOTE in the code for more details.

 - Settle Time and Error: The PID controller will never be perfect and there will always be a small error between the desired heading
   and the actual heading. This is normal. The controller will try to reduce this error to zero but will never get there. Instead it
   will "settle" at some small error value. The time taken to reach this value is known as the settle time and the resulting error is
   known as the steady-state error. Because of this, turns can and will have a small variation in terms of how long they take to complete
   and the final heading. If the turn_constant is set correctly the settle time and error should be small, but its good to
   plan to have a few seconds of margin for your autnomous routine and to read back the heading after a turn to see where you
   are actually pointing

 - Timeouts: If the robot is blocked during a turn command the PID controller will keep trying to turn the robot until it reaches
   the desired heading. We use a timeout to stop the robot after a certain time. This is done using the set_timeout() function. The timeout
   should be set to a value that is longer than the time it takes to complete the turn under normal conditions. This way if the robot is
   blocked it will stop after the timeout rather than continuing to try and turn forever. See NOTE in the code for more details.

3. Heading Eror (see GYRO_SCALE notes below)

Each inertial sensor has a certain amount of built-in error, typically in the range of +/- 1% to 2%. For example most sensors
I've used will turn the robot in the range of 355 to 365 degrees when in reality we want to turn 360 degrees. So for each sensor
we need to determine what the error is experimentally and then compensate for this in the code. The best way to do this is on
some kind of test rig where you can measure the actual turn accurately. Ideally this is done by rotating the sensor both left
and right and at different speeds taking the average. For expediency I implement code below that turns the robot 10 times after
which you can measure the error and divide by 10 to get the average error per turn. This value is then used to adjust the turn
commands or the reading of the inertial sensor. See NOTE in the code for more details.

4. Understanding the difference between inertial.angle(), inertial.rotation() and inertial.heading()

VEX API provides 3 ways to read the inertial sensor
 - inertial.heading(): Provides a value in the range 0 to 360 degress similar to a compass heading (not recommended as explained below)
 - inertial.rotation(): Provides a continuous value that is either positive or negative (recommended)
 - inertial.angle(): Provides a value in the range -180 to +180 degrees (again not recommended as explained below)
Only inertial.rotation() is recommended for use as it provides a continuous value that can be positive or negative allowing
the GYRO_SCALE compensation to work correctly. For example applying the GYRO_SCALE compensation to inertial.heading() will not work
correclty as turning a robot left by 10 degrees results in a heading of 350 degrees. Applying a GYRO_SCALE compensation to this value
will not work as intended. It is better to to use inertial.rotation() and then provide your own function to translate this to a
heading if needed.

5. Long-term drift

Inertial sensors are subject to long-term drift due to the nature of the sensors used. This means that over time the heading
will drift away from the actual heading. The amount of drift will depend on the quality of
