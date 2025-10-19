# EXAMPLE CODE USING INERTIAL SENSOR TO TURN ROBOT 360 DEGREES
#
# See README at: https://github.com/NixRobotics/V5GyroExample/blob/main/README.md
#

# Library imports
from vex import *

brain = Brain()

# declare motors
l1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
l2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(l1, l2)
r1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
r2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(r1, r2)

inertial = Inertial(Ports.PORT5)

# NOTE: GYRO_SCALE is used to compensate for each inertial sensor's built in error. This will be different for each sensor
#  and must be determined experimentally before use.
# 
#  If GYRO_SCALE_UNKNOWN is set to True then the robot will turn 10 times to make the error more visible. Measure the error
#  and divide by 10 to get the amount the robot actually turns when instructed to turn 360 degrees. This value can then be
#  entered for ACTUAL_ROBOT_FULL_TURN, e.g. if the robot is pointing at 45degress to the right from where it started it says
#  that the robot will turn by 360 + 45/10 = 364.5 degrees when instructed to turn 360 degrees. So we set ACTUAL_ROBOT_FULL_TURN
#  to 364.5 and then set GYRO_SCALE_UNKNOWN to False to indicate we know what the error is.
#
#  The two values GYRO_SCALE_FOR_TURNS and GYRO_SCALE_FOR_READOUT are then used in the code as follows:
#   - GYRO_SCALE_FOR_TURNS is used when we tell the drivetrain to turn. In the example above we want the robot to turn less
#   - GURO_SCALE_FOR_READOUT is used when we read the inertial sensor to display the current heading. In the example above
#     the inertial sensor is returning a value that is too small so we need to multiply by a factor > 1 to get the correct value
#
# IMPORTANT: If the robot does not turn cleanly meaning TURN_CONSTANT needs adjusting, do that first (see NOTE below). If this is
#  the case temporarily set GYRO_SCALE_UNKOWN to False and ACTUAL_ROBOT_FULL_TURN to 360.0 and come back to this later

GYRO_SCALE_UNKOWN = True
ACTUAL_ROBOT_FULL_TURN = 360.0 # e.g. if robot actually turns 365 degrees for a 360 rotation enter 365 here
GYRO_SCALE_FOR_TURNS = 360.0 / ACTUAL_ROBOT_FULL_TURN
GYRO_SCALE_FOR_READOUT = ACTUAL_ROBOT_FULL_TURN / 360.0

track_width = 15 * 25.4 # will not be used with inertial sensor
wheel_base = 15 * 25.4 # will not be used with inertial sensor
wheel_travel = 319.19 # still used for driving forward / backwards
ext_gear_ratio = 1 # still needed for driving forward / backwards

# NOTE: Use SmartDrive here instead
drivetrain = SmartDrive(left_drive, right_drive, inertial, wheel_travel, track_width, wheel_base, MM, ext_gear_ratio)

# NOTE: Use ROBOT_INITIALIZED to allow movement. Calibration time is hidden when connected to field, but we need to prevent robot
#  from moving if we just do Program->Run on the controller
ROBOT_INITIALIZED = False

def pre_autonomous():
    # actions to do when the program starts
    global ROBOT_INITIALIZED

    # IMPORTANT: wait for sensors to initialize fully. Always include a small delay when using any sensors. This includes the 3-wire ports
    wait(0.1, SECONDS)

    # calibrate inertial and wait for completion - takes around 2 seconds
    # IMPORTANT: Robot must be stationary on a flat surface while this runs. Do not touch robot during calibration
    if inertial.installed():
        inertial.calibrate()
        while inertial.is_calibrating():
            wait(50, MSEC)

    ROBOT_INITIALIZED = True

def autonomous():
    # wait for initialization to complete
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    # place automonous code here

# NOTE: TURN_CONSTANT is used by the drivetrain to control how fast the robot responds to the difference in actual heading vs
#  desired heading. If robot swings back and forth this value is too high, if robot turns very slowly or never reaches the heading
#  this is too low. You want to find a value where the robot stops cleanly with just a very small wobble at the end. This value will
#  be dependent on robot weight and speed at which you turn, so you may need to adjust occasionally
TURN_CONSTANT = 1.0

# NOTE: TIME_FOR_FULL_TURN is how fast the robot can complete one full revolution. It is used to calculate a timeout value to stop
#  the robot in case turn command does not complete, e.g. if blocked against something
TIME_FOR_FULL_TURN = 2.0 # seconds. Set to 2 seconds by default - adjust accordingly based on your robot and turn_velocity() setting

def full_turn(number_of_turns = 1):
    # set the turn_constant - see NOTE above
    drivetrain.set_turn_constant(TURN_CONSTANT)
    drivetrain.set_drive_velocity(50, PERCENT)
    drivetrain.set_turn_velocity(50, PERCENT)
    # set the timeout - see NOTE above. We add 1 second on top of the turn to provide some margin. If you change the speed at which
    # the robot turns you need to provide your own calculation here
    drivetrain.set_timeout(TIME_FOR_FULL_TURN * number_of_turns + 1, SECONDS)
    # NOTE: here we use the inverse of gyro_scale
    drivetrain.turn_for(RIGHT, number_of_turns * 360.0 * GYRO_SCALE_FOR_TURNS, DEGREES)
    # TODO: can add out own timeout detection here
    drivetrain.stop(BRAKE)

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("Waiting for robot to initialize fully ... ")
    brain.screen.next_row()

    # wait for initialization to complete
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    brain.screen.print("done")

    # Good idea to check if inertial sensor is present before using it as unexpected motion can occur
    if not inertial.installed():
        brain.screen.print("NO INERTAIL SENSOR")
        while True:
            wait(20, MSEC)

    # NOTE: DO NOT USE inertial.heading() - the scale calculation will not work. Use either inertial.rotation() or inertial.angle()
    #  Here we multiply the value returned from the inertial sensor by the scaling factor. In our example above of robot turning 365 degrees
    #  (when we wanted 360 degrees) it means the inertial sensor is returning a too small value, so we multiply by 365/360 in this case. This is
    #  the opposite of when we tell the drivetrain what we want it to do. Because it turns too far we want it to turn less so in the turn_far()
    #  command we tell it to turn less by a factor of 360/365 (the inverse)
    start_angle = inertial.rotation(DEGREES) * GYRO_SCALE_FOR_READOUT
    brain.screen.print("Starting Heading: ", start_angle)

    if GYRO_SCALE_UNKOWN:
        full_turn(10)
    else:
        full_turn(1)

    end_angle = inertial.rotation(DEGREES) * GYRO_SCALE_FOR_READOUT
    brain.screen.next_row()
    brain.screen.print("End Heading: ", end_angle)

    # place driver control in this while loop
    while True:
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()
