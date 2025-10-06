# Library imports
from vex import *

brain = Brain()

l1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
l2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
left_drive = MotorGroup(l1, l2)
r1 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
f2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)
right_drive = MotorGroup(r1, r2)

inertial = Inertial(Ports.PORT5)

# NOTE: Set GYRO_SCALE to the amount the robot actually turns when instructed to turn 360 degrees, e.g. if it turns by 365 degrees
#  enter that here. If set to None the code below will turn the robot by 10 revolutions so you can see the error more clearly.
#  E.g. if the robot finishes at an angle of 45 degrees after being instructed to turn 10*360 degrees, then GYRO_SCALE should be
#  set to (10 * 360 + 45) / 10 = 364.5
# IMPORTANT: If the robot does not turn cleanly meaning TURN_CONSTANT needs adjusting, do that first (see NOTE below). If this is
#  the case temporarily set GYRO_SCALE to 360.0 and come back to this later
GYRO_SCALE = None

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
    drivetrain.turn_for(RIGHT, number_of_turns * 360 * (360 / gyro_scale)), DEGREES)
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
    start_angle = inertial.rotation(DEGREES) * (gyro_scale / 360)
    brain.screen.print("Starting Heading: " + inertial.rotation())

    if GYRO_SCALE is None:
        full_turn(10)
    else:
        full_turn(1)

    end_angle = inertial.rotation(DEGREES) * (gyro_scale / 360)
    brain.screen.next_row()
    brain.screen.print("End Heading: " + inertial.rotation())

    # place driver control in this while loop
    while True:
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()
