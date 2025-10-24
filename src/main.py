# EXAMPLE CODE USING INERTIAL SENSOR TO TURN ROBOT 360 DEGREES
#
# See README at: https://github.com/NixRobotics/V5GyroExample/blob/main/README.md
#
# To use:
# 1. Edit DRIVE DECLARATIONS section to match your motor ports and directions and inertial sensor port
# 2. Adjust drive parameters such as track_width, wheel_base, wheel_travel, ext_gear_ratio
# 3. (optional) If using DriveTrain mode adjust DRIVE_TRAIN_TURN_SCALE to make sure robot turns roughly 360 degrees when instructed
# 4. Determine TURN_CONSTANT for your robot (see notes)
# 5. Determine ACTUAL_ROBOT_FULL_TURN for your robot (see notes)
# 6. See how long it takes your robot to do one full turn and set TIME_FOR_FULL_TURN accordingly
# 7. Once all parameters are set, run demo2_turn_to_headings() to see how well robot can turn to specific headings

# Library imports
from vex import *

brain = Brain()

# DEVICE DECLARATIONS

# declare motors
l1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
l2 = Motor(Ports.PORT3, GearSetting.RATIO_18_1, True)
left_drive = MotorGroup(l1, l2)
r1 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
r2 = Motor(Ports.PORT4, GearSetting.RATIO_18_1, False)
right_drive = MotorGroup(r1, r2)

inertial = Inertial(Ports.PORT5)

# DRIVE TRAIN AND SMART DRIVE SETUP

# drive parameters - adjust to match your robot
track_width = 10 * 25.4 # (mm) will not be used with inertial sensor
wheel_base = 9.25 * 25.4 # (mm) will not be used with inertial sensor
wheel_travel = 319.19 # (mm) still used for driving forward / backwards
ext_gear_ratio = 1 # still needed for driving forward / backwards

# NOTE: DriveTrain is used to allow basic driving capabilities, uses only motor encoders for turning and driving
# Turns are calculated based on track_width and wheel_base, but these will require tweaking to get reasonable results
# When using traction wheels particularly towards the center of each side of the robot, the turns will be much tighther
# than the calculation used by VEX. Mostly this is due to the wheel_base being close to the diameter of the wheel (in the case of
# a single traction wheel on each side). With enough trial and error you can find a scale value that may give reasonable results.
# However, turn accuracy is greatly affected by turn speed and weight of robot (e.g. when loading up with game elements),
# so using SmartDrive with inertial sensor is preferred
DRIVE_TRAIN_TURN_SCALE = 0.525
drive_train = DriveTrain(left_drive, right_drive, wheel_travel, track_width, wheel_base, MM, ext_gear_ratio)

# NOTE: SmartDrive is used to allow the use of the inertial sensor for turning.
# Typically you would not declare both DriveTrain and SmartDrive in the same program. This is just used for demo purposes
smart_drive = SmartDrive(left_drive, right_drive, inertial, wheel_travel, track_width, wheel_base, MM, ext_gear_ratio)

# NOTE: To use SmartDrive we also need to define a few parameters to handle the inertial sensor's built in error and the nature of
# PID control used by SmartDrive. Specifically we define here:
# - GYRO_SCALE
# - TURN_CONSTANT
# - TIME_FOR_FULL_TURN
# see discussion below for details on each parameter

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
#   - GYRO_SCALE_FOR_TURNS is used when we tell the robot to turn. In the example above we want the robot to turn less
#   - GURO_SCALE_FOR_READOUT is used when we read the inertial sensor to display the current heading. In the example above
#     the inertial sensor is returning a value that is too small so we need to multiply by a factor > 1 to get the correct value
#
# IMPORTANT: If the robot does not turn cleanly meaning TURN_CONSTANT needs adjusting, do that first (see NOTE below). If this is
#  the case temporarily set GYRO_SCALE_UNKNOWN to False and ACTUAL_ROBOT_FULL_TURN to 360.0 and come back to this later

GYRO_SCALE_UNKNOWN = True # set to True if you do not know the actual full turn value yet - will cause robot to do 10 full turns for calibration
ACTUAL_ROBOT_FULL_TURN = 360.0 # (362 for CODE BOT) e.g. if robot actually turns 365 degrees for a 360 rotation enter 365 here
GYRO_SCALE_FOR_TURNS = 360.0 / ACTUAL_ROBOT_FULL_TURN
GYRO_SCALE_FOR_READOUT = ACTUAL_ROBOT_FULL_TURN / 360.0

# NOTE: TURN_CONSTANT is used by the SmartDrive class to control how fast the robot responds to the difference in actual heading vs
#  desired heading. If robot swings back and forth this value is too high, if robot turns very slowly or never reaches the heading
#  this is too low. You want to find a value where the robot stops cleanly with just a very small wobble at the end. This value will
#  be dependent on robot weight and speed at which you turn, so you may need to adjust occasionally
TURN_CONSTANT = 1.0 # starting point - adjust as needed based on robot response (CODE BOT uses 0.7)

# NOTE: TIME_FOR_FULL_TURN is how fast the robot can complete one full revolution. It is used to calculate a timeout value to stop
#  the robot in case turn command does not complete, e.g. if blocked against something
TIME_FOR_FULL_TURN = 2.0 # seconds. Set to 2 seconds by default - adjust accordingly based on your robot and turn_velocity() setting

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

# AUTONOMOUS HELPER FUNCTIONS

class GyroHelper:
    # returns the inertial sensor's corrected direction as continuous ROTATION [-inf, +inf]
    # this is the only version of the direction routines that queries the inertial sensor directly
    @staticmethod
    def gyro_rotation():
        return inertial.rotation(DEGREES) * GYRO_SCALE_FOR_READOUT

    # performs modulus operation on the input so that output is in range [0, 360) degrees
    # note that this will lose history on total full revolutions, but useful if we want current HEADING of the robot
    # @param rotation as ROTATION value (either corrected or not - function is agnostic)
    @staticmethod
    def to_heading(rotation):
        return rotation % 360.0

    # performs modulus operation and offset on the input so that output is in range (-180, + 180] degrees
    # note that this will lose history on total full revolutions, but useful if we want current ANGLE of the robot
    # @param rotation as ROTATION value (either corrected or not - function is agnostic)
    @staticmethod
    def to_angle(rotation):
        angle = rotation % 360.0
        if (angle > 180.0): angle -= 360.0
        return angle

    # returns the inertial sensor's corrected direction as HEADING [0, 360) degrees
    @staticmethod
    def gyro_heading():
        return GyroHelper.to_heading(GyroHelper.gyro_rotation())

    # returns the inertial sensor's corrected direction as ANGLE (-180, +180] degrees
    @staticmethod
    def gyro_angle():
        return GyroHelper.to_angle(GyroHelper.gyro_rotation())

    # Calculate a "raw" turn angle to get the robot facing towards a "real" HEADING based on current gyro reading
    #
    # This will return the smallest amount either left or right, ie no turns greater than 180deg. Provide own function if you want to turn
    # longer way around for some reason  e.g. 270degrees left instead of 90degrees right
    #
    # @param Input heading reflects the true HEADING we want the robot to finish at
    # Returns the scaled turn ANGLE with negative value up to -180deg * gyro_scale for left turn and positive value up to +180deg * scale_scale for right turn
    #
    # NOTE: The scaled return value in this case will *not* represent true motion of the robot, but rather the value we want from the gyro to get this motion
    # Therefore, returned value may exceed -180 to +180 degree range necessarily to compensate for a robot that underturns, so we apply the scale factor last,
    # meaning do not apply any additional limit code or bounds checking on the return value
    @staticmethod
    def calc_angle_to_heading(heading):
        # read corrected sensor as HEADING - this should reflect the robot's true HEADING, assuming scale factor is set correctly and sensor has not
        # drifted too much
        current_heading = GyroHelper.gyro_heading()
        # delta_heading will be the difference between the desired (real) heading and current (real) heading
        delta_heading = heading - current_heading
        # ensure result is in range -180deg (left turns) to +180deg (right turns) and finally multiply by scale factor
        delta_angle = GyroHelper.to_angle(delta_heading) * GYRO_SCALE_FOR_TURNS

        # returned value can be fed direcltly to drivetrain.turn_for(), but not drivetrain.turn_to_heading()
        return delta_angle

    # Computes the "raw" rotation value we want the gyro to read for a "real" HEADING
    # @param Input heading reflects the true HEADING we want the robot to finish at
    # Returns a scaled rotation value that can be used with drivetrain.turn_to_rotation()
    @staticmethod
    def calc_rotation_at_heading(heading):
        # First get the robot's total "real" rotation and heading - be careful not to read the inertial sensor twice in the same routine
        # in case it gets updated.
        current_rotation = GyroHelper.gyro_rotation()
        current_heading = GyroHelper.to_heading(current_rotation)

        # Calculate the real heading and angle delta to get to the desired heading
        delta_heading = heading - current_heading
        delta_angle = GyroHelper.to_angle(delta_heading)

        # The new rotation value will be the current + the angle delta * scale factor
        new_rotation = current_rotation + delta_angle
        new_rotation *= GYRO_SCALE_FOR_TURNS

        # Return value can be used with drivetrain.turn_to_rotation() - will not work with drivetrain.turn_to_heading()
        return new_rotation

# turn_for() is the simplest case, we just multiply by scale factor and call corresponding drivetrain command
def turn_for(turn_direction, turn_angle):
    # ... do pre turn stuff here
    smart_drive.turn_for(turn_direction, turn_angle * GYRO_SCALE_FOR_TURNS, DEGREES)
    # ... do post turn stuff here

# turn_to_heading1 is a sligthly more complicated version using drivetrain.turn_for() where we provide the direction we want to end up facing rather than
# a turn amount. This is useful if we don't want to keep track of each individual turn
def turn_to_heading1(heading):
    # ... do pre turn stuff here
    smart_drive.set_drive_velocity(50, PERCENT)
    smart_drive.set_turn_velocity(50, PERCENT)
    smart_drive.set_turn_constant(TURN_CONSTANT)
    smart_drive.set_timeout(TIME_FOR_FULL_TURN + 1, SECONDS) # conservative
    smart_drive.set_stopping(BRAKE)

    # actual turn
    turn_angle = GyroHelper.calc_angle_to_heading(heading)
    # NOTE: calc_angle_to_heading() will apply the scaling factor so we don't need to do that here
    smart_drive.turn_for(RIGHT, turn_angle, DEGREES)

    # ... do post turn stuff here
    smart_drive.stop(BRAKE)

# turn_to_heading2 uses the drivetrain.turn_to_rotation() as another example as to how to turn to an absolute heading
def turn_to_heading2(heading):
    # ... do pre turn stuff here
    smart_drive.set_drive_velocity(50, PERCENT)
    smart_drive.set_turn_velocity(50, PERCENT)
    smart_drive.set_turn_constant(TURN_CONSTANT)
    smart_drive.set_timeout(TIME_FOR_FULL_TURN + 1, SECONDS) # conservative
    smart_drive.set_stopping(BRAKE)

    # actual turn
    new_rotation = GyroHelper.calc_rotation_at_heading(heading)
    smart_drive.turn_to_rotation(new_rotation, DEGREES)

    # ... do post turn stuff here
    smart_drive.stop(BRAKE)

def autonomous():
    # wait for initialization to complete
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    # place automonous code here

def full_turn_smart_drive(number_of_turns):
    smart_drive.set_drive_velocity(50, PERCENT)
    smart_drive.set_turn_velocity(50, PERCENT)
    # set the turn_constant - see NOTE above
    smart_drive.set_turn_constant(TURN_CONSTANT)
    # set the timeout - see NOTE above. We add 1 second on top of the turn to provide some margin. If you change the speed at which
    # the robot turns you need to provide your own calculation here
    smart_drive.set_timeout(TIME_FOR_FULL_TURN * number_of_turns + 1, SECONDS)
    # NOTE: here we use the inverse of gyro_scale
    smart_drive.turn_for(RIGHT, number_of_turns * 360.0 * GYRO_SCALE_FOR_TURNS, DEGREES)
    # TODO: can add out own timeout detection here
    smart_drive.stop(BRAKE)

def full_turn_drive_train(number_of_turns):
    print("DriveTrain full turn")
    drive_train.set_drive_velocity(50, PERCENT)
    drive_train.set_turn_velocity(50, PERCENT)
    # NOTE: Only difference in code here is that DriveTrain does not use the turn constant
    # drive_train.set_turn_constant(TURN_CONSTANT)
    # set the timeout - see NOTE above. We add 1 second on top of the turn to provide some margin. If you change the speed at which
    # the robot turns you need to provide your own calculation here
    # NOTE: for drivetrain, the timeout should not be needed in most circumstances unless motors are blocked
    drive_train.set_timeout(TIME_FOR_FULL_TURN * number_of_turns + 1, SECONDS)
    # NOTE: here we use the inverse of gyro_scale
    drive_train.turn_for(RIGHT, number_of_turns * 360.0 * DRIVE_TRAIN_TURN_SCALE, DEGREES)
    # TODO: can add out own timeout detection here
    drive_train.stop(BRAKE)

def full_turn(number_of_turns = 1, use_smart_drive = True):
    if use_smart_drive:
        full_turn_smart_drive(number_of_turns)
    else:
        full_turn_drive_train(number_of_turns)

# DEMO1: Use this to determine ACTUAL_ROBOT_FULL_TURN and TURN_CONSTANT
def demo1_full_turns(use_smart_drive = True):
    # NOTE: DO NOT USE inertial.heading() - the scale calculation will not work. Use either inertial.rotation() or inertial.angle()
    #  Here we multiply the value returned from the inertial sensor by the scaling factor. In our example above of robot turning 365 degrees
    #  (when we wanted 360 degrees) it means the inertial sensor is returning a too small value, so we multiply by 365/360 in this case. This is
    #  the opposite of when we tell the SmartDrive what we want it to do. Because it turns too far we want it to turn less so in the turn_far()
    #  command we tell it to turn less by a factor of 360/365 (the inverse)
    start_angle = GyroHelper.gyro_heading()
    brain.screen.print("Starting Heading: ", start_angle)
    brain.screen.next_row()
    print("Starting Heading: ", start_angle)

    if GYRO_SCALE_UNKNOWN:
        full_turn(number_of_turns = 10, use_smart_drive = use_smart_drive)
    else:
        full_turn(number_of_turns = 1, use_smart_drive = use_smart_drive)

    end_angle = GyroHelper.gyro_heading()
    brain.screen.print("End Heading: ", end_angle)
    brain.screen.next_row()
    print("End Heading: ", end_angle)

# DEMO2: Once robot has been tuned for a full turn, use this to test turning to specific headings
def demo2_turn_to_headings():
    headings = [0, 90, 180, 270, 0, 90, 180, 270, 0, 270, 180, 90, 0, 270, 180, 90, 0]
    for heading in headings:
        brain.screen.print("Turning to Heading: ", heading)
        brain.screen.next_row()
        print("Turning to Heading: ", heading)
        # choose either version of turn_to_heading() here
        # turn_to_heading1(heading)
        turn_to_heading2(heading)
        current_heading = GyroHelper.gyro_heading()
        brain.screen.print("Current Heading: ", current_heading)
        brain.screen.next_row()
        print("Current Heading: ", current_heading)
        wait(1, SECONDS)

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("Waiting for robot to initialize fully ... ")
    brain.screen.next_row()

    # wait for initialization to complete
    while not ROBOT_INITIALIZED:
        wait(10, MSEC)

    brain.screen.print("done")
    brain.screen.next_row()

    # Good idea to check if inertial sensor is present before using it as unexpected motion can occur
    if not inertial.installed():
        brain.screen.print("NO INERTAIL SENSOR")
        while True:
            wait(20, MSEC)

    demo1_full_turns(use_smart_drive=True)
    # demo2_turn_to_headings()

    drive_train.stop(COAST)
    # place driver control in this while loop
    while True:
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, autonomous)
pre_autonomous()
