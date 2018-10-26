import rospy
from PiBot import PiBot

robot = PiBot()

# TODO: function that scans the field for the object
# TODO: function that moves towards the object until required distance

set_speed = robot.set_wheels_speed
get_flir = robot.get_front_left_ir
get_fmir = robot.get_front_middle_ir
get_frir = robot.get_front_right_ir



def turn(speed, side):
    if not side:
        speed = -speed
    robot.set_left_wheel_speed(speed)
    robot.set_right_wheel_speed(- speed)


def turn_precise(degrees, side, speed):
    wheelturngoal = (degrees * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER)
    multiplier = 1
    if side == 0:
        multiplier = -1
    wheelturngoal = wheelturngoal * multiplier
    lencgoal = robot.get_left_wheel_encoder() + wheelturngoal

    if side == 1:
        turn(speed, 1)
        while lencgoal > robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
    else:
        turn(speed, 0)
        while lencgoal < robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
    set_speed(0)


def scan_for_object():
    wheelturngoal = (360 * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER)  # full 360 degree turn
    turn(16, 1)  # does turning with speed 13 clockwise
    left_encoder = robot.get_left_wheel_encoder()
    last_middle_ir = get_fmir()
    while left_encoder < wheelturngoal:
        middle_ir = get_fmir()
        if abs(last_middle_ir - middle_ir) > 10:
            set_speed(20)
            break



scan_for_object()