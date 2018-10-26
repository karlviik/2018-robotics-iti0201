import rospy
from PiBot import PiBot

robot = PiBot()

# TODO: function that scans the field for the object
# TODO: function that moves towards the object until required distance


def set_speed(speed):
    robot.set_left_wheel_speed(speed)
    robot.set_right_wheel_speed(speed)


def turn_precise(degrees, side, speed):
    wheelturngoal = (degrees * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER)
    multiplier = 1
    if side == 0:
        multiplier = -1
    wheelturngoal = wheelturngoal * multiplier
    lencgoal = robot.get_left_wheel_encoder() + wheelturngoal

    if side == 1:
        robot.set_left_wheel_speed(speed)
        robot.set_right_wheel_speed(- speed)
        while lencgoal > robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
    else:
        robot.set_left_wheel_speed(- speed)
        robot.set_right_wheel_speed(speed)
        while lencgoal < robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
    robot.set_wheels_speed(0)



def scan_for_object():
    pass
    # TODO: implement a way of finding the object, perhaps based on difference of sensors.
    # Just one sensor or all three sensors? Use a buffer?
    # Prolly do full circle? Or not?
    # If sensor step 1 is far, step 2 is closer, step 3 is far
    # then step 2 has object. Prolly 10cm difference is okay?

turn(180, 1, 20)
rospy.sleep(1)
turn(90, 0, 20)