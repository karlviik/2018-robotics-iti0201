from PiBot import PiBot
import rospy


def turn(degrees, side):
    wheelturngoal = robot.AXIS_LENGTH / (360 / degrees) / robot.WHEEL_DIAMETER * 360
    if side == 0:
        wheelturngoal = wheelturngoal * -1
    lencgoal = robot.get_left_wheel_encoder() + wheelturngoal

    robot.set_left_wheel_speed(17)
    robot.set_right_wheel_speed(-17)
    while lencgoal > robot.get_left_wheel_encoder():
        rospy.sleep(0.05)
    robot.set_wheels_speed(0)

# Create a robot instance
robot = PiBot()

# Get distance from object using the front middle IR sensor
distance_from_object = robot.get_front_middle_ir()

# Drive towards object
robot.set_wheels_speed(30)
while distance_from_object > 0.18:
    distance_from_object = robot.get_front_middle_ir()
    print(distance_from_object)
    rospy.sleep(0.05)

turn(90, 0)


# Stop the robot when done
robot.set_wheels_speed(0)