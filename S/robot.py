from PiBot import PiBot
import rospy


def turn(degrees, side, speed, currentspeed):
    wheelturngoal = (robot.AXIS_LENGTH / (360 / degrees) / robot.WHEEL_DIAMETER) * 360
    multiplier = 1
    if side == 0:
        multiplier = -1
    wheelturngoal = wheelturngoal * multiplier
    lencgoal = robot.get_left_wheel_encoder() + wheelturngoal

    if side == 1:
        robot.set_left_wheel_speed(currentspeed + speed)
        robot.set_right_wheel_speed(currentspeed - speed)
        while lencgoal > robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
    else:
        robot.set_left_wheel_speed(currentspeed - speed)
        robot.set_right_wheel_speed(currentspeed + speed)
        while lencgoal < robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
    robot.set_wheels_speed(currentspeed)


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
robot.set_wheels_speed(0)

turn(90, 0, 17, 0)

lenc = robot.get_left_wheel_encoder()
robot.set_wheels_speed(-30)
distance_wall = robot.get_front_right_ir()
while distance_wall < 80:
    distance_wall = robot.get_front_right_ir()
    rospy.sleep(0.05)
robot.set_wheels_speed(0)


# Stop the robot when done
robot.set_wheels_speed(0)