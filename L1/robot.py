"""Take a shot at following the line. White is lava."""
import rospy
from PiBot import PiBot

robot = PiBot()
# This whole section is just to make writing stuff easier, means linel1() is same as robot.get_rightmost_toomuchtext()
# To note is that left and right sides are switched as in this task robot always moves backwards, so might aswell
# treat robot's back as its front.
linel1 = robot.get_rightmost_line_sensor
linel2 = robot.get_second_line_sensor_from_right
linel3 = robot.get_third_line_sensor_from_right
liner1 = robot.get_leftmost_line_sensor
liner2 = robot.get_second_line_sensor_from_left
liner3 = robot.get_third_line_sensor_from_left


def speed(perc):
    """Set wheel speeds, but treats backwards as forwards and vice versa. Mostly for shorter typing."""
    robot.set_wheels_speed(-perc)


def speedl(perc):
    """Set right wheel speed as negative of perc(entage) because treating back as front."""
    robot.set_right_wheel_speed(-perc)


def speedr(perc):
    """Set left wheel speed as negative or perc(entage) because treating back as front."""
    robot.set_left_wheel_speed(-perc)


def turn(perc):  # negative speed turns left, positive right
    """Make the robot turn in a stationary position. Positive input turns right and negative left when treating front as back."""
    speedr(-perc)
    speedl(perc)


lastside = 1
while True:
    speed(30)
    while linel3() < 300 and liner3() < 300:
        rospy.sleep(0.025)
    if linel3() > 700 and liner3() > 700:
        speed(0)
        if linel2() < 300 or linel1() < 300:
            turn(-30)
            lastside = 0
        elif liner2() < 300 or liner1() < 300:
            turn(30)
            lastside = 1
        else:
            if lastside:
                turn(30)
            else:
                turn(-30)
    elif linel3() > 700:
        speedr(23)
        lastside = 1
    elif liner3() > 700:
        speedl(23)
        lastside = 0
    rospy.sleep(0.025)
