from PiBot import PiBot
import rospy

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
    robot.set_wheels_speed(-perc)


def speedl(perc):
    robot.set_right_wheel_speed(-perc)


def speedr(perc):
    robot.set_left_wheel_speed(-perc)


def turn(perc):  # negative speed turns left, positive right
    speedr(-perc)
    speedl(perc)


def waitblack(side):  # waits for black to go under corresponding sensor
    if side:
        while liner3 > 700:
            rospy.sleep(0.025)
    else:
        while linel3 > 700:
            rospy.sleep(0.025)


lastside = 1
while True:
    speed(20)
    while linel3() < 300 and liner3() < 300:
        rospy.sleep(0.025)
    if linel3() > 700 and liner3() > 700:  # prolly better to use whiles instead of ifs to not do useless tasks
        speed(0)
        if linel2() < 300 or linel1() < 300:
            turn(-20)
            lastside = 0
            waitblack(0)
        elif liner2() < 300 or liner1() < 300:
            turn(20)
            lastside = 1
            waitblack(1)
        else:
            if lastside:
                turn(20)
                waitblack(1)
            else:
                turn(-20)
                waitblack(0)
    elif linel3() > 700:  # these can't really use while loops
        speedr(15)
        lastside = 1
        rospy.sleep(0.025)
    elif liner3() > 700:
        speedl(15)
        lastside = 0
        rospy.sleep(0.025)
