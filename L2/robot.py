"""Take a shot at following the line. White is lava."""
import rospy
from PiBot import PiBot
from math import floor

robot = PiBot()
# This whole section is just to make writing stuff easier, means linel1() is same as robot.get_rightmost_toomuchtext()
# To note is that left and right sides are switched as in this task robot always moves backwards, so might aswell
# treat robot's back as its front.
getlinel1 = robot.get_rightmost_line_sensor
getlinel2 = robot.get_second_line_sensor_from_right
getlinel3 = robot.get_third_line_sensor_from_right
getliner1 = robot.get_leftmost_line_sensor
getliner2 = robot.get_second_line_sensor_from_left
getliner3 = robot.get_third_line_sensor_from_left


def setspeed(perc):
    robot.set_wheels_speed(-perc)


def setspeedl(perc):
    robot.set_right_wheel_speed(-perc)


def setspeedr(perc):
    robot.set_left_wheel_speed(-perc)


def turn(perc):  # negative speed turns left, positive right
    if perc > 0:
        setspeedl(perc)
    else:
        setspeedr(perc)


def turnstat(perc):  # negative speed turns left, positive right
    setspeedr(-perc)
    setspeedl(perc)


def preciseturn(degrees, mode, speed):  # negative degrees left, positive right. Mode 0 is both wheels, mode 1 is 1 wheel
    wheelturngoal = (robot.AXIS_LENGTH / (360 / degrees) / robot.WHEEL_DIAMETER) * 360
    if mode:
        wheelturngoal = wheelturngoal * 2
    if degrees > 0:
        lencgoal = robot.get_right_wheel_encoder() - wheelturngoal
        setspeedl(speed)
        if not mode:
            setspeedr(-speed)
        clenc = robot.get_right_wheel_encoder()
        while clenc > lencgoal:
            rospy.sleep(0.01)
            clenc = robot.get_right_wheel_encoder()
    else:
        rencgoal = robot.get_left_wheel_encoder() + wheelturngoal
        setspeedr(speed)
        if not mode:
            setspeedl(-speed)
        crenc = robot.get_left_wheel_encoder()
        while crenc > rencgoal:
            rospy.sleep(0.01)
            crenc = robot.get_left_wheel_encoder()
    setspeed(30)


def crossing(crosscount):
    if crosscount % 3 == 0:
        preciseturn(-90, 1, 20)
    # if crosscount % 3 == 1:
        # make the bot ignore everything and move straight
    elif crosscount % 3 == 2:
        preciseturn(90, 1, 20) # make the bot turn 90 degrees RIGHT
    return crosscount + 1


def updatelines():
    return [floor(getlinel1() / 512), floor(getlinel2() / 512), floor(getlinel3() / 512), floor(getliner3() / 512), floor(getliner2() / 512), floor(getliner1() / 512)]


lastside = 1
while True:
    setspeed(30)
    while getlinel3() < 300 and getliner3() < 300:
        rospy.sleep(0.01)
    if getlinel3() > 700 and getliner3() > 700:  # prolly better to use whiles instead of ifs to not do useless tasks but tried it and sometimes it spun wrong
        setspeed(0)
        if getlinel2() < 300 or getlinel1() < 300:
            turnstat(-20)
            lastside = 0
            while getlinel3() > 700 and getliner3() > 700:
                rospy.sleep(0.01)
        elif getliner2() < 300 or getliner1() < 300:
            turnstat(20)
            lastside = 1
            while getlinel3() > 700 and getliner3() > 700:
                rospy.sleep(0.01)
        else:
            if lastside:
                turnstat(30)
                while getlinel3() > 700 and getliner3() > 700:
                    rospy.sleep(0.01)
            else:
                turnstat(-30)
                while getlinel3() > 700 and getliner3() > 700:
                    rospy.sleep(0.01)
    elif getlinel3() > 700:  # these can't really use while loops anyways
        setspeedr(20)
        lastside = 1
        while getlinel3() > 700 and getliner3() < 300:
            rospy.sleep(0.01)
    elif getliner3() > 700:
        setspeedl(20)
        lastside = 0
        while getliner3() > 700 and getlinel3() < 300:
            rospy.sleep(0.01)
    rospy.sleep(0.01)