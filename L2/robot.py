"""Take a shot at following the line. White is lava."""
from PiBot import PiBot
import rospy


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
    """
    Set wheel speeds, but treats backwards as forwards and vice versa. Mostly for shorter typing.

    :param perc: percentage
    :return: nada
    """
    robot.set_wheels_speed(-perc)


def setspeedl(perc):
    """
    Set right wheel speed as negative of perc(entage) because treating back as front.

    :param perc: percentage
    :return: nada
    """
    robot.set_right_wheel_speed(-perc)


def setspeedr(perc):
    """
    Set left wheel speed as negative or perc(entage) because treating back as front.

    :param perc: percentage
    :return: nada
    """
    robot.set_left_wheel_speed(-perc)


def turn(perc):  # negative speed turns left, positive right
    """
    Make the robot turn in a stationary position. Positive input turns right and negative left when treating front as back.

    :param perc: percentage
    :return: nada
    """
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
        lencgoal = robot.get_right_wheel_encoder - wheelturngoal
        setspeedl(speed)
        if not mode:
            setspeedr(-speed)
        clenc = robot.get_right_wheel_encoder
        while clenc > lencgoal:
            rospy.sleep(0.01)
            clenc = robot.get_right_wheel_encoder
    else:
        rencgoal = robot.get_left_wheel_encoder - wheelturngoal
        setspeedr(speed)
        if not mode:
            setspeedl(-speed)
        crenc = robot.get_left_wheel_encoder
        while crenc > rencgoal:
            rospy.sleep(0.01)
            crenc = robot.get_left_wheel_encoder
    setspeed(25)


def crossing(crosscount):
    if crosscount % 3 == 0:
        preciseturn(90, 1, 20)
    # if crosscount % 3 == 1:
        # make the bot ignore everything and move straight
    elif crosscount % 3 == 2:
        preciseturn(90, 1, 20) # make the bot turn 90 degrees RIGHT
    return crosscount + 1


def updatelines():
    return [getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()]


counter = 0
lastside = 1
while True:
    setspeed(25)
    lines = updatelines()
    print(lines)
    while lines[2] < 300 and lines[3] < 300:
        if lines[1] < 300 and lines[0] > 700:
            setspeedl(20)
            lastside = 0
        elif lines[4] < 300 and lines[5] > 700:
            setspeedr(20)
            lastside = 1
        elif lines[0] < 300 or lines[5] < 300:
            counter = crossing(counter)
            print("middles are on, l0 or l5 is 300")
        else:
            setspeed(25)
        rospy.sleep(0.01)
        lines = updatelines()
    while lines[2] > 700 and lines[3] < 300:
        if lines[0] > 700 and lines[5] > 700:
            setspeedr(20)
            lastside = 1
        if lines[0] < 300 or lines[5] < 300:
            counter = crossing(counter)
            print("l2 is 700 and l3 is 300, l0 or l5 is 300")
        rospy.sleep(0.01)
        lines = updatelines()
    while lines[2] < 300 and lines[3] > 700:
        if lines[0] > 700 and lines[5] > 700:
            setspeedl(20)
            lastside = 0
        if lines[0] < 300 or lines[5] < 300:
            counter = crossing(counter)
            print("l2 is 300 and l3 is 700, l0 or l5 is 300")
        rospy.sleep(0.01)
        lines = updatelines()
    while lines[2] > 700 and lines[3] > 700:
        if lastside:
            turn(20)
        else:
            turn(-20)
        rospy.sleep(0.01)
        lines = updatelines()
    rospy.sleep(0.01)
    lines = updatelines()
