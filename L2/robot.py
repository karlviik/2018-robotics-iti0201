"""Take a shot at following the line. White is lava."""
import rospy
from PiBot import PiBot

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
    return [getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()]


lastside = 1
while True:
    setspeed(30)
    while linel3() < 300 and liner3() < 300:
        rospy.sleep(0.025)
    if linel3() > 700 and liner3() > 700:  # prolly better to use whiles instead of ifs to not do useless tasks but tried it and sometimes it spun wrong
        setspeed(0)
        if linel2() < 300 or linel1() < 300:
            turnstat(-30)
            lastside = 0
        elif liner2() < 300 or liner1() < 300:
            turnstat(30)
            lastside = 1
        else:
            if lastside:
                turnstat(30)
            else:
                turnstat(-30)
    elif linel3() > 700:  # these can't really use while loops anyways
        setspeedr(23)
        lastside = 1
    elif liner3() > 700:
        setspeedl(23)
        lastside = 0
    rospy.sleep(0.025)