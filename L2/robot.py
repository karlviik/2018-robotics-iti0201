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
            print("stuck1")
    else:
        rencgoal = robot.get_left_wheel_encoder() + wheelturngoal
        setspeedr(speed)
        if not mode:
            setspeedl(-speed)
        crenc = robot.get_left_wheel_encoder()
        while crenc > rencgoal:
            rospy.sleep(0.01)
            crenc = robot.get_left_wheel_encoder()
            print("stuck2")
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


counter = 0
lastside = 1
while True:
    setspeed(30)
    lines = updatelines()
    print(lines)
    if lines[2] < 512 and lines[3] < 512:
        while lines[2] < 512 and lines[3] < 512:  # while 2 and 3 are on black
            if 512 < lines[0] or 512 < lines[5]:  # if 0 or 5 gets black, activate crossing code.
                counter = crossing(counter)
            rospy.sleep(0.01)
            lines = updatelines()
    if lines[3] < 512 < lines[2]:
        setspeedr(20)
        lastside = 1
        while lines[3] < 512 < lines[2]:  # if only 1 of them are on black
            if lines[0] < 512 or lines[1] < 512:
                counter = crossing(counter)
            rospy.sleep(0.01)
            lines = updatelines()
    if lines[2] < 512 < lines[3]:
        setspeedl(20)
        lastside = 0
        while lines[2] < 512 < lines[3]:
            if lines[4] < 512 or lines[5] < 512:
                counter = crossing(counter)
            rospy.sleep(0.01)
            lines = updatelines()
    if lines[2] > 512 and lines[3] > 512:
        if lines[1] < 512 and lines[4] < 512 or lines[0] < 512 and lines[4] < 512 or lines[1] < 512 and lines[5] < 512:
            counter = crossing(counter)
        else:
            if lastside:
                turnstat(25)
                while lines[2] > 512 and lines[3] > 512:
                    rospy.sleep(0.01)
                    lines = updatelines()
            else:
                turnstat(-25)
                while lines[2] > 512 and lines[3] > 512:
                    rospy.sleep(0.01)
                    lines = updatelines()
            while lines[2] > 512 and lines[3] > 512 and not (
                    lines[1] < 512 and lines[4] < 512 or lines[0] < 512 and lines[4] < 512 or lines[1] < 512 and lines[
                5] < 512):
                rospy.sleep(0.01)
                lines = updatelines()

    """           
    while lines[2] < 511.9 and lines[3] > 512:
        if lines[0] > 512 and lines[5] > 512:
            setspeedl(17)
            lastside = 0
        if lines[0] < 511.9 or lines[5] < 511.9:
            counter = crossing(counter)
            print("l2 is 511.9 and l3 is 512, l0 or l5 is 511.9")
        rospy.sleep(0.01)
        lines = updatelines()
    while lines[2] > 512 and lines[3] > 512:
        setspeed(0)
        if lastside:
            turn(20)
        else:
            turn(-20)
        rospy.sleep(0.01)
        lines = updatelines()
    """
    rospy.sleep(0.01)
    lines = updatelines()
