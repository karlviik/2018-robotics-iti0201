"""Take a shot at following the line. White is lava."""
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
        print("turned right")
        setspeedl(perc)
    else:
        print("turned left")
        setspeedr(perc)


def turnstat(perc):  # negative speed turns left, positive right
    setspeedr(-perc)
    setspeedl(perc)


def preciseturn(degrees, side, speed, currentspeed):
    print("this is where turn would happen")
    rospy.sleep(0.1)
    wheelturngoal = (robot.AXIS_LENGTH / (360 / degrees) / robot.WHEEL_DIAMETER) * 360
    multiplier = 1
    if side == 1:
        multiplier = -1
    wheelturngoal = wheelturngoal * multiplier
    lencgoal = robot.get_left_wheel_encoder() + wheelturngoal

    if side == 1:
        setspeedr(currentspeed + speed)
        setspeedl(currentspeed - speed)
        while lencgoal > robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
    else:
        setspeedr(currentspeed - speed)
        setspeedl(currentspeed + speed)
        while lencgoal < robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
    print("bot used preciseturn")
    setspeed(25)


def crossing(crosscount):
    if crosscount % 3 == 0:
        preciseturn(90, 1, 10, 10)
    #if crosscount % 3 == 1:
        # make the bot ignore everything and move straight
    elif crosscount % 3 == 2:
        preciseturn(90, 0, 10, 10) # make the bot turn 90 degrees RIGHT
    return crosscount + 1


counter = 0
lastside = 1
while True:
    setspeed(25)

    # this part moves forward while middle 2 sensors are on a black, but crossing if also l1 or r1 also black
    while linel3() < 300 and liner3() < 300:
        if linel1() < 300 or liner1() < 300:
            counter = crossing(counter)
        rospy.sleep(0.025)

    if linel3() > 700 and linel1() > 700 and liner1() > 700 and liner3() < 300:
        setspeedr(20)
        lastside = 1
    elif linel3() < 300 and linel1() > 700 and liner1() > 700 and liner3() > 700:
        setspeedl(20)
        lastside = 0
    elif linel3() > 700 and liner3() > 700:
        setspeed(0)
        if lastside:
            turnstat(20)
        else:
            turnstat(-20)
    elif (linel3() < 300 or liner3() < 300) and (linel1() < 300 or liner1() < 300):
        counter = crossing(counter)

    """
    if linel3() > 700 and liner3() > 700:  # prolly better to use whiles instead of ifs to not do useless tasks but tried it and sometimes it spun wrong
        setspeed(0)
        if linel2() < 300 or linel1() < 300:
            turn(-20)
            lastside = 0
        elif liner2() < 300 or liner1() < 300:
            turn(20)
            lastside = 1
        else:
            if lastside:
                turnstat(20)
            else:
                turnstat(-20)
    """
    #elif linel3() > 700:  # these can't really use while loops anyways
    #    setspeedr(20)
    #    lastside = 1
    #elif liner3() > 700:
    #    setspeedl(20)
    #    lastside = 0
    rospy.sleep(0.025)
