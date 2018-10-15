"""Take a shot at following the line. White is lava."""
from PiBot import PiBot
import rospy
from math import pi

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
left_distance = robot.get_right_wheel_encoder
right_distance = robot.get_right_wheel_encoder


def speed(perc):
    """
    Set wheel speeds, but treats backwards as forwards and vice versa. Mostly for shorter typing.

    :param perc: percentage
    :return: nada
    """
    robot.set_wheels_speed(-perc)


def speedl(perc):
    """
    Set right wheel speed as negative of perc(entage) because treating back as front.

    :param perc: percentage
    :return: nada
    """
    robot.set_right_wheel_speed(-perc)


def speedr(perc):
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
    speedr(-perc)
    speedl(perc)


def preciseturn(side, turnspeed):  # side 0 is left, side 1 is right
    speed(0)
    wheelturngoal = (robot.AXIS_LENGTH / robot.WHEEL_DIAMETER) * 90
    enc = robot.get_right_wheel_encoder()
    if side:
        encgoal = robot.get_right_wheel_encoder() - wheelturngoal
        turn(turnspeed)
        while enc > encgoal:
            rospy.sleep(0.01)
            enc = robot.get_right_wheel_encoder()
    else:
        encgoal = robot.get_right_wheel_encoder() + wheelturngoal
        turn(-turnspeed)
        while enc < encgoal:
            rospy.sleep(0.01)
            enc = robot.get_right_wheel_encoder()


def crossing(crosscount):
    print("This is turn number " + crosscount + 1)
    speed(0)
    lenc = robot.get_right_wheel_encoder()
    # 200 == distance from the present position of robot to the middle of the crossroad ahead.
    lencgoal = lenc - 200
    speed(17)
    while lenc > lencgoal:
        rospy.sleep(0.01)
        lenc = robot.get_right_wheel_encoder()
    if crosscount % 3 == 0:
        preciseturn(0, 20)
    elif crosscount % 3 == 2:
        preciseturn(1, 20)  # make the bot turn 90 degrees RIGHT
    return crosscount + 1


def main():
    countandturn = 0
    last_side = 0
    while True:
        while linel3() < 600 and liner3() < 600:
            # print("move forward")
            if linel1() < 600 or liner1() < 600:
                print("finita la commedia")
                countandturn = crossing(countandturn)
            speed(20)

        # it has to turn to the left if linel1() < 600.
        if linel1() < 600:
            last_side = 1
            while liner3() > 600:
                turn(-15)
        # it has to turn to the right if liner1() < 600 respectively
        elif liner1() < 600:
            last_side = 0
            while linel3() > 600:
                turn(15)
        # condition for maneuvering. in other words, we can just change speed of different vehicles to adjust the
        # trajectory of the robot.
        elif linel2() < 600 and linel3() < 600:
            speedl(15)
            speedr(20)
        elif liner2() < 600 and liner3() < 600:
            speedr(15)
            speedl(20)
        # try to predict direction of the next turn. if the robot turned to the left, then it is
        # more possible that the next turn has to be in the same direction (case of loop).
        else:
            if last_side:
                turn(-15)
            else:
                turn(15)
        # condition to catch a crossroad when robot doesn't move straight.
        if linel1() < 600 and linel3() < 600 or linel1() < 600 and liner3() < 600 or liner1() < 600 \
                and linel3() < 600 or liner1() < 600 and liner3() < 600:
            print("surprise!")
            countandturn = crossing(countandturn)


main()

"""
def updatelines():
    return [floor(getlinel1() / 512), floor(getlinel2() / 512), floor(getlinel3() / 512), floor(getliner3() / 512), floor(getliner2() / 512), floor(getliner1() / 512)]
"""