"""Take a shot at following the line. White is lava."""
from PiBot import PiBot
import rospy
from math import pi

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
            rospy.sleep(0.005)
            enc = robot.get_right_wheel_encoder()
    else:
        encgoal = robot.get_right_wheel_encoder() + wheelturngoal
        turn(-turnspeed)
        while enc < encgoal:
            rospy.sleep(0.005)
            enc = robot.get_right_wheel_encoder()


def crossing(crosscount):
    print("This is turn number " + str(crosscount + 1))
    speed(0)
    lenc = robot.get_right_wheel_encoder()
    # 200 == distance from the present position of robot to the middle of the crossroad ahead.
    lencgoal = lenc - 200
    speed(17)
    while lenc > lencgoal:
        rospy.sleep(0.005)
        lenc = robot.get_right_wheel_encoder()
    if crosscount % 3 == 0:
        preciseturn(0, 20)
    elif crosscount % 3 == 2:
        preciseturn(1, 20)  # make the bot turn 90 degrees RIGHT
    speed(20)
    return crosscount + 1


def main():
    countandturn = 0
    last_side = 0
    while True:
        l3, r3, l2, r2 = getlinel3(), getliner3(), getlinel2(), getliner2()
        if l3 < 600 and r3 < 600:  # or l2 > 500 and l3 < 600 or r2 > 500 and r3 < 600
            speed(20)
            while l3 < 600 and r3 < 600:
                rospy.sleep(0.005)
                l3, r3, l1, r1 = getlinel3(), getliner3(), getlinel1(), getliner1()
                if l1 < 600 or r1 < 600:
                    print("finita la commedia")
                    countandturn = crossing(countandturn)

        l1, l2, l3, r3, r2, r1 = getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
        # condition for maneuvering. in other words, we can just change speed of different vehicles to adjust the
        # trajectory of the robot.
        if l3 < 600:
            last_side = 1
            speedl(15)
            speedr(20)
        elif r3 < 600:
            last_side = 0
            speedr(15)
            speedl(20)
        # it has to turn to the left if L1 is black
        elif (l1 < 600 or l2 < 600) and 600 < r3:
            last_side = 1
            turn(-15)
            while l3 > 600:  # changed r3 to l3 inside the loop
                rospy.sleep(0.005)
                l3 = getliner3()
        # it has to turn to the right if R1 is black
        elif (r1 < 600 or r2 < 600) and 600 < l3:
            last_side = 0
            turn(15)
            while r3 > 600:  #  changed l3 to r3 inside the loop
                rospy.sleep(0.005)
                r3 = getlinel3()

        # try to predict direction of the next turn. if the robot turned to the left, then it is
        # more possible that the next turn has to be in the same direction (case of loop).
        else:
            check = 1
            l1, l2, l3, r3, r2, r1 = getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
            if l1 < 600 or l2 < 600:
                last_side = 1
            elif r1 < 600 or r2 < 600:
                last_side = 0
            elif r3 < 600 or l3 < 600:
                check = 0
            print("I just turned!")
            print(l1, l2, l3, r3, r2, r1)
            if last_side and check:  # true is left
                turn(-15)
                while l3 > 600 or (l2 > 600 and r3 > 600):  # expanded these checks
                    if r1 < 600 or r2 < 600:
                        last_side = 0
                        break
                    rospy.sleep(0.005)
                    l2, l3, r3, r2, r1 = getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
            elif check:
                turn(15)
                while r3 > 600 or (r2 > 600 and l3 > 600):
                    if l1 < 600 or l2 < 600:
                        last_side = 1
                        break
                    rospy.sleep(0.005)
                    r2, r3, l3, l2, l1 = getliner2(), getliner3(), getlinel3(), getlinel2(), getlinel1()
            print("And I just finished turning!")
        l1, l2, l3, r3, r2, r1 = getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
        # condition to catch a crossroad when robot doesn't move straight.
        if (l1 < 600 or r1 < 600) and (l3 < 600 or r3 < 600):
            print("surprise!")
            countandturn = crossing(countandturn)
        rospy.sleep(0.005)


main()

"""
def updatelines():
    return [floor(getlinel1() / 512), floor(getlinel2() / 512), floor(getlinel3() / 512), floor(getliner3() / 512), floor(getliner2() / 512), floor(getliner1() / 512)]
"""