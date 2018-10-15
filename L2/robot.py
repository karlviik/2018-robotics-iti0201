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


class Counter:
    def __init__(self, count=0):
        self.count = count

    def add_one(self):
        self.count += 1


counter = Counter()


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


def move():
    last_side = 0
    flag = ""
    while True:
        while linel3() < 600 and liner3() < 600:
            # print("move forward")
            if linel1() < 600 or liner1() < 600:
                flag = "break"
                break
            speed(17)
        # We have a crossroad if flag == break
        if flag == "break":
            speed(0)
            print("finita la commedia")
            distance = left_distance()
            # 200 == distance from the present position of robot to the middle of the crossroad ahead.
            while left_distance() > distance - 200:
                speed(15)
            # go to the main function.
            break

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
            distance = left_distance()
            while left_distance() > distance - 200:
                speed(15)
            break


def main():
    while True:
        move()
        speed(0)
        distance = left_distance()
        print("start turning")
        # 592  - a rule of thumb. Just calculated this value and added to the formula.
        # formula is designed to turn left
        while left_distance() < distance + (pi * 592) / 4:
            turn(-20)
        move()
        move()
        distance = right_distance()
        # to turn right
        while right_distance() > distance - (pi * 592) / 4:
            turn(20)
        speed(0)


main()

"""Take a shot at following the line. White is lava.""""""
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


def preciseturn(degrees, mode, speed):  # negative deg left, positive right. Mode 0 is both wheels, mode 1 is 1 wheel
    setspeed(0)
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


def crossing(crosscount):
    if crosscount % 3 == 0:
        preciseturn(-90, 1, 20)
    # if crosscount % 3 == 1:
        # make the bot ignore everything and move straight
    elif crosscount % 3 == 2:
        preciseturn(90, 1, 20)  # make the bot turn 90 degrees RIGHT
    return crosscount + 1


def updatelines():
    return [floor(getlinel1() / 512), floor(getlinel2() / 512), floor(getlinel3() / 512), floor(getliner3() / 512), floor(getliner2() / 512), floor(getliner1() / 512)]


lastside = 1
while True:
    setspeed(30)
    lines = updatelines()
    while not lines[3] and not lines[4]:
        rospy.sleep(0.01)
    if lines[3] and lines[4]:  # prolly better to use whiles instead of ifs to not do useless tasks but tried it and sometimes it spun wrong
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
            if getlinel1() < 300 or getlinel2() < 300 or getlinel3() < 300:
                lastside = 0
            if getliner1() < 300 or getliner2() < 300 or getliner3() < 300:
                lastside = 1
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
    rospy.sleep(0.01)"""