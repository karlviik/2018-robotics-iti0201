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

# sensors that are directed strictly back
forward_distance_l = robot.get_rear_right_straight_ir
forward_distance_r = robot.get_rear_left_straight_ir

# side sensors
distance_right = robot.get_rear_left_side_ir
distance_left = robot.get_rear_right_side_ir


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


def main():
    while True:
        l3, r3 = getlinel3(), getliner3()
        if l3 < 600 and r3 < 600:
            if forward_distance_l() < 0.07 or forward_distance_r() < 0.07:
                break
            speed(20)

        l1, l2, l3, r3, r2, r1 = getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
        if l1 < 600 < r3:
            turn(-15)
            while r3 > 600:
                rospy.sleep(0.005)
                r3 = getliner3()
        elif r1 < 600 < l3:
            turn(15)
            while l3 > 600:
                rospy.sleep(0.005)
                l3 = getlinel3()
        elif l2 < 600 and l3 < 600:
            speedl(15)
            speedr(20)
        elif r2 < 600 and r3 < 600:
            speedr(15)
            speedl(20)

        if forward_distance_l() < 0.07 or forward_distance_r() < 0.07:
            break

        if l1 > 600 and l2 > 600 and l3 > 600 and r3 > 600 and r2 > 600 and r1 > 600:
            turn(15)
        rospy.sleep(0.005)


def move_along_wall(value, side):
    print("move along")
    flag = 0
    if value < 0.02:
        value += 0.01
    while True:
        if getlinel3() < 300 or getliner3() < 300 or getliner1() < 300 or getlinel1() < 300:
            flag = 1
            break
        if value - side() < -0.05:
            break
        elif abs(value - side()) <= 0.002:
            rospy.sleep(0.05)
            speedl(17)
            speedr(17)
        elif value - side() > 0.002:
            speedl(16)
            speedr(18)
        elif value - side() < -0.002:
            speedl(18)
            speedr(16)
    speed(0)
    return flag


def precise_turn(side):  # side 0 is left, side 1 is kright
    # 90 degree rotation
    wheel_turn_goal = (robot.AXIS_LENGTH / robot.WHEEL_DIAMETER) * 90
    enc = robot.get_right_wheel_encoder()
    if side:  # turn right
        enc_goal = robot.get_right_wheel_encoder() - wheel_turn_goal
        turn(20)
        while enc > enc_goal:
            rospy.sleep(0.005)
            enc = robot.get_right_wheel_encoder()
    else:  # turn left
        enc_goal = robot.get_right_wheel_encoder() + wheel_turn_goal
        turn(-20)
        while enc < enc_goal:
            rospy.sleep(0.005)
            enc = robot.get_right_wheel_encoder()
    speed(0)


# before rotation robot has to be a little bit further from the edge of the object
# it is important to have a space for movement after rotation
def move_forward(value):
    print("move forward")
    flag = 0
    lenc = robot.get_right_wheel_encoder()
    lencgoal = lenc - value
    speed(20)
    while lenc > lencgoal:
        rospy.sleep(0.005)
        lenc = robot.get_right_wheel_encoder()
        if getlinel3() < 300 or getliner3() < 300 or getliner1() < 300 or getlinel1() < 300:
            flag = 1
            break
    speed(0)
    return flag


def move_until_the_wall(value):
    print("until the wall")
    if value:
        while distance_left() > 0.1:
            speed(20)
    else:
        while distance_right() > 0.1:
            speed(20)
    speed(0)


def left():
    precise_turn(0)
    while True:
        flag = move_along_wall(distance_right(), distance_right())
        if flag:
            break
        flag = move_forward(425)
        if flag:
            break
        precise_turn(1)
        move_until_the_wall(0)


def right():
    precise_turn(1)
    while True:
        flag = move_along_wall(distance_left(), distance_left())
        if flag:
            break
        flag = move_forward(425)
        if flag:
            break
        precise_turn(0)
        move_until_the_wall(1)


def control():
    obstacles = 0
    while True:
        while True:
            if obstacles == 1:
                precise_turn(0)
            elif obstacles == 2:
                precise_turn(1)
            main()
            speed(0)
            if forward_distance_l() < forward_distance_r():
                left_side = 0
                while forward_distance_r() - forward_distance_l() > 0.002:
                    turn(-15)
            else:
                left_side = 1
                while forward_distance_l() - forward_distance_r() > 0.002:
                    turn(15)
            if left_side:
                obstacles = 1
                print("left")
                left()
            else:
                obstacles = 2
                print("rigth")
                right()


control()
