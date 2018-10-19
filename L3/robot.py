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
    :return: None
    """
    robot.set_wheels_speed(-perc)


def speedl(perc):
    """
    Set right wheel speed as negative of perc(entage) because treating back as front.

    :param perc: percentage
    :return: None
    """
    robot.set_right_wheel_speed(-perc)


def speedr(perc):
    """
    Set left wheel speed as negative or perc(entage) because treating back as front.

    :param perc: percentage
    :return: None
    """
    robot.set_left_wheel_speed(-perc)


def turn(perc):  # negative speed turns left, positive right
    """
    Make the robot turn in a stationary position. Positive input turns right and negative left when treating front as back.

    :param perc: percentage
    :return: None
    """
    speedr(-perc)
    speedl(perc)


def main():
    """
    Function that controls movement, while robot is on a black line.

    :return: None
    """
    while True:
        l3, r3 = getlinel3(), getliner3()
        if l3 < 600 and r3 < 600:
            if forward_distance_l() < 0.07 or forward_distance_r() < 0.07:
                break
            speed(20)

        l1, l2, l3, r3, r2, r1 = getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
        check()

        if forward_distance_l() < 0.07 or forward_distance_r() < 0.07:
            break

        if l1 > 600 and l2 > 600 and l3 > 600 and r3 > 600 and r2 > 600 and r1 > 600:
            turn(20)
        rospy.sleep(0.005)


def check():
    """
    Function makes small changes in robot's direction in order to make movement smoother.

    :return: None
    """
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


def move_along_wall(value, side, bol):
    """
    Function controls robot's movement while robot moves along the wall.

    :param value: float == value of a particular sensor, depends on the first turn in loop.
    :param side: the first value of sensor at all.
    :return: flag == int (1/0). 1 if robot detected black line. 0 if didn't
    """
    if not bol:
        speed2, speed1 = speedr, speedl
    else:
        speed1, speed2 = speedr, speedl
    print("move along")
    flag = 0
    if value < 0.02:
        value += 0.01
    while True:
        if getlinel3() < 300 or getliner3() < 300 or getliner1() < 300 or getlinel1() < 300:
            flag = 1
            break
        if value - side() < -0.02:
            break
        elif abs(value - side()) <= 0.002:
            rospy.sleep(0.05)
            speed(17)
        elif value - side() > 0.002:
            speed1(15)
            speed2(18)
        elif value - side() < -0.002:
            speed1(18)
            speed2(15)
        print(value - side())
        rospy.sleep(0.005)
    speed(0)
    return flag


def precise_turn(side):  # side 0 is left, side 1 is kright
    """
    Robot makes 90 degree turn.

    :param side: int (1/0). 1 means turn right. 0 means turn left.
    """
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
    """
    Robot moves just forward.

    :param value: integer: robot changes its wheel_encoder on value.
    :return: None
    """
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
    """
    Robot moves forward until the moment while side sensors detect wall.

    :param value: int: 1 == left, 0 == right side. Depends on initial turn.
    :return: None
    """
    print("until the wall")
    if value:
        while distance_left() > 0.1:
            speed(20)
    else:
        while distance_right() > 0.1:
            speed(20)
    speed(0)


def left():
    """
    Function is responsible for robot's movement if it turns left.

    :return: None
    """
    precise_turn(0)
    while True:
        flag = move_along_wall(distance_right(), distance_right, 0)
        if flag:
            break
        flag = move_forward(450)
        if flag:
            break
        precise_turn(1)
        move_until_the_wall(0)
    move_forward(200)


def right():
    """
    Function is responsible for robot's movement if it turns right.

    :return: None
    """
    precise_turn(1)
    while True:
        flag = move_along_wall(distance_left(), distance_left, 1)
        if flag:
            break
        flag = move_forward(450)
        if flag:
            break
        precise_turn(0)
        move_until_the_wall(1)
    move_forward(200)


def control():
    """
    Function of general control. It is responsible for robot's initial turn.

    :return: None
    """
    obstacles = 0
    while True:
        if obstacles == 1:
            while getliner3() > 600 and getlinel3() > 600:
                turn(-20)
        elif obstacles == 2:
            while getliner3() > 600 and getlinel3() > 600:
                turn(20)
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


print(forward_distance_l())
control()
