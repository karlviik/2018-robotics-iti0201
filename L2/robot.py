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
left_distance = robot.get_right_wheel_encoder
right_distance = robot.get_right_wheel_encoder


def speed(percentage):
    """
    Set wheel speeds, but treats backwards as forwards and vice versa. Mostly for shorter typing.

    :param percentage: percentage
    :return: nada
    """
    robot.set_wheels_speed(-percentage)


def speed_left(percentage):
    """
    Set right wheel speed as negative of perc(entage) because treating back as front.

    :param percentage: percentage
    :return: nada
    """
    robot.set_right_wheel_speed(-percentage)


def speed_right(percentage):
    """
    Set left wheel speed as negative or perc(entage) because treating back as front.

    :param percentage: percentage
    :return: nada
    """
    robot.set_left_wheel_speed(-percentage)


def turn(percentage):
    """
    Make the robot turn in a stationary position. Positive input turns right and negative left when treating front as back.

    :param percentage: percentage
    :return: nada
    """
    speed_right(-percentage)
    speed_left(percentage)


def precise_turn(side, turn_speed):
    """
    Make robot do a 90 degree turn in given direction.

    :param side: where to turn, 0 is left, 1 is right
    :param turn_speed: speed of turning
    :return: none
    """
    speed(0)  # stop bot
    wheel_turn_goal = (robot.AXIS_LENGTH / robot.WHEEL_DIAMETER) * 90
    encoder = robot.get_right_wheel_encoder()
    if side:  # turning right
        encoder_goal = robot.get_right_wheel_encoder() - wheel_turn_goal  # encoder goal to reach to get 90 degree turn
        turn(turn_speed)
        while encoder > encoder_goal:
            rospy.sleep(0.005)
            encoder = robot.get_right_wheel_encoder()
    else:  # turning right
        encoder_goal = robot.get_right_wheel_encoder() + wheel_turn_goal
        turn(-turn_speed)
        while encoder < encoder_goal:
            rospy.sleep(0.005)
            encoder = robot.get_right_wheel_encoder()


def crossing(crossing_count):
    """
    Manage 90 degree turns at intersections.

    :param crossing_count: current count of crossings already made
    :return: crosscount + 1
    """
    print("This is turn number " + str(crossing_count + 1))
    speed(0)

    # moves bot forward a bit so it would do 90 degree turn more or less directly on crossing
    encoder = robot.get_right_wheel_encoder()
    encoder_goal = encoder - 200  # 200 is approximate distance from current position to the middle of the crossroad
    speed(17)
    while encoder > encoder_goal:
        rospy.sleep(0.005)
        encoder = robot.get_right_wheel_encoder()

    # initiates the 90 degree turns
    if crossing_count % 3 == 0:
        precise_turn(0, 17)  # turn left with speed 20
    elif crossing_count % 3 == 2:
        precise_turn(1, 17)  # turn right with speed 20

    speed(20)
    return crossing_count + 1


def turn_a_bit(last_side):
    """
    Make the bot turn if it needs turning to correct course.

    :param last_side: direction in which to turn in
    :return: new or old last_side value
    """
    check = 1
    l1, l2, l3, r3, r2, r1 = getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
    if (l1 < 600 or l2 < 600) and r2 > 600 and r1 > 600:
        last_side = 1
    elif (r1 < 600 or r2 < 600) and l2 > 600 and l1 > 600:
        last_side = 0
    elif r3 < 600 or l3 < 600:
        check = 0
    if last_side and check:  # true is left
        turn(-15)
        while l3 > 600 or (l2 > 600 and r3 > 600):  # expanded these checks
            if r1 < 600 or r2 < 600 or r3 < 600:
                last_side = 0
                speed(0)
                break
            rospy.sleep(0.01)
            l2, l3, r3, r2, r1 = getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
        speed(0)
    elif check:
        turn(15)
        while r3 > 600 or (r2 > 600 and l3 > 600):
            if l1 < 600 or l2 < 600 or l3 < 600:
                last_side = 1
                speed(0)
                break
            rospy.sleep(0.01)
            r2, r3, l3, l2, l1 = getliner2(), getliner3(), getlinel3(), getlinel2(), getlinel1()
        speed(0)
    return last_side


def main():
    """Move bot forward, correct course and detect intersections and initiate turns."""
    count_and_turn = 0
    last_side = 0
    while True:
        l1, l2, l3, r3, r2, r1 = getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
        if l2 > 600 and r2 > 600 and (l3 < 600 or r3 < 600):
            speed(16)
            print("{")
            while getlinel2() > 600 and getliner2() > 600 and (getlinel3() < 600 or getliner3() < 600):
                if getlinel1() < 600 or getliner1() < 600:
                    count_and_turn = crossing(count_and_turn)
                rospy.sleep(0.005)
            print("}}}}}}}}}}")
            l1, l2, l3, r3, r2, r1 = getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
        # condition for maneuvering. in other words, we can just change speed of different vehicles to adjust the
        # trajectory of the robot.
        if l3 < 600:
            last_side = 1
            speed_left(15)
            speed_right(20)
        elif r3 < 600:
            last_side = 0
            speed_right(15)
            speed_left(20)

        # try to predict direction of the next turn. if the robot turned to the left, then it is
        # more possible that the next turn has to be in the same direction (case of loop).
        else:
            last_side = turn_a_bit(last_side)
        l1, l2, l3, r3, r2, r1 = getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
        # print(l1, l2, l3, r3, r2, r1)
        if (l1 < 600 or r1 < 600) and (l3 < 600 or r3 < 600) or (l2 < 600 and r1 < 600) or (r2 < 600 and l1 < 600) or (l2 < 600 and l3 < 600 and r3 < 600 and r2 < 600) or (r1 < 600 and l1 < 600):
            count_and_turn = crossing(count_and_turn)
        rospy.sleep(0.002)


main()
