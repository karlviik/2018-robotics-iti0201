"""Take a shot at following the line. White is lava."""
from PiBot import PiBot
import rospy

robot = PiBot()
# This whole section is just to make writing stuff easier, means linel1() is same as robot.get_rightmost_toomuchtext()
# To note is that left and right sides are switched as in this task robot always moves backwards, so might aswell
# treat robot's back as its front.
get_line_l1 = robot.get_rightmost_line_sensor
get_line_l2 = robot.get_second_line_sensor_from_right
get_line_l3 = robot.get_third_line_sensor_from_right
get_line_r1 = robot.get_leftmost_line_sensor
get_line_r2 = robot.get_second_line_sensor_from_left
get_line_r3 = robot.get_third_line_sensor_from_left
left_distance = robot.get_right_wheel_encoder
right_distance = robot.get_right_wheel_encoder


def speed(speed_percentage):
    """
    Set wheel speeds, but treats backwards as forwards and vice versa. Mostly for shorter typing.

    :param speed_percentage: percentage
    :return: nada
    """
    robot.set_wheels_speed(-speed_percentage)


def speed_left(speed_percentage):
    """
    Set right wheel speed as negative of perc(entage) because treating back as front.

    :param speed_percentage: percentage
    :return: nada
    """
    robot.set_right_wheel_speed(-speed_percentage)


def speed_right(speed_percentage):
    """
    Set left wheel speed as negative or perc(entage) because treating back as front.

    :param speed_percentage: percentage
    :return: nada
    """
    robot.set_left_wheel_speed(-speed_percentage)


def turn(speed_percentage):
    """
    Make the robot turn in a stationary position. Positive input turns right and negative left when treating front as back.

    :param speed_percentage: percentage
    :return: nada
    """
    speed_right(-speed_percentage)
    speed_left(speed_percentage)


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


def crossing(crossing_count: int):
    """
    Manage 90 degree turns at intersections.

    :param crossing_count: current count of crossings already made
    :return: crosscount + 1
    """
    print("This is turn number " + str(crossing_count + 1))
    speed(0)

    # moves bot forward a bit so it would do 90 degree turn more or less directly on crossing
    left_encoder = robot.get_right_wheel_encoder()
    left_encoder_goal = left_encoder - 200  # 200 is approximate distance from current position to the middle of the crossroad
    speed(17)
    while left_encoder > left_encoder_goal:
        rospy.sleep(0.005)
        left_encoder = robot.get_right_wheel_encoder()

    # initiates the 90 degree turns
    if crossing_count % 3 == 0:
        precise_turn(0, 20)  # turn left with speed 20
    elif crossing_count % 3 == 2:
        precise_turn(1, 20)  # turn right with speed 20

    speed(20)
    return crossing_count + 1


def main():
    """
    Main function, move the bot forward and correct course and detect intersections and initiate turning.

    :return: None
    """
    count_and_turn = 0
    last_side = 0
    while True:
        left_line_1, left_line_2, left_line_3, right_line_3, right_line_2, right_line_1 = get_line_l1(), get_line_l2(), get_line_l3(), get_line_r3(), get_line_r2(), get_line_r1()
        if left_line_2 > 600 and right_line_2 > 600 and (left_line_3 < 600 or right_line_3 < 600):
            speed(20)
            print("{")
            while get_line_l2() > 600 and get_line_r2() > 600 and (get_line_l3() < 600 or get_line_r3() < 600):
                if get_line_l1() < 600 or get_line_r1() < 600:
                    # print("     finita la commedia")
                    # print("     " + str(left_line_1) + str(left_line_2) + str(left_line_3) + str(right_line_3) + str(right_line_2) + str(right_line_1))
                    count_and_turn = crossing(count_and_turn)
                rospy.sleep(0.01)
            print("}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}")
            left_line_1, left_line_2, left_line_3, right_line_3, right_line_2, right_line_1 = get_line_l1(), get_line_l2(), get_line_l3(), get_line_r3(), get_line_r2(), get_line_r1()
        # condition for maneuvering. in other words, we can just change speed of different vehicles to adjust the
        # trajectory of the robot.
        if left_line_3 < 600:
            last_side = 1
            speed_left(15)
            speed_right(20)
        elif right_line_3 < 600:
            last_side = 0
            speed_right(15)
            speed_left(20)
        # it has to turn to the left if L1 is black
        # elif (left_line_1 < 600 or left_line_2 < 600) and 600 < right_line_3:
        #    last_side = 1
        #    turn(-15)
        #    while left_line_3 > 600:  # changed right_line_3 to left_line_3 inside the loop
        #        rospy.sleep(0.005)
        #        left_line_3 = getliner3()
        # it has to turn to the right if R1 is black
        # elif (right_line_1 < 600 or right_line_2 < 600) and 600 < left_line_3:
        #    last_side = 0
        #    turn(15)
        #    while right_line_3 > 600:  #  changed left_line_3 to right_line_3 inside the loop
        #        rospy.sleep(0.005)
        #        right_line_3 = getlinel3()
        # try to predict direction of the next turn. if the robot turned to the left, then it is
        # more possible that the next turn has to be in the same direction (case of loop).
        else:
            check = 1
            # left_line_1, left_line_2, left_line_3, right_line_3, right_line_2, right_line_1 = getlinel1(), getlinel2(), getlinel3(), getliner3(), getliner2(), getliner1()
            if (left_line_1 < 600 or left_line_2 < 600) and right_line_2 > 600 and right_line_1 > 600:
                last_side = 1
            elif (right_line_1 < 600 or right_line_2 < 600) and left_line_2 > 600 and left_line_1 > 600:
                last_side = 0
            elif right_line_3 < 600 or left_line_3 < 600:
                check = 0
            print("{\n     I just turned!")
            print("     " + str(left_line_1) + str(left_line_2) + str(left_line_3) + str(right_line_3) + str(right_line_2) + str(right_line_1))
            if last_side and check:  # true is left
                turn(-15)
                while left_line_3 > 600 or (left_line_2 > 600 and right_line_3 > 600):  # expanded these checks
                    if right_line_1 < 600 or right_line_2 < 600 or right_line_3 < 600:
                        last_side = 0
                        speed(0)
                        break
                    rospy.sleep(0.01)
                    left_line_2, left_line_3, right_line_3, right_line_2, right_line_1 = get_line_l2(), get_line_l3(), get_line_r3(), get_line_r2(), get_line_r1()
                speed(0)
            elif check:
                turn(15)
                while right_line_3 > 600 or (right_line_2 > 600 and left_line_3 > 600):
                    if left_line_1 < 600 or left_line_2 < 600 or left_line_3 < 600:
                        last_side = 1
                        speed(0)
                        break
                    rospy.sleep(0.01)
                    right_line_2, right_line_3, left_line_3, left_line_2, left_line_1 = get_line_r2(), get_line_r3(), get_line_l3(), get_line_l2(), get_line_l1()
                speed(0)
            print("     And I just finished turning!\n}}}}}}}}}}}}")
        left_line_1, left_line_2, left_line_3, right_line_3, right_line_2, right_line_1 = get_line_l1(), get_line_l2(), get_line_l3(), get_line_r3(), get_line_r2(), get_line_r1()
        print(left_line_1, left_line_2, left_line_3, right_line_3, right_line_2, right_line_1)
        # condition to catch a crossroad when robot doesn't move straight.
        if (left_line_1 < 600 or right_line_1 < 600) and (left_line_3 < 600 or right_line_3 < 600) or (left_line_2 < 600 and right_line_1 < 600) or (right_line_2 < 600 and left_line_1 < 600) or (left_line_2 < 600 and left_line_3 < 600 and right_line_3 < 600 and right_line_2 < 600) or (right_line_1 < 600 and left_line_1 < 600):
            print("surprise!")
            count_and_turn = crossing(count_and_turn)
        rospy.sleep(0.05)


main()
