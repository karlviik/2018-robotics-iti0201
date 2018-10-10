from PiBot import PiBot
import rospy

robot = PiBot()
linel1 = robot.get_rightmost_line_sensor
linel2 = robot.get_second_line_sensor_from_right
linel3 = robot.get_third_line_sensor_from_right
liner1 = robot.get_leftmost_line_sensor
liner2 = robot.get_second_line_sensor_from_left
liner3 = robot.get_third_line_sensor_from_left


def speed(perc):
    robot.set_wheels_speed(-perc)


def speedl(perc):
    robot.set_right_wheel_speed(-perc)


def speedr(perc):
    robot.set_left_wheel_speed(-perc)


left_second = robot.get_second_line_sensor_from_left()
"""
while True:
    # if data from sensor says that the colour is black, we proceed moving back (-30)
    while left_second < 300:
        left_second = robot.get_second_line_sensor_from_left()
        right_third = robot.get_third_line_sensor_from_right()
        robot.set_wheels_speed(-20)
    # if the colour is not black - stop
    robot.set_wheels_speed(0)
    # We turn left by default. If right_most sensor is black, we understand that we need to continue turning in
    # opposite direction. It is the fastest way to get a black colour on left_second sensor.
    # If the colour on left_second is black and the colour on right_most is white, start the first loop
    while True:
        left_second = robot.get_second_line_sensor_from_left()
        right_most = robot.get_rightmost_line_sensor()
        if left_second < 300 < right_most:
            break
        else:
            if left_second > 300:
                if right_most < 300:
                    while left_second > 300:
                        print("third")
                        left_second = robot.get_second_line_sensor_from_left()
                        robot.set_left_wheel_speed(-15)
                        robot.set_right_wheel_speed(15)
                else:
                    robot.set_left_wheel_speed(15)
                    robot.set_right_wheel_speed(-15)
"""
lastside = 1
while True:
    speed(20)
    while linel3() < 300 and liner3() < 300:
        rospy.sleep(0.05)
    if linel3() > 700 and liner3() > 700:
        speed(0)
        if linel2() < 300 or linel1() < 300:
            speedl(-20)
            speedr(20)
            lastside = 0
        elif liner2() < 300 or liner1() < 300:
            speedl(20)
            speedr(-20)
            lastside = 1
        else:
            if lastside:
                speedl(20)
                speedr(-20)
            else:
                speedl(-20)
                speedr(20)
    elif linel3() > 700:
        speedr(20)
        lastside = 0
    elif liner3() > 700:
        speedl(20)
        lastside = 1
    rospy.sleep(0.05)
