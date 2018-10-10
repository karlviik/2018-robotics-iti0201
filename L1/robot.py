from PiBot import PiBot
import rospy

robot = PiBot()
left_second = robot.get_second_line_sensor_from_left()

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