"""Find object and move to it. Or something."""
import rospy
from PiBot import PiBot
import math
GAIN = 20  # TODO: get good gain constant

robot = PiBot()


def fmir_buffer_init():
    """
    Fill fmir buffer with average value over 10 measurements.

    :return: dict of variables
    """
    total = 0
    for i in range(10):
        total += robot.get_front_middle_ir()
        rospy.sleep(0.05)
    fmir_average = total / (i + 1)
    return fmir_average, [fmir_average, fmir_average, fmir_average], fmir_average


def fmir_buffering(variables):
    """
    See if fmir value is correct. If not, return last fmir.

    :param variables: variables dict
    :return: variables dict with new fmir and buffer
    """
    buffer = variables["fmir_buffer"]
    fmir = robot.get_front_middle_ir()
    variables["last_fmir"] = variables["front_mid_ir"]
    buffer.pop(0)
    buffer.append(fmir)
    average = (buffer[0] + buffer[1] + buffer[2]) / 3
    if average * 0.85 < fmir < average * 1.15:
        variables["front_mid_ir"] = fmir
    else:
        variables["front_mid_ir"] = variables["last_fmir"]
    variables["fmir_buffer"] = buffer
    return variables


def sense(variables):
    """Get left, right wheel encoders and front middle IR sensor value, put to var dict and return the dict."""
    variables["last_left_enc"] = variables["left_enc"]
    variables["last_right_enc"] = variables["right_enc"]
    variables["left_enc"] = robot.get_left_wheel_encoder()
    variables["right_enc"] = robot.get_right_wheel_encoder()

    variables = fmir_buffering(variables)  # updates fmir with buffer

    variables["last_time"] = variables["current_time"]
    variables["current_time"] = rospy.get_time()
    return variables


def p_speed(variables, method, target_speed):  # target speed should be in meters/second
    """
    Control left and right wheel speed with P control method.

    :param variables: dictionary with all the variables
    :param method: 1 for clockwise turning, 2 for moving straight
    :param target_speed: speed to aim for
    :return: dictionary with new left and right wheel speeds
    """
    r_dist = math.pi * robot.WHEEL_DIAMETER * (360 / (variables["right_enc"] - variables["last_right_enc"]))
    l_dist = math.pi * robot.WHEEL_DIAMETER * (360 / (variables["left_enc"] - variables["last_left_enc"]))
    time_diff = variables["current_time"] - variables["last_time"]
    print(r_dist, l_dist, time_diff, variables["right_enc"], variables["last_right_enc"], variables["left_enc"], variables["last_left_enc"])
    r_speed = r_dist / time_diff
    l_speed = l_dist / time_diff
    l_error = target_speed - l_speed
    if method == 1:  # clockwise turning
        r_error = - target_speed - r_speed
    elif method == 2:   # moving straight
        r_error = target_speed - r_speed
    else:
        raise KeyError("Just some error, yo. Method has to be either 1 or 2 for p_speed")
    print(r_speed, l_speed)
    variables["left_speed"] = math.ceil(variables["left_speed"] + GAIN * l_error)
    variables["right_speed"] = math.ceil(variables["right_speed"] + GAIN * r_error)
    return variables


def plan(variables):
    """Do all the planning in variable dict and then return it. Because Python."""
    if variables["phase"] == "scanning":
        if variables["scan_progress"] == 0:
            variables["left_speed"], variables["right_speed"] = 12, -12
            variables["scan_progress"] = 1
        else:
            diff = variables["last_fmir"] - variables["front_mid_ir"]
            print("Differnece is: " + str(diff))
            print(variables["left_speed"], variables["right_speed"])
            print(variables["fmir_buffer"])
            print("------------------------------------------------------")
            if abs(diff) > 0.15:  # if difference between last valid and current valid measurement is bigger than 15 cm
                #if diff > 0:  # if bot scans clockwise and bot is currently on object:
                    # TODO: REQUIRES NEXT PHASE DONE! these parts should affect the next phase, ie if on obj, turn away and then scan sector by sector until exactly on object.
                #    pass
                #else:  # if bot has passed the object:
                    # TODO: REQUIRES NEXT PHASE DONE! these parts should affect the next phase, ie if not on obj, no need to turn away
                #    pass
                #variables["phase"] = "zero to obj"  # activate next phase
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = "move to obj"
            else:
                variables = p_speed(variables, 1, 0.05)
    elif variables["phase"] == "zero to obj":
        # TODO: do a full pass over the object, sector by sector, and if get the object, turn back to it (distance starts increasing after obj again) Also P/I/D again
        pass  # this part should aim itself directly at the object with PIDing
        # take 3 measurements per angle and try to get as close as possible. When got to as close as possible, check the direction haven't checked yet
    elif variables["phase"] == "move to obj":
        if variables["left_speed"] == 0:
            variables["left_speed"], variables["right_speed"] = 12, 12
        p_speed(variables, 2, 0.1)
        # TODO: rather straightforward (haha pun-ish) thing, move SLOWLY towards object because of fmir buffer measuring. Could also stop every X distance.
        pass  # this part should move straight to obj until like 15cm close while PIDing
    elif variables["phase"] == "blind to obj":
        # TODO: part where it gets the measurement and based on that blindly moves towards object for some distance (not time)

        pass  # this part should move the last X distance to obj
    return variables


def act(variables):
    """Change left and right wheel speed based on plan()."""
    robot.set_left_wheel_speed(variables["left_speed"])
    robot.set_right_wheel_speed(variables["right_speed"])


def main():
    """Create some variables used in the loop and then run the loop of sense, plan, act."""
    variables = dict()
    variables["left_speed"] = 0
    variables["right_speed"] = 0
    variables["left_enc"] = robot.get_left_wheel_encoder()
    variables["right_enc"] = robot.get_right_wheel_encoder()
    variables["last_fmir"], variables["fmir_buffer"], variables["front_mid_ir"] = fmir_buffer_init()
    variables["phase"] = "scanning"
    variables["scan_progress"] = 0
    variables["current_time"] = 0
    variables["last_time"] = 0
    while True:
        variables = sense(variables)
        variables = plan(variables)
        act(variables)
        rospy.sleep(0.05)


if __name__ == "__main__":
    main()
