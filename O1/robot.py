"""Fid object and move to it."""
import rospy
from PiBot import PiBot

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
    return fmir_average, [fmir_average, fmir_average, fmir_average]


def fmir_buffering(variables):
    """
    See, if fmir value is correct. If not, return last fmir.

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
    variables["left_enc"] = robot.get_left_wheel_encoder()
    variables["right_enc"] = robot.get_right_wheel_encoder()
    variables = fmir_buffering(variables)  # updates fmir with buffer
    return variables


def plan(variables):
    if variables["phase"] == "scanning":
        pass
        # if fmir difference is more than 15 cm or something
            # then stop scanning and zero in on the object?
        # else
            # if haven't turned like 1.5 turns:
                # change wheel speed according to encoders and stuff, P/I/D controlling or something
    elif variables["phase"] == "zero to obj":
        pass  # this part should aim itself directly at the object with PIDing
        # take 3 measurements per angle and try to get as close as possible. When got to as close as possible, check the direction haven't checked yet
    elif variables["phase"] == "move to obj":
        pass  # this part should move straight to obj until like 15cm close while PIDing
    elif variables["phase"] == "blind to obj":
        pass  # this part should move the last X distance to obj
    return variables


def act(variables):
    robot.set_left_wheel_speed(variables["left_speed"])
    robot.set_right_wheel_speed(variables["right_speed"])


def main():
    variables = dict()
    variables["left_speed"] = 0
    variables["right_speed"] = 0
    variables["last_fmir"], variables["fmir_buffer"] = fmir_buffer_init()
    variables["phase"] = "scanning"

    while True:
        variables = sense(variables)
        variables = plan(variables)
        act(variables)


if __name__ == "__main__":
    main()
