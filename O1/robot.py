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
    buffer.pop(0)
    buffer.append(fmir)
    average = (buffer[0] + buffer[1] + buffer[2]) / 3
    if average * 0.85 < fmir < average * 1.15:
        variables["front_mid_ir"] = fmir
    else:
        variables["front_mid_ir"] = variables["last_fmir"]
    variables["last_fmir"] = fmir
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
    elif variables["phase"] == "turn back to object":
        pass
    elif variables["phase"] == "":
        pass
    elif variables["phase"] == "scanning":
        pass
    elif variables["phase"] == "scanning":
        pass
    elif variables["phase"] == "scanning":
        pass
    elif variables["phase"] == "scanning":
        pass
    elif variables["phase"] == "scanning":
        pass

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
