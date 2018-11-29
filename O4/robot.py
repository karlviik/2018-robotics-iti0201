"""Do robot-y things."""
import rospy
from PiBot import PiBot
import math
GAIN = 50

robot = PiBot()

def fmir_buffer_init():
    """
    Fill fmir buffer with average value over 10 measurements. Only done once at the beginning.

    :return: last_fmir, 4-element fmir buffer and fmir, all of them average values.
    """
    # take 10 measurements, get their average and fill buffer
    total = 0
    for i in range(10):
        total += robot.get_front_middle_ir()
        rospy.sleep(0.05)
    fmir_average = total / (i + 1)
    return fmir_average, [fmir_average, fmir_average, fmir_average, fmir_average], fmir_average


def fmir_buffering(variables):
    """
    See if fmir value is correct. If not, return last fmir.

    :param variables: variables dict
    :return: variables dict with new fmir and buffer
    """
    buffer = variables["fmir_buffer"]  # read buffer into var for easier writing, could remove this though
    fmir = robot.get_front_middle_ir()  # read fmir into var
    variables["last_fmir"] = variables["fmir"]  # put last allowed fmir value into "last_fmir" dict key

    # remove oldest and add new fmir reading
    buffer.pop(0)
    buffer.append(fmir)

    # get average fmir value from buffer
    average = (buffer[0] + buffer[1] + buffer[2] + buffer[3]) / 4
    variables["fmir_buffer_avg"] = average

    # if new fmir reading is within the allowed deviation of average buffer value
    if average * 0.85 < fmir < average * 1.15:
        variables["fmir"] = fmir
    else:
        variables["fmir"] = variables["last_fmir"]

    variables["fmir_buffer"] = buffer  # read new buffer into dictionary
    return variables  # return the new dictionary


def sense(variables):
    """Put sensor values into dictionary and return it."""
    # put last encoder values into respective dict keys
    variables["last_left_enc"] = variables["left_enc"]
    variables["last_right_enc"] = variables["right_enc"]

    # read new values in
    variables["left_enc"] = robot.get_left_wheel_encoder()
    variables["right_enc"] = robot.get_right_wheel_encoder()

    # calculate how much robot has turned during the tick in degrees, clockwise
    variables["turn_amount"] = robot.WHEEL_DIAMETER * ((variables["left_enc"] - variables["right_enc"]) - (variables["last_left_enc"] - variables["last_right_enc"])) / (2 * robot.AXIS_LENGTH)


    variables = fmir_buffering(variables)  # updates "fmir", "last_fmir", "fmir_buffer_avg" and "fmir_buffer" dict keys

    variables["last_time"] = variables["current_time"]  # put last time into respective dict key
    variables["current_time"] = rospy.get_time()  # get new time

    # gets line sensors
    variables["l3"], variables["l2"], variables["l1"] = robot.get_leftmost_line_sensor(), robot.get_second_line_sensor_from_left(), robot.get_third_line_sensor_from_left()
    variables["r3"], variables["r2"], variables["r1"] = robot.get_rightmost_line_sensor(), robot.get_second_line_sensor_from_right(), robot.get_third_line_sensor_from_right()
    return variables


def p_speed(variables, method, target_speed):  # target speed should be in meters/second. "distance", "r/l_distance"
    """
    Control left and right wheel speed with P control method.

    :param variables: dictionary with all the variables
    :param method: 1 for clockwise turning, 2 for moving straight, 3 for counterclockwise turning
    :param target_speed: speed to aim for
    :return: dictionary with new left and right wheel speeds
    """
    # just a check to not do anything if speed is 0 or last speed was 0
    if (variables["right_speed"] == 0 and variables["left_speed"] == 0) or variables["p_ignore"]:
        variables["p_ignore"] = False
        return variables

    # calculate distance the bot has traveled during the past cycle
    r_dist = math.pi * robot.WHEEL_DIAMETER * ((variables["right_enc"] - variables["last_right_enc"]) / 360)
    l_dist = math.pi * robot.WHEEL_DIAMETER * ((variables["left_enc"] - variables["last_left_enc"]) / 360)

    # not used currently
    variables["distance"] = (r_dist + l_dist) / 2
    variables["r_distance"] = r_dist
    variables["l_distance"] = l_dist

    # time between this and last cycle
    time_diff = variables["current_time"] - variables["last_time"]

    # calculate wheel speeds based on v = s / t
    r_speed = r_dist / time_diff
    l_speed = l_dist / time_diff

    # get left wheel speed error
    if method == 1 or method == 2:  # clockwise turning or moving straight
        l_error = target_speed - l_speed
    elif method == 3:  # counterclockwise turning
        l_error = - target_speed - l_speed

    # get right wheel speed error, two separate versions because right wheel turns backwards during turning
    if method == 1:  # clockwise turning
        r_error = - target_speed - r_speed
    elif method == 2 or method == 3:   # moving straight or counterclockwise turning
        r_error = target_speed - r_speed

    # calculate new right and left wheel speeds by adding rounded value of GAIN constant times wheel speed error
    variables["right_speed"] = variables["right_speed"] + round(GAIN * r_error)
    variables["left_speed"] = variables["left_speed"] + round(GAIN * l_error)

    # return dictionary with variable dictionary with new speeds
    return variables


def plan(variables):
    """Do all the mathsy planning."""
    # scanning phase
    if variables["phase"] == "scanning":
        # initialisation
        if variables["init1"]:
            # make so next time it isn't triggered
            variables["init1"] = False

            # start turning clockwise
            variables["left_speed"], variables["right_speed"] = 12, -12

            # restart universal counter and initial detection subphase flag
            variables["counter"] = 0
            variables["flag"] = False

            # if scan phase is being restarted from beginning
            if variables["init2"]:
                # restart scan progress to 0
                variables["scan_progress"] = 0

        # if it's not initialisation
        else:
            # add turn amount in degrees to scan progress
            variables["scan_progress"] += variables["turn_amount"]

            # if bot has turned multiplier amount of turns without detecting an object
            if False:  # if variables["scan_progress"] > (360 * [MULTIPLIER]):
                pass
            # TODO: put jump to roaming phase here. Currently it turns infinitely. Need force return to avoid pcontrol

            # if scanning has not reached the limit
            else:
                # last and current fmir sensor difference
                diff = variables["last_fmir"] - variables["fmir"]

                # output some things
                print("Difference is:      ", diff)
                print("Left, right speeds: ", variables["left_speed"], variables["right_speed"])
                print("Scan progress:      ", variables["scan_progress"])
                print("Buffer values:      ", variables["fmir_buffer"])
                print("-----------------------------------------------------------------------------")

                # if difference is more than 20cm (only detects going onto object for simplicity) and subphase is not active
                if diff > 0.2:
                    # continue scanning, but activate a subphase in this phase. Reset counter just in case
                    variables["flag"] = True
                    variables["counter"] = 0  # in case the 20cm diff comes from error and true obj is right after

                    # save suspected object distance into a variable
                    variables["obj_distance"] = variables["fmir"]

                # if subphase is active
                elif variables["flag"]:
                    # add 1 to counter
                    variables["counter"] += 1

                    # if counter has reached 4 cycles
                    if variables["counter"] >= 4:
                        # if object is still in the cone, within 7 cm
                        print("fmir:         ", variables["fmir"])
                        print("obj distance: ", variables["obj_distance"])
                        print("their diff:   ", abs(variables["fmir"] - variables["obj_distance"]))
                        if abs(variables["fmir"] - variables["obj_distance"]) < 0.07:
                            # stop the robot
                            variables["left_speed"], variables["right_speed"] = 0, 0

                            # after it has done numerous checks and is sure it is object, continue
                            if variables["counter"] >= 15:
                                # just in case save the lesser of the 2 into object distance
                                variables["obj_distance"] = min([variables["fmir"], variables["obj_distance"]])

                                # start zeroing to object phase by trying to detect the edges
                                variables["phase"] = "zero_to_obj"
                                variables["init1"], variables["init2"] = True, False

                        # if it prolly is not an object, zero counter and cancel subphase
                        else:
                            variables["counter"] = 0
                            variables["flag"] = False

                            # if bot was stopped, start again
                            if variables["right_speed"] == 0:
                                variables["left_speed"], variables["right_speed"] = 12, -12
                                variables["p_ignore"] = True

                # if no object has been detected and it's just scanning as normal
                else:
                    pass
                    # TODO: implement something to assist with roaming direction choosing

                # do p controller for speed correction
                variables = p_speed(variables, 1, 0.025)

    # zeroing to object at long-ish distance (like up to 80cm... Hopefully)
    elif variables["phase"] == "zero_to_obj":  # init2 false: next is move to obj. True: next is blind to obj
        # initialisation
        if variables["init1"]:
            # cancel it
            variables["init1"] = False

            # put flag to false
            variables["flag"] = False

            # restart counter and start turning clockwise until obj is lost. Also zero rota progress. Force exit
            variables["counter"] = 0
            variables["left_speed"], variables["right_speed"] = 12, -12
            variables["rota_progress"] = 0
            return variables

        # update rota progress
        variables["rota_progress"] += variables["turn_amount"]

        # if is moving clockwise to detect edge(lord)
        if variables["counter"] == 0:
            # p control this, thanks
            variables = p_speed(variables, 1, 0.02)

            # if it has exited the object, save the degrees and +1 counter to go to next subphase
            if variables["fmir"] - 0.1 > variables["obj_distance"]:
                variables["r_edge"] = variables["rota_progress"]
                variables["counter"] = 1
                variables["left_speed"], variables["right_speed"] = -12, 12

        # if moving counterclockwise to detect edge
        elif variables["counter"] == 1:
            # p controller for counterclockwise turning
            variables = p_speed(variables, 3, 0.02)

            # if flag is not true, turn it true if it has gone back to object from passing it.
            if not variables["flag"]:
                if variables["fmir"] - 0.1 < variables["obj_distance"]:
                    variables["flag"] = True

            # if it has gone back to obj, start detecting for left edge
            # if has detected that it's off object again, save edge degrees and activate next subphase. Calculate goal
            elif variables["fmir"] - 0.1 > variables["obj_distance"]:
                variables["l_edge"] = variables["rota_progress"]
                variables["counter"] = 2
                variables["goal"] = (variables["l_edge"] + variables["r_edge"]) / 2

                # start turning clockwise towards object middle.
                variables["left_speed"], variables["right_speed"] = 12, -12

        # if rotating towards middle of object
        elif variables["counter"] == 2:
            # p control this, thanks
            variables = p_speed(variables, 1, 0.02)

            # stop bot if has reached it and activate next phase according to init2
            if variables["goal"] < variables["rota_progress"]:
                variables["left_speed"], variables["right_speed"] = 0, 0
                if not variables["init2"]:
                    variables["phase"] = "move_to_obj"
                else:
                    variables["phase"] = "blind_to_obj"
                variables["init1"], variables["init2"] = True, True

    elif variables["phase"] == "move_to_obj":
        if variables["init1"]:
            variables["init1"] = False
            variables["left_speed"], variables["right_speed"] = 16, 16
        else:
            p_speed(variables, 2, 0.06)

    elif variables["phase"] == "blind_to_obj":
        pass

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
    variables["last_fmir"], variables["fmir_buffer"], variables["fmir"] = fmir_buffer_init()
    variables["phase"] = "scanning"
    variables["init1"] = True  # used extensively for marking if plan phase is being run first time or not
    variables["init2"] = True  # used for marking if plan phase is being run from start or continues after interruption
    variables["flag"] = False  # may be used somewhere
    variables["p_ignore"] = False  # used to get p controller not to control

    variables["scan_progress"] = 0 #don't need?
    variables["current_time"] = 0
    variables["last_time"] = 0
    variables["move_to_obj_counter"] = 0
    variables["blind_cycle_counter"] = 0
    variables["obj_verify_counter"] = 0
    variables["max_fmir"] = float("inf")
    variables["scan_measure_start"] = ""
    variables["verify_multicheck"] = 0
    variables["closest_wall"] = float("inf")
    variables["closest_wall_encoder_diff"] = 0  # don't have to initialise this here, but doing it anyways
    variables["are_you_zeroing"] = 0
    variables["closest_obj_reading"] = float("inf")

    while True:
        variables = sense(variables)
        variables = plan(variables)
        act(variables)
        rospy.sleep(0.03)


if __name__ == "__main__":
    main()