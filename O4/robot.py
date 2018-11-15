"""Find object and move to it. Or something."""
import rospy
from PiBot import PiBot
import math
GAIN = 50

# problems:
#           a)  (--noise --cone) it may detect objects when there is none, making it not suitable for silver and gold
#               as it means it would register probably more than 3 objects with a full turn due to noise
#
#           b)  (--cone) it can detect the object at the very left edge and as it moves towards the object it can
#               lose the object as cone no longer hits it, resulting in another scanning. Only happens sometimes.

# TODO: check if all instances of "move_to_obj_counter" work properly
# TODO: (meaning they don't break valid object detections and always trigger on fake object detections)

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

    # if new fmir reading is within the allowed deviation of average buffer value
    if average * 0.85 < fmir < average * 1.15:
        variables["fmir"] = fmir
    else:
        variables["fmir"] = variables["last_fmir"]

    variables["fmir_buffer"] = buffer  # read new buffer into dictionary
    return variables  # return the new dictionary


def sense(variables):
    """Get left, right wheel encoders and front middle IR sensor value, put to var dict and return the dict."""
    # put last values into respective dict keys
    variables["last_left_enc"] = variables["left_enc"]
    variables["last_right_enc"] = variables["right_enc"]

    # read new values in
    variables["left_enc"] = robot.get_left_wheel_encoder()
    variables["right_enc"] = robot.get_right_wheel_encoder()

    variables = fmir_buffering(variables)  # updates "fmir", "last_fmir" and "fmir_buffer" dict keys

    variables["last_time"] = variables["current_time"]  # put last time into respective dict key
    variables["current_time"] = rospy.get_time()  # get new time
    return variables


def p_speed(variables, method, target_speed):  # target speed should be in meters/second
    """
    Control left and right wheel speed with P control method.

    :param variables: dictionary with all the variables
    :param method: 1 for clockwise turning, 2 for moving straight
    :param target_speed: speed to aim for
    :return: dictionary with new left and right wheel speeds
    """
    # calculate distance the bot has traveled during the past cycle
    r_dist = math.pi * robot.WHEEL_DIAMETER * ((variables["right_enc"] - variables["last_right_enc"]) / 360)
    l_dist = math.pi * robot.WHEEL_DIAMETER * ((variables["left_enc"] - variables["last_left_enc"]) / 360)

    # not used currently
    variables["distance"] = (r_dist + l_dist) / 2

    # time between this and last cycle
    time_diff = variables["current_time"] - variables["last_time"]

    # calculate wheel speeds based on v = s / t
    r_speed = r_dist / time_diff
    l_speed = l_dist / time_diff

    # get left wheel speed error, no second option (unlike right wheel) because left wheel always moves straight
    l_error = target_speed - l_speed

    # get right wheel speed error, two separate versions because right wheel turns backwards during turning
    if method == 1:  # clockwise turning
        r_error = - target_speed - r_speed
    elif method == 2:   # moving straight
        r_error = target_speed - r_speed

    # calculate new right and left wheel speeds by adding rounded value of GAIN constant times wheel speed error
    variables["right_speed"] = variables["right_speed"] + round(GAIN * r_error)
    variables["left_speed"] = variables["left_speed"] + round(GAIN * l_error)

    # return dictionary with variable dictionary with new speeds
    return variables


def plan(variables):
    """Do all the planning in variable dict and then return it. Because Python."""
    # scanning phase
    if variables["phase"] == "scanning":
        # if condition that is filled every time scanning is started, starts the turning
        if variables["scan_progress"] == 0:
            variables["left_speed"], variables["right_speed"] = 12, -12
            variables["scan_progress"] = 1

            # if the scanning is starting again
            if variables["scan_measure_start"] == "":
                # set a variable for measuring how much the bot has turned
                variables["scan_measure_start"] = variables["left_enc"] - variables["right_enc"]

            # forceful - ish break out of plan() so can properly update scan rota amount before its checking
            return variables

        # calculates how much robot has turned during scanning
        variables["scan_rota_amount"] = robot.WHEEL_DIAMETER * ((variables["left_enc"] - variables["right_enc"]) - variables["scan_measure_start"]) / (2 * robot.AXIS_LENGTH)

        # if bot has turned 1.5 turns without detecting an object
        if variables["scan_rota_amount"] > (360 * 1.5) and variables["move_to_obj_counter"] < 40:
            print(variables["move_to_obj_counter"])
            variables["phase"] = "end"
            pass  # TODO: implement rotation amount tracking in line 131 else thing

        # if scanning is already in progress and hasn't turned enough
        else:
            # last and current fmir sensor reading difference, used for object detection
            diff = variables["last_fmir"] - variables["fmir"]

            # output for checking the difference, wheel speeds and the buffer
            print("Differnece is: " + str(diff))
            print(variables["left_speed"], variables["right_speed"])
            print(variables["scan_rota_amount"], variables["fmir_buffer"])
            print("------------------------------------------------------")

            # if diff is more than 20cm, then it most likely has detected an object
            if abs(diff) > 0.20:
                # stops turning and scanning and changes phase to "verify obj"
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["scan_progress"] = 0
                variables["phase"] = "verify obj"

                # saves the approximate object distance into the dictionary
                variables["obj_distance"] = variables["fmir"]

            # if it has not detected an object and it's still scanning
            else:
                # run the p controller function to adjust right and left wheel speeds
                variables = p_speed(variables, 1, 0.05)

    # verifying if it has detected an object or not, removes some cases of false positives
    elif variables["phase"] == "verify obj":
        # increment the counter for fmir buffer verifying
        variables["obj_verify_counter"] += 1

        # when buffer has gotten entirely new set of values
        if variables["obj_verify_counter"] == 5:
            # if it most likely actually is an object
            if variables["obj_distance"] * 1.1 > variables["fmir"]:
                # if it has done less than specified amount of checks, do another one
                if variables["verify_multicheck"] < 3:
                    variables["verify_multicheck"] += 1

                # if it has done enouch checks, then it prolly is an object and start moving towards it, zero multicheck
                else:
                    variables["phase"] = "move to obj"
                    variables["verify_multicheck"] = 0

            # if it ain't, go back to scanning
            else:
                variables["phase"] = "scanning"

            # zero the counter
            variables["obj_verify_counter"] = 0



    # moving to object phase
    elif variables["phase"] == "move to obj":
        # if moving to object has not started, based on wheel speed, could do same with scanning though...
        if variables["left_speed"] == 0:
            # start moving
            variables["left_speed"], variables["right_speed"] = 12, 12

            # if last time the cycle ran it was a fake object
            if variables["move_to_obj_counter"] < 40:
                variables["move_to_obj_counter"] = 0

        # if it is moving
        else:
            # increase the counter for detecting if it has actually detected an object by one
            variables["move_to_obj_counter"] = variables["move_to_obj_counter"] + 1

            # if current fmir value is more than 10 cm shorter than maximum allowed fmir value
            # NOTE: max_fmir does not reset when it does "move to obj" to "scanning" to "move to obj"
            if variables["max_fmir"] > variables["fmir"] + 0.1:
                # then put current fmir plus 10 cm as max fmir
                variables["max_fmir"] = variables["fmir"] + 0.1

            # if current fmir is longer than allowed fmir, meaning it has lost the object or there was no object
            if variables["max_fmir"] < variables["fmir"]:
                # stop moving and start scanning again
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = "scanning"

                # also allows bot to spin more if it has moved towards object but has lost sight of it
                if variables["move_to_obj_counter"] >= 40:
                    variables["scan_measure_start"] = ""

            # for when it still has object
            else:
                # do p controller for adjusting speed of wheels so it'd move as straight as possible
                p_speed(variables, 2, 0.1)

                # if bot has gotten to withing 20 cm of the object
                if variables["fmir"] < 0.20:
                    # stop moving and change phase to next one
                    variables["left_speed"], variables["right_speed"] = 0, 0
                    variables["phase"] = "blind to obj"

    # blindly moving towards object phase
    elif variables["phase"] == "blind to obj":
        # add 1 to counter, this is to get fmir buffer to be as correct as possible
        variables["blind_cycle_counter"] = variables["blind_cycle_counter"] + 1

        # if it has reached 4 new readings for the fmir buffer
        if variables["blind_cycle_counter"] == 4:
            # calculate the time goal of how long should bot move forward with said speed
            # fmir minus 0.07 means it tries to get at distance of 7 cm from the object
            variables["timegoal"] = rospy.get_time() + (variables["fmir"] - 0.07) / 0.07

            # and start moving forward
            variables["left_speed"], variables["right_speed"] = 12, 12

        # for when counter is above 4, meaning timegoal has been set and movement has been started
        elif variables["blind_cycle_counter"] > 4:
            # run the p-controller to kinda try to be at the 0.07 meters / second speed used in timegoal calculation
            variables = p_speed(variables, 2, 0.07)

            # if bot has moved more than the timegoal said
            if variables["current_time"] > variables["timegoal"]:
                # stop moving and start end phase
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = "end"

    # end phase, literally does nothing
    elif variables["phase"] == "end":
        pass

    # return dictionary with all the new values
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
    variables["scan_progress"] = 0
    variables["current_time"] = 0
    variables["last_time"] = 0
    variables["move_to_obj_counter"] = 0
    variables["blind_cycle_counter"] = 0
    variables["obj_verify_counter"] = 0
    variables["max_fmir"] = float("inf")
    variables["scan_measure_start"] = ""
    variables["verify_multicheck"] = 0

    while True:
        variables = sense(variables)
        variables = plan(variables)
        act(variables)
        rospy.sleep(0.02)


if __name__ == "__main__":
    main()
