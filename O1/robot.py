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
    if fmir > 1.2:
        fmir = 1.2
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


def move_to_obj(variables):
    """
    Move to object phase broken into a separate function to keep plan() complexity down.

    :param variables: dict of variables
    :return variables: dict of variables with new values
    """
    # if moving to object has not started, based on wheel speed, could do same with scanning though...
    if variables["left_speed"] == 0:
        # start moving
        variables["left_speed"], variables["right_speed"] = 10, 10

    # if it is moving
    else:
        print("Moving to obj, max fmir and fmir: ", variables["max_fmir"], variables["fmir"], variables["fmir_buffer"])
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

        # for when it still has object
        else:
            # do p controller for adjusting speed of wheels so it'd move as straight as possible
            p_speed(variables, 2, 0.015)

            # if bot has gotten to withing 20 cm of the object
            if variables["fmir"] < 0.20:
                # stop moving and change phase to next one
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = "blind to obj"
    return variables


def scan(variables):
    """Do scanning."""
    # if condition that is filled every time scanning is started, starts the turning
    if variables["scan_progress"] == 0:
        variables["left_speed"], variables["right_speed"] = 10, -10
        variables["scan_progress"] = 1

    # if scanning is already in progress
    else:
        # last and current fmir sensor reading difference, used for object detection
        diff = variables["last_fmir"] - variables["fmir"]

        # output for checking the difference, wheel speeds and the buffer
        print("Differnece is: " + str(diff))
        print(variables["left_speed"], variables["right_speed"])
        print(variables["fmir_buffer"])
        print("------------------------------------------------------")

        # if diff is more than 20cm, then it most likely has detected an object
        if abs(diff) > 0.30:
            # stops turning and scanning and changes phase to "move to obj"
            variables["left_speed"], variables["right_speed"] = 0, 0
            variables["scan_progress"] = 0
            variables["phase"] = "move to obj"

        # if it has not detected an object and it's still scanning
        else:
            # run the p controller function to adjust right and left wheel speeds
            variables = p_speed(variables, 1, 0.015)
    return variables


def blind(variables):
    """Do blindy stuff."""
    # add 1 to counter, this is to get fmir buffer to be as correct as possible
    variables["counter"] = variables["counter"] + 1
    if variables["counter"] == 4 and variables["other_counter"] <= 10:
        variables["other_counter"] += 1
        if variables["other_counter"] != 10:
            variables["counter"] = 0
        avg = (variables["fmir_buffer"][0] + variables["fmir_buffer"][1] + variables["fmir_buffer"][2] + variables["fmir_buffer"][3]) / 4
        if avg > variables["avg"]:
            variables["avg"] = avg
        print("avg and buffer:", avg, variables["fmir_buffer"])

    # if it has reached 4 new readings for the fmir buffer
    if variables["counter"] == 5 and variables["other_counter"] >= 10:
        # calculate the time goal of how long should bot move forward with said speed
        # fmir minus 0.07 means it tries to get at distance of 7 cm from the object
        avg = variables["avg"]
        degrees_to_target = 360 * avg / (math.pi * robot.WHEEL_DIAMETER)
        print("deg to target:", degrees_to_target, "average dist:", avg)
        variables["l_target_drive"] = variables["left_enc"] + degrees_to_target
        variables["r_target_drive"] = variables["right_enc"] + degrees_to_target

        # and start moving forward
        variables["left_speed"], variables["right_speed"] = 10, 10
        rospy.sleep(3)

    # for when counter is above 4, meaning timegoal has been set and movement has been started
    elif variables["counter"] > 5 and variables["other_counter"] >= 10:
        print("left_enc", variables["left_enc"], "target_drive", variables["l_target_drive"], variables["left_speed"],
              variables["right_speed"])
        variables = p_speed(variables, 2, 0.02)
        if variables["left_enc"] > variables["l_target_drive"] and variables["right_enc"] > variables["r_target_drive"]:
            variables["left_speed"], variables["right_speed"] = 0, 0
            variables["phase"] = "end"
    return variables


def plan(variables):
    """Do all the planning in variable dict and then return it. Because Python."""
    # scanning phase
    if variables["phase"] == "scanning":
        variables = scan(variables)

    # moving to object phase
    elif variables["phase"] == "move to obj":
        variables = move_to_obj(variables)

    # blindly moving towards object phase
    elif variables["phase"] == "blind to obj":
        variables = blind(variables)

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
    variables["counter"] = 0
    variables["other_counter"] = 0
    variables["avg"] = 0
    variables["max_fmir"] = float("inf")
    while True:
        variables = sense(variables)
        variables = plan(variables)
        act(variables)
        rospy.sleep(0.05)  # was 0.02


if __name__ == "__main__":
    main()
