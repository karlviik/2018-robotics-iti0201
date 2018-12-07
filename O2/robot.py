"""O2."""
from PiBot import PiBot
import rospy
from math import cos, sin, sqrt, asin, pi
import math
GAIN = 50
robot = PiBot()
# TODO kontrollib kas on ikka objet ja saab täpsemad äärte encoderid


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
    return fmir_average, [fmir_average, fmir_average, fmir_average], fmir_average


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
    average = (buffer[0] + buffer[1] + buffer[2]) / 3

    # if new fmir reading is within the allowed deviation of average buffer value
    if average * 0.85 < fmir < average * 1.15:
        variables["fmir"] = fmir
    else:
        variables["fmir"] = variables["last_fmir"]

    variables["fmir_buffer"] = buffer  # read new buffer into dictionary
    return variables  # return the new dictionary


def sense(variables):
    """Get left, right wheel encoders and front middle IR sensor value, put to var dict and return the dict."""
    # read these things in.
    variables["last_left_speed"] = variables["left_speed"]
    variables["last_right_speed"] = variables["right_speed"]

    # put last values into respective dict keys
    variables["last_left_enc"] = variables["left_enc"]
    variables["last_right_enc"] = variables["right_enc"]

    # read new values in
    variables["left_enc"] = robot.get_left_wheel_encoder()
    variables["right_enc"] = robot.get_right_wheel_encoder()

    # calculate how much robot has turned during the tick in degrees, clockwise
    variables["turn_amount"] = robot.WHEEL_DIAMETER * ((variables["left_enc"] - variables["right_enc"]) - (
                variables["last_left_enc"] - variables["last_right_enc"])) / (2 * robot.AXIS_LENGTH)
    variables["abs_rota"] += variables["turn_amount"]

    variables = fmir_buffering(variables)  # updates "fmir", "last_fmir" and "fmir_buffer" dict keys

    variables["last_time"] = variables["current_time"]  # put last time into respective dict key
    variables["current_time"] = rospy.get_time()  # get new time

    # calculate distance the bot has traveled during the past cycle
    r_dist = math.pi * robot.WHEEL_DIAMETER * ((variables["right_enc"] - variables["last_right_enc"]) / 360)
    l_dist = math.pi * robot.WHEEL_DIAMETER * ((variables["left_enc"] - variables["last_left_enc"]) / 360)

    # save to variables
    variables["distance"] = (r_dist + l_dist) / 2
    variables["r_distance"] = r_dist
    variables["l_distance"] = l_dist

    return variables


def p_speed(variables, l_target_speed, r_target_speed=None):  # target speed should be in meters/second
    """
    Control left and right wheel speed with P control method.

    :param variables: dictionary with all the variables
    :param method: 1 for clockwise turning, 2 for moving straight, 3 for counterclockwise turning
    :param l_target_speed: left wheel speed to aim for
    :param r_target_speed: right wheel speed to aim for. If missing, assume it's same as left
    :return: dictionary with new left and right wheel speeds
    """
    # just a check to not do anything if speed is 0 or last speed was 0
    if (variables["right_speed"] == 0 and variables["left_speed"] == 0) or variables["p_ignore"]:
        variables["p_ignore"] = False
        return variables

    # check to skip this if last speeds were in the other direction
    if variables["last_right_speed"] / variables["right_speed"] <= 0 or variables["last_left_speed"] / variables["left_speed"] <= 0:
        return variables

    if variables["right_speed"] < 0 < variables["left_speed"]:
        method = 1
    elif variables["right_speed"] > 0 > variables["left_speed"]:
        method = 3
    else:
        method = 2

    # if no r target speed was given, then prolly not needed and make them equal
    if r_target_speed is None:
        r_target_speed = l_target_speed

    # time between this and last cycle
    time_diff = variables["current_time"] - variables["last_time"]

    # calculate wheel speeds based on v = s / t
    r_speed = variables["r_distance"] / time_diff
    l_speed = variables["l_distance"] / time_diff

    # get left wheel speed error
    if method == 1 or method == 2:  # clockwise turning or moving straight
        l_error = l_target_speed - l_speed
    elif method == 3:  # counterclockwise turning
        l_error = - l_target_speed - l_speed

    # get right wheel speed error, two separate versions because right wheel turns backwards during turning
    if method == 1:  # clockwise turning
        r_error = - r_target_speed - r_speed
    elif method == 2 or method == 3:   # moving straight or counterclockwise turning
        r_error = r_target_speed - r_speed

    # calculate new right and left wheel speeds by adding rounded value of GAIN constant times wheel speed error
    variables["right_speed"] = variables["right_speed"] + round(GAIN * r_error)
    variables["left_speed"] = variables["left_speed"] + round(GAIN * l_error)

    # return dictionary with variable dictionary with new speeds
    return variables


# TODO Kui leiab kaks objekti siis läheb nende kahe vahele
def decide(variables, median_list):
    # leave 2 closest object into the list
    if len(median_list) == 3:
        median_list.remove(sorted(median_list, key=lambda x: x[0])[-1])

    # angle between 2 objects
    angle_between_two_closest_objects = (median_list[1][1] - median_list[0][1]) % 360

    distance_between_two_closest_objects = sqrt(
        median_list[0][0] ** 2 + median_list[1][0] ** 2 - 2 * median_list[0][0] * median_list[1][0] * cos(
            angle_between_two_closest_objects))
    beta = asin((median_list[0][0] * sin(
        angle_between_two_closest_objects)) / distance_between_two_closest_objects)  # nurk mida vaja d arvutamiseks
    d = sqrt((distance_between_two_closest_objects / 2) ** 2 + median_list[1][0] ** 2 - 2 * (
            distance_between_two_closest_objects / 2) * median_list[1][0] * cos(beta))  # palju sõitma peab mediaanini

    # if robot can't fit through the 2 closest objects
    if distance_between_two_closest_objects < robot.AXIS_LENGTH + 0.05:
        # TODO "phase" = drive to other side of triangle
        return variables

    else:
        # TODO: make so this part is compatible with the turn_new thing
        # palju robot peab kõige parempoolsest pöörama et suund oleks mediaan radiaanides
        gamma = asin(((distance_between_two_closest_objects / 2) * sin(beta)) / d)
        gamma = (180 * gamma / pi) % 360
        distance = (pi * robot.AXIS_LENGTH) * (gamma / 360)
        degrees_to_spin = (360 * distance / variables["wheel circumference"])
        target = median_list[1][1] - degrees_to_spin

        variables["target_turn"] = target
        variables["distance_to_mid"] = d
        variables["phase"] = "turn"

        print(distance_between_two_closest_objects)
        print(beta)
        print(d)
        print(gamma)
        print(distance)
        print(degrees_to_spin)
        print(target)

        return variables


def turn_to_object(variables, median_list):
    """For second turning."""
    # TODO: make this part compatible with turn_new I guess
    variables["target_turn"] = median_list[1]
    variables["distance_to_mid"] = median_list[0] / 3
    variables["phase"] = "turn"

    print("turn_to_object")
    print(variables["target_turn"])
    print(variables["distance_to_mid"])

    return variables


def plan(variables):
    """Do all the planning in variable dict and then return it. Because Python."""
    object_count = variables["object_count"]
    on_object = variables["on_object"]
    # scanning phase
    if variables["phase"] == "scanning":

        # if initialisation
        if variables["init"]:
            variables["init"] = False
            variables["left_speed"], variables["right_speed"] = 12, -12
            variables["scan_start"] = variables["abs_rota"]
            print("speed 12 -12 (init scanning)")

        # if already in progress
        else:
            # last and current fmir sensor reading difference, used for object detection
            diff = variables["last_fmir"] - variables["fmir"]

            if variables["abs_rota"] - variables["scan_start"] > 360:
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["init"] = True
                variables["phase"] = "decide"
            # if diff is more than 20cm, then it most likely has detected an object
            elif on_object == 0:
                if diff > 0.20:
                    variables["counter"] = 0
                    variables["flag"] = True
                    variables["obj_distance"] = variables["fmir"]
                elif variables["flag"]:
                    variables["counter"] += 1
                    if variables["counter"] >= 4:
                        if variables["obj_distance"] + 0.1 > variables["fmir"]:
                            variables["on_object"] = 1
                            variables["object_count"] += 1
                            variables["phase"] = "zero_to_obj"
                            variables["next_phase"] = "scanning"
                            variables["init1"] = True
                        else:
                            variables["flag"] = False
            else:  # elif on_object == 1:
                variables["on_object"] = 0
                if variables["obj_count"] == 1:
                    variables["first_obj_encoder"] = variables["left_enc"]
                    variables["first_obj_deg"] = variables["abs_rota"]
                    variables["first_obj_distance"] = variables["zero_distance"]
                    variables["left_speed"], variables["right_speed"] = 12, -12
                elif variables["obj_count"] == 2:
                    variables["second_obj_encoder"] = variables["left_enc"]
                    variables["second_obj_deg"] = variables["abs_rota"]
                    variables["second_obj_distance"] = variables["zero_distance"]
                    variables["left_speed"], variables["right_speed"] = 12, -12
                elif variables["obj_count"] == 3:
                    variables["third_obj_encoder"] = variables["left_enc"]
                    variables["third_obj_deg"] = variables["abs_rota"]
                    variables["third_obj_distance"] = variables["zero_distance"]
                    variables["left_speed"], variables["right_speed"] = 0, 0
                    variables["phase"] = "decide"

    # TODO: implement usage
    # turns bot in desired direction desired amount. Wants "goal" in deg and "init1" and "next_phase"
    elif variables["phase"] == "turn_new":
        # initialisation
        if variables["init1"]:
            variables["init1"] = False

            # zero the turn amount
            variables["turn_progress"] = 0

            # if goal is positive aka clockwise
            if variables["goal"] > 0:
                # set speeds as so
                variables["left_speed"], variables["right_speed"] = 12, -12

            # if goal is negative aka counterclockwise
            if variables["goal"] < 0:
                # set speeds as so
                variables["left_speed"], variables["right_speed"] = -12, 12

        # if not initialisation
        else:
            # update how much bot has turned
            variables["turn_progress"] += variables["turn_amount"]

            # if has turned enough, stop bot and go to next phase
            if abs(variables["goal"]) - abs(variables["turn_progress"]) < 0:
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = variables["next_phase"]
                variables["init1"] = True

    # zeroing to object
    elif variables["phase"] == "zero_to_obj":
        # initialisation, needs "obj_distance" from external or takes fmir if it's smaller. Also "next_phase"
        if variables["init1"]:
            # cancel it
            variables["init1"] = False

            # put flag to false
            variables["flag"] = False

            # takes obj distance as fmir if it's shorter
            if variables["obj_distance"] > variables["fmir"]:
                variables["obj_distance"] = variables["fmir"]

            # restart counter and start turning clockwise until obj is lost. Also zero rota progress. Force exit
            variables["counter"] = 0
            variables["left_speed"], variables["right_speed"] = 12, -12
            variables["rota_progress"] = 0
            return variables

        # update rota progress
        variables["rota_progress"] += variables["turn_amount"]

        # if is moving clockwise to detect edge(lord)
        if variables["counter"] == 0:

            # if it has exited the object, save the degrees and +1 counter to go to next subphase
            if variables["fmir"] - 0.15 > variables["obj_distance"]:
                variables["r_edge"] = variables["rota_progress"]
                variables["counter"] = 1
                variables["left_speed"], variables["right_speed"] = -12, 12

        # if moving counterclockwise to detect edge
        elif variables["counter"] == 1:

            # if flag is not true, turn it true if it has gone back to object from passing it.
            if not variables["flag"]:
                if variables["fmir"] - 0.15 < variables["obj_distance"]:
                    variables["flag"] = True

            # if it has gone back to obj, start detecting for left edge
            # if has detected that it's off object again, save edge degrees and activate next subphase. Calculate goal
            elif variables["fmir"] - 0.15 > variables["obj_distance"]:
                variables["l_edge"] = variables["rota_progress"]
                variables["counter"] = 2
                variables["goal"] = (variables["l_edge"] + variables["r_edge"]) / 2

                # start turning clockwise towards object middle.
                variables["left_speed"], variables["right_speed"] = 12, -12

        # if rotating towards middle of object
        elif variables["counter"] == 2:

            # stop bot if has reached it and activate next phase according to init2
            if variables["goal"] < variables["rota_progress"]:
                variables["left_speed"], variables["right_speed"] = 0, 0

                variables["phase"] = variables["next_phase"]
                variables["zero_distance"] = variables["fmir"]
                variables["init1"] = True

    # decide, which object is which and what to do
    elif variables["phase"] == "decide":
        angle_between_second_and_first = abs(variables["first_obj_deg"] - variables["second_obj_deg"])
        first_obj = [variables["first_object_distance"], variables["first_object_deg"]]
        second_obj = [variables["second_object_distance"], variables["second_object_deg"]]
        if variables["obj_count"] == 2:
            median_list = [first_obj, second_obj]
            if angle_between_second_and_first > 180:
                median_list = [second_obj, first_obj]
            variables = decide(variables, median_list)
        elif variables["obj_count"] == 3:
            angle_between_third_and_second = abs(variables["second_obj_deg"] - variables["third_obj_deg"])
            third_obj = [variables["third_object_distance"], variables["third_object_deg"]]

            if angle_between_second_and_first > 180:  # second left, third middle, first right
                if variables["at_median"] == 0:
                    median_list = [second_obj, third_obj, first_obj]
                    variables = decide(variables, median_list)
                else:
                    variables = turn_to_object(variables, third_obj)

            elif angle_between_third_and_second > 180:  # third left, first middle, second right
                if variables["at_median"] == 0:
                    median_list = [third_obj, first_obj, second_obj]
                    variables = decide(variables, median_list)
                else:
                    variables = turn_to_object(variables, first_obj)

            else:  # first left, second, middle, third right
                if variables["at_median"] == 0:
                    median_list = [first_obj, second_obj, third_obj]
                    variables = decide(variables, median_list)
                else:
                    variables = turn_to_object(variables, second_obj)

    # turn to median, between the two closest objects
    elif variables["phase"] == "turn":
        if variables["turning"] == 0:
            variables["left_speed"], variables["right_speed"] = -12, 12
            variables["turning"] = 1
        else:
            if variables["left_enc"] < variables["target_turn"]:
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = "drive"

    # drive to median, between the two closest objects
    elif variables["phase"] == "drive":
        if variables["driving"] == 0:
            degrees_to_target = 360 * variables["distance_to_mid"] / (pi * robot.WHEEL_DIAMETER)
            variables["target_drive"] = variables["left_enc"] + degrees_to_target
            variables["left_speed"], variables["right_speed"] = 12, 12
            variables["driving"] = 1
        else:
            print(variables["left_enc"], variables["target_drive"])
            if variables["left_enc"] > variables["target_drive"]:
                print("Goes here! Noice!")
                variables["left_speed"], variables["right_speed"] = 0, 0
                if variables["at_median"] == 0:
                    variables["turning"] = 0
                    variables["driving"] = 0
                    variables["at_median"] = 1
                    variables["init"] = True
                    variables["phase"] = "scanning"
                else:
                    variables["phase"] = "end"

    # do p controller
    variables = p_speed(variables, 0.03)

    # return dictionary with all the new values
    return variables


def act(variables):
    """Change left and right wheel speed based on plan()."""
    robot.set_left_wheel_speed(variables["left_speed"])
    robot.set_right_wheel_speed(variables["right_speed"])


def main():
    """Create some variables used in the loop and then run the loop of sense, plan, act."""
    variables = dict()
    variables["p_ignore"] = True
    variables["left_speed"] = -1
    variables["right_speed"] = -1
    variables["init"] = True
    variables["init1"] = True
    variables["flag"] = False
    variables["abs_rota"] = 0
    variables["wheel circumference"] = robot.WHEEL_DIAMETER * pi
    variables["turn_back"] = 0  # if it should turn back after it has checked
    variables["has_checked"] = 0  # if it has checked the object
    variables["on_object_check"] = 0  # for checking, if it is on object or not
    variables["at_median"] = 0  # if it is at median or not, and if it should start the second turn
    variables["object_count"] = 0  # counts how many objects has it seen
    variables["on_object"] = 0  # if it is on object
    variables["turning"] = 0  # if it is turning already
    variables["driving"] = 0  # if it is driving
    variables["left_speed"] = 0  # for act, setting left wheel speed
    variables["right_speed"] = 0  # for act, setting right wheel speed
    variables["left_enc"] = robot.get_left_wheel_encoder()  # for sense
    variables["right_enc"] = robot.get_right_wheel_encoder()  # for sense
    variables["last_fmir"], variables["fmir_buffer"], variables["fmir"] = fmir_buffer_init()
    variables["phase"] = "scanning"  # phase it should be in
    variables["scan_progress"] = 0  # if it has started spinning and looking for a object
    variables["current_time"] = 0
    variables["last_time"] = 0
    variables["counter"] = 0
    variables["max_fmir"] = float("inf")
    variables["first_object_first_distance"] = float("inf")
    variables["second_object_first_distance"] = float("inf")
    variables["third_object_first_distance"] = float("inf")
    variables["first_object_second_encoder"] = float("inf")
    variables["second_object_second_encoder"] = float("inf")
    variables["third_object_second_encoder"] = float("inf")

    while True:
        variables = sense(variables)
        variables = plan(variables)
        act(variables)
        rospy.sleep(0.05)


if __name__ == "__main__":
    main()
