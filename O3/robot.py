"""O2."""
from PiBot import PiBot
import rospy
from math import cos, sin, sqrt, asin, pi, degrees, radians
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
    return fmir_average, [fmir_average, fmir_average, fmir_average], fmir_average


def fmir_buffering(variables):
    """
    See if fmir value is correct. If not, return last fmir.

    :param variables: variables dict
    :return: variables dict with new fmir and buffer
    """
    buffer = variables["fmir_buffer"]  # read buffer into var for easier writing, could remove this though
    fmir = robot.get_front_middle_ir()  # read fmir into var
    if fmir > 1:
        fmir = 1
    variables["last_fmir"] = variables["fmir"]  # put last allowed fmir value into "last_fmir" dict key
    variables["past_fmirs"].pop(0)
    variables["past_fmirs"].append(variables["last_fmir"])

    # remove oldest and add new fmir reading
    buffer.pop(0)
    buffer.append(fmir)

    # get average fmir value from buffer
    average = (buffer[0] + buffer[1] + buffer[2]) / 3

    # if new fmir reading is within the allowed deviation of average buffer value
    if average * 0.9 < fmir < average * 1.1:
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
    variables["right_speed"] = int(variables["right_speed"] + round(GAIN * r_error))
    variables["left_speed"] = int(variables["left_speed"] + round(GAIN * l_error))

    # return dictionary with variable dictionary with new speeds
    return variables


def decide(variables, median_list):
    """Decide what to do."""
    # leave 2 closest object into the list
    print(median_list)
    if len(median_list) == 3:
        median_list.remove(sorted(median_list, key=lambda x: x[0])[-1])
    print(median_list)
    # angle between 2 objects
    angle_between_two_closest_objects = (median_list[1][1] - median_list[0][1]) % 360  # kraadides
    distance_between_two_closest_objects = sqrt(
        median_list[0][0] ** 2 + median_list[1][0] ** 2 - 2 * median_list[0][0] * median_list[1][0] * cos(
            radians(angle_between_two_closest_objects)))  # meetrites
    beta = asin((median_list[0][0] * sin(
        radians(angle_between_two_closest_objects))) / distance_between_two_closest_objects)  # nurk mida vaja d arvutamiseks, radiaanides
    d = sqrt((distance_between_two_closest_objects / 2) ** 2 + median_list[1][0] ** 2 - 2 * (
            distance_between_two_closest_objects / 2) * median_list[1][0] * cos(beta))  # palju sõitma peab mediaanini meetrites

    # if robot can't fit through the 2 closest objects
    if distance_between_two_closest_objects < robot.AXIS_LENGTH:
        print("AAAAAAAAAAAAAH PAY ATTENTION KID AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        print("distance between closest and robo axis", distance_between_two_closest_objects, robot.AXIS_LENGTH)
        # palju robot peab pöörama, et oleks 90 kraadise nurga all esimese kahe objektiga
        # distance_from_perpendicular on kaugus ristumise punktist
        distance_from_perpendicular = (sin(beta) * median_list[1][0]) / sin(1.5708)

        # length of the arc what robot needs to drive, outside wheel, 55 degrees to new point
        distance_to_other_side = 2.35 * distance_from_perpendicular
        print(distance_from_perpendicular, sin(beta), median_list[1][0], sin(1.5708))

        # multiplier on suhe mida korrutada sisemise ratta speedi sellega (p-controlleri target speed)
        variables["multiplier"] = (distance_from_perpendicular + 0.185) / (distance_from_perpendicular + 0.185 + robot.AXIS_LENGTH)
        print("and multiplier is: ", variables["multiplier"])
        variables["dgoal"] = 100  # 120 kraadi see kaare asi

        abs_goal = median_list[1][1] - (180-(90 + degrees(beta))) + 90  # ?
        variables["goal"] = abs_goal - variables["abs_rota"]
        variables["distance_to_new_side"] = distance_to_other_side
        variables["phase"] = "turn_new"
        variables["next_phase"] = "moving_forw_arc"
        return variables
    else:
        # median list esimene on vist vasakpoolne ja teine parempoolne? jah
        # median_list = [distance, degrees]
        if median_list[0][0] > median_list[1][0]:
            variables["isitmirrored"] = True
        else:
            variables["isitmirrored"] = False
        obj1, obj2 = sorted(median_list, key=lambda x: x[0])  # obj1 peaks olema lähem nii, obj2 kaugem
        degdiff = abs(obj1[1] - obj2[1])
        objconnector = sqrt(obj1[0] ** 2 + obj2[0] ** 2 - 2 * obj1[0] * obj2[0] * cos(radians(degdiff)))
        botobj1obj2angle = degrees(asin(sin(radians(degdiff)) * obj2[0] / objconnector))
        otherangle = 180 - botobj1obj2angle
        variables["secondstraightmove"] = sin(radians(otherangle)) * obj1[0] - 0.2
        variables["firststraightmove"] = objconnector / 2 + cos(radians(otherangle)) * obj1[0]
        if not variables["isitmirrored"]:
            absgoal = obj1[1] + otherangle
        else:
            absgoal = obj1[1] - otherangle
        variables["goal"] = absgoal - variables["abs_rota"]
        variables["init1"] = True
        variables["phase"] = "turn_new"
        variables["next_phase"] = "pre-drive"
        return variables



def turn_to_object(variables, median_list):
    """For second turning."""
    # median_list = [distance, degrees]
    variables["distance_to_mid"] = median_list[0] / 3
    variables["goal"] = median_list[1] - variables["abs_rota"]
    variables["phase"] = "turn_new"
    variables["next_phase"] = "drive"  # eee ma mõtlesin seda kohta
    return variables


def plan(variables):
    """Do all the planning in variable dict and then return it."""
    object_count = variables["obj_count"]  # ?
    on_object = variables["on_obj"]
    # scanning phase
    if variables["phase"] == "scanning":

        # if initialisation
        if variables["init"]:
            variables["init"] = False
            variables["left_speed"], variables["right_speed"] = 12, -12
            variables["scan_start"] = variables["abs_rota"]
            variables["prev_diff"] = 0
            variables["preprev_diff"] = 0

        # if already in progress
        else:
            # last and current fmir sensor reading difference, used for object detection
            diff = variables["last_fmir"] - variables["fmir"]
            print(diff)
            print(variables["fmir_buffer"])
            if variables["abs_rota"] - variables["scan_start"] > 360:
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["init"] = True
                variables["phase"] = "decide"
            # if diff is more than 20cm, then it most likely has detected an object
            elif on_object == 0:
                if variables["flag"]:
                    variables["counter"] += 1
                    if variables["obj_distance"] + 0.10 > variables["fmir"] or variables["side"]:
                        if variables["counter"] >= 5 or variables["side"]:
                            variables["on_obj"] = 1
                            variables["obj_count"] += 1
                            variables["phase"] = "zero_to_obj"
                            variables["next_phase"] = "scanning"
                            variables["init1"] = True
                            variables["prev_diff"] = 0
                            variables["preprev_diff"] = 0
                    else:
                        variables["flag"] = False
                    variables["prev_diff"] = diff
                    variables["preprev_diff"] = variables["prev_diff"]

                elif diff + variables["prev_diff"] + variables["preprev_diff"] > 0.25 and variables["fmir"] < 0.80:
                    variables["side"] = False  # ehk detectis vasakult poolt peale minnes
                    variables["counter"] = 0
                    variables["flag"] = True
                    variables["obj_distance"] = variables["fmir"]
                elif diff + variables["prev_diff"] + variables["preprev_diff"] < -0.25 and variables["past_fmirs"][-4] < 0.80:
                    print(variables["abs_rota"] + 6, variables["r_absolute"])
                    if variables["abs_rota"] - 6 > variables["r_absolute"]:
                        variables["side"] = True
                        variables["flag"] = True
                        variables["obj_distance"] = variables["past_fmirs"][-4]
                        for i in range(6):
                            if variables["past_fmirs"][i] > variables["obj_distance"] + 0.1 or variables["past_fmirs"][i] < variables["obj_distance"] - 0.1:
                                variables["flag"] = False
                                variables["side"] = False

            else:  # elif on_object == 1:
                variables["on_obj"] = 0
                if object_count == 1:
                    variables["first_obj_deg"] = variables["abs_rota"]
                    variables["first_obj_distance"] = variables["zero_distance"]
                    variables["left_speed"], variables["right_speed"] = 12, -12
                    print("1")
                elif object_count == 2:
                    variables["second_obj_deg"] = variables["abs_rota"]
                    variables["second_obj_distance"] = variables["zero_distance"]
                    variables["left_speed"], variables["right_speed"] = 12, -12
                    if variables["lastissame"]:
                        variables["lastissame"] = False
                        variables["first_obj_deg"] = variables["second_obj_deg"] - variables["degsforsame"]
                        variables["first_obj_distance"] = variables["zero_distance"]
                        variables["second_obj_deg"] += variables["degsforsame"]

                    print("2")
                elif object_count == 3:
                    variables["third_obj_deg"] = variables["abs_rota"]
                    variables["third_obj_distance"] = variables["zero_distance"]
                    variables["left_speed"], variables["right_speed"] = 0, 0
                    variables["phase"] = "decide"
                    if variables["lastissame"]:
                        variables["lastissame"] = False
                        variables["second_obj_deg"] = variables["third_obj_deg"] - variables["degsforsame"]
                        variables["second_obj_distance"] = variables["zero_distance"]
                        variables["third_obj_deg"] += variables["degsforsame"]
                    print("3")

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
            print(abs(variables["goal"]) - abs(variables["turn_progress"]))

            # if has turned enough, stop bot and go to next phase
            if abs(variables["goal"]) - abs(variables["turn_progress"]) <= 0:
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

            variables["rota_progress"] = 0

            if not variables["side"]:  # takes obj distance as fmir if it's shorter
                if variables["obj_distance"] > variables["fmir"]:
                    variables["obj_distance"] = variables["fmir"]

                # restart counter and start turning clockwise until obj is lost. Also zero rota progress. Force exit
                variables["counter"] = 0
                variables["left_speed"], variables["right_speed"] = 12, -12
            else:
                variables["r_edge"] = 0
                variables["r_absolute"] = variables["abs_rota"]
                variables["counter"] = 1
                variables["left_speed"], variables["right_speed"] = -12, 12

            return variables

        # update rota progress
        variables["rota_progress"] += variables["turn_amount"]
        print(variables["fmir"])

        # if is moving clockwise to detect edge(lord)
        if variables["counter"] == 0:

            # if it has exited the object, save the degrees and +1 counter to go to next subphase
            if variables["fmir"] - 0.10 > variables["obj_distance"] or variables["fmir"] + 0.10 < variables["obj_distance"]:
                variables["r_edge"] = variables["rota_progress"]
                variables["r_absolute"] = variables["abs_rota"]
                variables["counter"] = 1
                variables["left_speed"], variables["right_speed"] = -12, 12

        # if moving counterclockwise to detect edge
        elif variables["counter"] == 1:

            # if flag is not true, turn it true if it has gone back to object from passing it.
            if not variables["flag"]:
                if variables["fmir"] - 0.10 < variables["obj_distance"]:
                    variables["flag"] = True

            # if it has gone back to obj, start detecting for left edge
            # if has detected that it's off object again, save edge degrees and activate next subphase. Calculate goal
            elif variables["fmir"] - 0.10 > variables["obj_distance"] or variables["fmir"] + 0.10 < variables["obj_distance"]:
                variables["l_edge"] = variables["rota_progress"]
                variables["counter"] = 2
                variables["goal"] = (variables["l_edge"] + variables["r_edge"]) / 2

                # start turning clockwise towards object middle.
                variables["left_speed"], variables["right_speed"] = 12, -12
                print("obj distance: ", variables["obj_distance"])
                print("degrees: ", variables["l_edge"] - variables["r_edge"])
                print("Width of the object:", variables["obj_distance"] * sqrt(2 * (1 - cos(radians(abs(variables["l_edge"] - variables["r_edge"]) - 30)))))
                if variables["obj_distance"] * sqrt(2 * (1 - cos(radians(abs(variables["l_edge"] - variables["r_edge"]) - 30)))) > 0.125:
                    print("AAAAAAAAH I detected a wide object!")
                    if variables["obj_count"] == 1:
                        variables["obj_count"] = 2
                    elif variables["obj_count"] == 2:
                        variables["obj_count"] = 3
                    variables["lastissame"] = True
                    variables["degsforsame"] = abs(abs(variables["goal"]) - 20)  # diff between the 2 objects or something

        # if rotating towards middle of object
        elif variables["counter"] == 2:

            # stop bot if has reached it and activate next phase according to init2
            if variables["goal"] < variables["rota_progress"]:
                variables["left_speed"], variables["right_speed"] = 0, 0

                variables["phase"] = variables["next_phase"]
                variables["zero_distance"] = variables["fmir"]
                variables["init1"] = True
                variables["flag"] = False    # Oli puudu

    # decide, which object is which and what to do
    elif variables["phase"] == "decide":
        angle_between_second_and_first = abs(variables["first_obj_deg"] - variables["second_obj_deg"])
        first_obj = [variables["first_obj_distance"], variables["first_obj_deg"]]
        second_obj = [variables["second_obj_distance"], variables["second_obj_deg"]]
        if variables["obj_count"] == 2:
            print(angle_between_second_and_first)
            median_list = [first_obj, second_obj]
            if angle_between_second_and_first > 150:
                median_list = [second_obj, first_obj]
            variables = decide(variables, median_list)
            print(type(variables["at_median"]))
            variables["at_median"] = 0
            variables["badscancount"] += 1
            if variables["badscancount"] > 4:
                variables["at_median"] = 1
        elif variables["obj_count"] == 3:
            angle_between_third_and_second = abs(variables["second_obj_deg"] - variables["third_obj_deg"])
            third_obj = [variables["third_obj_distance"], variables["third_obj_deg"]]
            print(angle_between_second_and_first)
            print(angle_between_third_and_second)

            if angle_between_second_and_first > 150:  # second left, third middle, first right
                if variables["at_median"] == 0:
                    median_list = [second_obj, third_obj, first_obj]
                    variables = decide(variables, median_list)
                else:
                    variables = turn_to_object(variables, third_obj)

            elif angle_between_third_and_second > 150:  # third left, first middle, second right
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

    # drive to median, between the two closest objects
    elif variables["phase"] == "drive":
        if variables["driving"] == 0:
            degrees_to_target = 360 * variables["distance_to_mid"] / (pi * robot.WHEEL_DIAMETER)
            variables["target_drive"] = variables["left_enc"] + degrees_to_target
            variables["left_speed"], variables["right_speed"] = 12, 12
            variables["driving"] = 1
        else:
            if variables["left_enc"] > variables["target_drive"]:
                variables["left_speed"], variables["right_speed"] = 0, 0
                if variables["at_median"] == 0:
                    variables["driving"] = 0
                    variables["at_median"] = 1
                    variables["init"] = True
                    variables["obj_count"] = 0
                    variables["phase"] = "scanning"
                else:
                    variables["phase"] = "end"

    # universal driving
    elif variables["phase"] == "moving_forw_arc":
        if variables["init"]:
            variables["init"] = False
            variables["move_progress"] = 0
            variables["left_speed"], variables["right_speed"] = round(12 + 5 * variables["multiplier"]), 18
        else:
            variables = p_speed(variables, 0.08 * variables["multiplier"], 0.08)
            variables["p_ignore"] = True
            variables["move_progress"] += variables["turn_amount"]
            if abs(variables["move_progress"]) > variables["dgoal"]:
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["init"] = True
                variables["phase"] = "scanning"
                variables["obj_count"] = 0

    elif variables["phase"] == "pre-drive":
        variables["goal"] = variables["firststraightmove"]
        variables["phase"] = "drive_straight"
        variables["next_phase"] = "pre-90deg"

    elif variables["phase"] == "pre-90deg":
        goal = -82
        if variables["isitmirrored"]:
            goal = 82
        variables["goal"] = goal
        variables["phase"] = "turn_new"
        variables["next_phase"] = "pre-closedrive"

    elif variables["phase"] == "pre-closedrive":
        variables["goal"] = variables["secondstraightmove"]
        variables["phase"] = "drive_straight"
        variables["next_phase"] = "pre-smallscan"

    elif variables["phase"] == "drive_straight":
        # initialisation
        if variables["init1"]:
            variables["init1"] = False

            # reset how much it has to move straight and give it speeds to move straight
            variables["left_speed"], variables["right_speed"] = 15, 15

        # if not initialisation
        else:
            # substract traveled distance from goal
            variables["goal"] -= variables["distance"]

            # if goal has been reached or wall or some object is within 60 cm
            if variables["goal"] < 0:
                # stop bot and start scanning
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = variables["next_phase"]
                variables["init1"] = True

    elif variables["phase"] == "pre-smallscan":
        variables["goal"] = -82
        variables["phase"] = "turn_new"
        variables["next_phase"] = "smallscan"
        variables["init1"] = True  # just in case
        variables["initsscan"] = True

    elif variables["phase"] == "smallscan":
        print("doing smallscan")
        # if initialisation
        if variables["initsscan"]:
            variables["initsscan"] = False
            variables["left_speed"], variables["right_speed"] = 12, -12
            variables["scan_start"] = variables["abs_rota"]
            variables["prev_diff"] = 0
            variables["preprev_diff"] = 0
            variables["obj_count"] = 0

            # if already in progress
        else:
            # last and current fmir sensor reading difference, used for object detection
            diff = variables["last_fmir"] - variables["fmir"]
            print(diff)
            print(variables["fmir_buffer"])
            print(variables["abs_rota"] - variables["scan_start"])
            if variables["abs_rota"] - variables["scan_start"] > 180:
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["init"] = True
                variables["phase"] = "smalldecide"
            # if diff is more than 20cm, then it most likely has detected an object
            elif on_object == 0:
                if variables["flag"]:
                    variables["counter"] += 1
                    if variables["obj_distance"] + 0.10 > variables["fmir"] or variables["side"]:
                        if variables["counter"] >= 5 or variables["side"]:
                            variables["on_obj"] = 1
                            variables["obj_count"] += 1
                            variables["phase"] = "zero_to_obj"
                            variables["next_phase"] = "smallscan"
                            variables["init1"] = True
                            variables["prev_diff"] = 0
                            variables["preprev_diff"] = 0
                    else:
                        variables["flag"] = False
                    variables["prev_diff"] = diff
                    variables["preprev_diff"] = variables["prev_diff"]

                elif diff + variables["prev_diff"] + variables["preprev_diff"] > 0.25 and variables["fmir"] < 0.85:
                    variables["side"] = False  # ehk detectis vasakult poolt peale minnes
                    variables["counter"] = 0
                    variables["flag"] = True
                    variables["obj_distance"] = variables["fmir"]
                elif diff + variables["prev_diff"] + variables["preprev_diff"] < -0.25 and variables["past_fmirs"][-4] < 0.85:
                    print(variables["abs_rota"] + 6, variables["r_absolute"])
                    if variables["abs_rota"] - 6 > variables["r_absolute"]:
                        variables["side"] = True
                        variables["flag"] = True
                        variables["obj_distance"] = variables["past_fmirs"][-4]
                        for i in range(6):
                            if variables["past_fmirs"][i] > variables["obj_distance"] + 0.1 or variables["past_fmirs"][i] < variables["obj_distance"] - 0.1:
                                variables["flag"] = False
                                variables["side"] = False

            else:  # elif on_object == 1:
                variables["on_obj"] = 0
                if object_count == 1:
                    variables["first_obj_deg"] = variables["abs_rota"]
                    variables["first_obj_distance"] = variables["zero_distance"]
                    variables["left_speed"], variables["right_speed"] = 12, -12
                    print("1")
                elif object_count == 2:
                    variables["second_obj_deg"] = variables["abs_rota"]
                    variables["second_obj_distance"] = variables["zero_distance"]
                    variables["left_speed"], variables["right_speed"] = 12, -12
                    if variables["lastissame"]:
                        variables["lastissame"] = False
                        variables["first_obj_deg"] = variables["second_obj_deg"] - variables["degsforsame"]
                        variables["first_obj_distance"] = variables["zero_distance"]
                        variables["second_obj_deg"] += variables["degsforsame"]

                    print("2")
                elif object_count == 3:
                    variables["third_obj_deg"] = variables["abs_rota"]
                    variables["third_obj_distance"] = variables["zero_distance"]
                    variables["left_speed"], variables["right_speed"] = 0, 0
                    variables["phase"] = "smalldecide"
                    if variables["lastissame"]:
                        variables["lastissame"] = False
                        variables["second_obj_deg"] = variables["third_obj_deg"] - variables["degsforsame"]
                        variables["second_obj_distance"] = variables["zero_distance"]
                        variables["third_obj_deg"] += variables["degsforsame"]
                    print("3")

    elif variables["phase"] == "smalldecide":
        if object_count == 3:
            variables["second_obj_deg"] = variables["third_object_deg"]
            variables["yolotravel"] = min(variables["first_obj_distance"], variables["second_obj_distance"], variables["third_obj_distance"]) + 0.14
        else:
            variables["yolotravel"] = min(variables["first_obj_distance"], variables["second_obj_distance"]) + 0.14
        print(variables["yolotravel"])
        variables["goal"] = (variables["first_obj_deg"] + variables["second_obj_deg"]) / 2 - variables["abs_rota"]
        print("I calculated the goal, yo")
        variables["phase"] = "turn_new"
        variables["next_phase"] = "preyolodrive"
        variables["init1"] = True

    elif variables["phase"] == "preyolodrive":
        variables["goal"] = variables["yolotravel"]
        variables["phase"] = "drive_straight"
        variables["init1"] = True
        variables["next_phase"] = "oof"

    else:
        print("tomato")


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
    variables["badscancount"] = 0
    variables["multiplier"] = 1
    variables["wheel circumference"] = robot.WHEEL_DIAMETER * pi
    variables["turn_back"] = 0  # if it should turn back after it has checked
    variables["has_checked"] = 0  # if it has checked the object
    variables["on_obj_check"] = 0  # for checking, if it is on object or not
    variables["at_median"] = 0  # if it is at median or not, and if it should start the second turn
    variables["obj_count"] = 0  # counts how many objects has it seen
    variables["on_obj"] = 0  # if it is on object
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
    variables["lastissame"] = False
    variables["r_absolute"] = 0
    variables["past_fmirs"] = [variables["fmir"], variables["fmir"], variables["fmir"], variables["fmir"], variables["fmir"], variables["fmir"], variables["fmir"], variables["fmir"], variables["fmir"], variables["fmir"]]

    while True:
        variables = sense(variables)
        variables = plan(variables)
        act(variables)
        rospy.sleep(0.05)


if __name__ == "__main__":
    main()
