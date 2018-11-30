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

    # calculate distance the bot has traveled during the past cycle
    r_dist = math.pi * robot.WHEEL_DIAMETER * ((variables["right_enc"] - variables["last_right_enc"]) / 360)
    l_dist = math.pi * robot.WHEEL_DIAMETER * ((variables["left_enc"] - variables["last_left_enc"]) / 360)

    # save to variables
    variables["distance"] = (r_dist + l_dist) / 2
    variables["r_distance"] = r_dist
    variables["l_distance"] = l_dist

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

    # time between this and last cycle
    time_diff = variables["current_time"] - variables["last_time"]

    # calculate wheel speeds based on v = s / t
    r_speed = variables["r_distance"] / time_diff
    l_speed = variables["l_distance"] / time_diff

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

            # restart universal counter and initial detection subphase flag and closest wall
            variables["counter"] = 0
            variables["flag"] = False
            variables["closest_wall"] = float("inf")

            # if scan phase is being restarted from beginning
            if variables["init2"]:
                # restart scan progress to 0
                variables["scan_progress"] = 0

        # if it's not initialisation
        else:
            # add turn amount in degrees to scan progress
            variables["scan_progress"] += variables["turn_amount"]

            # if bot has turned multiplier amount of turns without detecting an object
            if variables["scan_progress"] > (360 * 1.5):
                # stop bot
                variables["left_speed"], variables["right_speed"] = 0, 0

                # calculate how much it has to turn to be 120 degrees from closest wall
                goal = variables["scan_progress"] - (variables["closest_wall_deg"] + 120)
                if abs(goal) > 180:
                    goal = goal % 360
                variables["goal"] = goal

                # mark next phase as turning and the phase after that as roam straight
                variables["next_phase"] = "roam_straight"
                variables["phase"] = "turn"
                variables["init1"] = True

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
                        # if object is still in the cone, within 10 cm
                        print("fmir:         ", variables["fmir"])
                        print("obj distance: ", variables["obj_distance"])
                        print("their diff:   ", abs(variables["fmir"] - variables["obj_distance"]))
                        if abs(variables["fmir"] - variables["obj_distance"]) < 0.15:
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
                    # if closest wall is further away than approximate distance from current fmirs
                    if variables["closest_wall"] > ((variables["fmir_buffer_avg"] + variables["fmir"] + variables["last_fmir"]) / 3):
                        # save the new closest wall
                        variables["closest_wall"] = ((variables["fmir_buffer_avg"] + variables["fmir"] + variables["last_fmir"]) / 3)

                        # save the bot aiming direction in degrees into a variables used for roam direction selection
                        variables["closest_wall_deg"] = variables["scan_progress"]

                # do p controller for speed correction
                variables = p_speed(variables, 1, 0.025)

    # turns bot in desired direction desired amount. Wants "goal" and "init1" and "next_phase"
    elif variables["phase"] == "turn":
        # initialisation
        if variables["init1"]:
            variables["init1"] = False
            # zero the turn amount
            variables["turn_progress"] = 0
            # if goal is positive aka clockwise
            if variables["goal"] > 0:
                # set speeds as so and p controller key also as so
                variables["p-key"] = 1
                variables["left_speed"], variables["right_speed"] = 12, -12

            # if goal is negative aka counterclockwise
            if variables["goal"] < 0:
                # set speeds and p controller as so
                variables["p-key"] = 3
                variables["left_speed"], variables["right_speed"] = -12, 12

        # if not initialisation
        else:
            # do p controlling and update how much bot has turned
            variables = p_speed(variables, variables["p-key"], 0.025)
            variables["turn_progress"] += variables["turn_amount"]

            # if has turned enough, stop bot and go to next phase
            if abs(variables["goal"]) - abs(variables["turn_progress"]) < 0:
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = variables["next_phase"]
                variables["init1"], variables["init2"] = True, True

    # moves straight until wall or distance and starts scanning again
    elif variables["phase"] == "roam_straight":
        # initialisation
        if variables["init1"]:
            variables["init1"] = False

            # reset how much it has to move straight and give it speeds to move straight
            variables["goal"] = 1.25
            variables["left_speed"], variables["right_speed"] = 15, 15

        # if not initialisation
        else:
            # p controller
            variables = p_speed(variables, 2, 0.07)

            # substract traveled distance from goal
            variables["goal"] -= variables["distance"]

            # if goal has been reached or wall or some object is within 60 cm
            if variables["goal"] < 0 or (variables["fmir"] < 0.6 and variables["last_fmir"] < 0.6):
                # stop bot and start scanning
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = "scanning"
                variables["init1"], variables["init2"] = True, True

    # zeroing to object at long-ish distance (like up to 80cm... Hopefully)
    elif variables["phase"] == "zero_to_obj":  # init2 false: next is move to obj. True: next is blind to obj
        # initialisation, needs "obj_distance" from external or takes fmir if it's smaller
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
            # p control this, thanks
            variables = p_speed(variables, 1, 0.02)

            print(variables["fmir"], variables["fmir_buffer"])

            # if it has exited the object, save the degrees and +1 counter to go to next subphase
            if variables["fmir"] - 0.15 > variables["obj_distance"]:
                print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH STARTING TO TURN LEFT NOW AAAAAAAAAH")
                variables["r_edge"] = variables["rota_progress"]
                variables["counter"] = 1
                variables["left_speed"], variables["right_speed"] = -12, 12

        # if moving counterclockwise to detect edge
        elif variables["counter"] == 1:
            # p controller for counterclockwise turning
            variables = p_speed(variables, 3, 0.02)

            print(variables["fmir"], variables["fmir_buffer"])

            # if flag is not true, turn it true if it has gone back to object from passing it.
            if not variables["flag"]:
                if variables["fmir"] - 0.15 < variables["obj_distance"]:
                    variables["flag"] = True

            # if it has gone back to obj, start detecting for left edge
            # if has detected that it's off object again, save edge degrees and activate next subphase. Calculate goal
            elif variables["fmir"] - 0.15 > variables["obj_distance"]:
                print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOH STARTING TO TORN BACK TO CENTER OOOOOOH")
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

    # move towards object until within 20cm
    elif variables["phase"] == "move_to_obj":
        # initialisation
        print(variables["l3"], variables["l2"], variables["l1"], variables["r1"], variables["r2"], variables["r3"])
        if variables["init1"]:
            variables["init1"] = False
            variables["p_ignore"] = True

            # get max_distance, used for cases where object is lost
            variables["max_distance"] = variables["fmir"] + 0.1

            # start moving straight
            variables["left_speed"], variables["right_speed"] = 16, 16

        # if it's not initialisation
        else:
            # if has been on line, save it into a flag
            if variables["l3"] < 500 or variables["l2"] < 500 or variables["l1"] < 500 or variables["r1"] < 500 or variables["r2"] < 500 or variables["r3"] < 500:
                variables["line"] = True

            # if current fmir is at least 10cm shorter than max allowed fmir
            if variables["max_distance"] > variables["fmir"] + 0.1:
                # then put current fmir plus 10 cm as max distance
                variables["max_distance"] = variables["fmir"] + 0.1

            # if object has been lost, start scanning again
            if variables["max_distance"] < variables["fmir"]:
                variables["init1"], variables["init2"] = True, True
                variables["phase"] = "scanning"
                variables["left_speed"], variables["right_speed"] = 0, 0

            # if it still has the object
            else:
                # if bot is within 20cm of the object, stop bot and start zeroing
                if variables["fmir"] < 0.20:
                    variables["left_speed"], variables["right_speed"] = 0, 0
                    variables["phase"] = "zero_to_obj"
                    variables["init1"], variables["init2"] = True, True

        # do p controlling
        variables = p_speed(variables, 2, 0.06)

    # blindly move towards object to a certain distance
    elif variables["phase"] == "blind_to_obj":
        # initialisation
        if variables["init1"]:
            variables["init1"] = False
            # put grabber to position, no senseplanact because this is 3 lines of code instead of 30
            robot.close_grabber(0)
            robot.set_grabber_height(0)
            rospy.sleep(2)

            # set goal as current distance from object (bad key name, but less keys in dict)
            variables["goal"] = variables["fmir"]
            # and start moving straight
            variables["left_speed"], variables["right_speed"] = 13, 13

        # if not initialisation
        else:
            # if has been on line, save it into a flag
            if variables["l3"] < 500 or variables["l2"] < 500 or variables["l1"] < 500 or variables["r1"] < 500 or variables["r2"] < 500 or variables["r3"] < 500:
                variables["line"] = True

            # do p controlling
            variables = p_speed(variables, 2, 0.03)

            # substract moved distance during tick from the "goal" aka approximate distance from object
            variables["goal"] -= variables["distance"]

            # if robot is approximately 7 cm away from object
            if variables["goal"] < 0.07:
                # stop robot
                variables["left_speed"], variables["right_speed"] = 0, 0
                act(variables)

                # pick up object, 4 lines of code instead of 20!
                robot.close_grabber(100)
                rospy.sleep(1)
                robot.set_grabber_height(100)
                rospy.sleep(3)

                # if line was behind, turn 180 degrees and start moving straight
                if variables["line"]:
                    variables["phase"] = "turn"
                    variables["next_phase"] = "move_until_wall"
                    variables["goal"] = 180
                    variables["init1"], variables["init2"] = True, False

                # if no line was behind, just start moving until wall
                else:
                    variables["phase"] = "move_until_wall"
                    variables["init1"], variables["init2"] = True, False

    # move straight until wall while detecting for a line
    elif variables["phase"] == "move_until_wall":
        # initialisation
        if variables["init1"]:
            variables["init1"] = False
            variables["flag"] = False

            # start moving
            variables["left_speed"], variables["right_speed"] = 15, 15

        # if not initialisation
        else:
            # if has detected a line, stop bot and start line following
            if variables["l3"] < 500 or variables["l2"] < 500 or variables["l1"] < 500 or variables["r1"] < 500 or variables["r2"] < 500 or variables["r3"] < 500:
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = "line_follow"
                variables["init1"], variables["init2"] = True, True
                return variables

            # do some p controlling
            variables = p_speed(variables, 2, 0.07)

            # if last fmir is closer than 40 cm and fmir is closer
            if variables["fmir"] <= variables["last_fmir"] < 0.4 and not variables["flag"]:
                # turn flag to true and start counter
                variables["flag"] = True
                variables["counter"] = 1
                variables["wall_distance"] = variables["fmir"]

            # if flag is true'd
            elif variables["flag"]:
                # increment counter
                variables["counter"] += 1

                # if counter has reached that and still is that case, then can start sweeping
                if variables["counter"] >= 10:
                    if variables["fmir"] < variables["last_fmir"] < variables["wall_distance"]:
                        variables["left_speed"], variables["right_speed"] = 0, 0
                        variables["init1"], variables["init2"] = True, True
                        variables["phase"] = "zero_to_wall"

                    # if that is not the case, turn flag to false
                    else:
                        variables["flag"] = False

    # zero to wall?
    elif variables["phase"] == "zero_to_wall":
        # initialisation
        if variables["init1"]:
            # make so next time it isn't triggered
            variables["init1"] = False

            # start turning clockwise
            variables["left_speed"], variables["right_speed"] = 12, -12

            # restart universal counter and initial detection subphase flag and closest wall
            variables["counter"] = 0
            variables["closest_wall"] = float("inf")

            # restart scan progress to 0
            variables["scan_progress"] = 0

        # if it's not initialisation
        else:
            # add turn amount in degrees to scan progress
            variables["scan_progress"] += variables["turn_amount"]
            print(variables["closest_wall"])
            # if bot has turned multiplier amount of turns
            if variables["scan_progress"] > (360 * 1):
                # stop bot
                variables["left_speed"], variables["right_speed"] = 0, 0

                # calculate how much it has to turn to be 80 degrees from closest wall
                goal = variables["scan_progress"] - (variables["closest_wall_deg"] + 80)
                if abs(goal) > 180:
                    goal = goal % 360
                variables["goal"] = goal

                # mark next phase as sweep
                variables["next_phase"] = "sweep"
                variables["phase"] = "turn"
                variables["init1"] = True
                variables["init3"] = True

            # if scanning has not reached the limit
            else:
                # if closest wall is further away than approximate distance from current fmirs
                if variables["closest_wall"] > ((variables["fmir_buffer_avg"] + variables["fmir"] + variables["last_fmir"]) / 3):
                    # save the new closest wall
                    variables["closest_wall"] = ((variables["fmir_buffer_avg"] + variables["fmir"] + variables["last_fmir"]) / 3)

                    # save the bot aiming direction in degrees into a variables used for roam direction selection
                    variables["closest_wall_deg"] = variables["scan_progress"]

                # do p controller for speed correction
                variables = p_speed(variables, 1, 0.025)

    # if is sweeping time
    elif variables["phase"] == "sweep":
        # initialisation
        if variables["init1"]:
            variables["init1"] = False
            variables["flag"] = False

            # set goal for moving straight
            variables["goal"] = 0.2

            # start moving
            variables["left_speed"], variables["right_speed"] = 15, 15

            # if init3 is true, create some bools
            if variables["init3"]:
                variables["init3"] = False
                variables["long"] = True
                variables["right"] = True

        # if not initialisation
        else:
            # if has detected a line, stop bot and start line following
            if variables["l3"] < 500 or variables["l2"] < 500 or variables["l1"] < 500 or variables["r1"] < 500 or variables["r2"] < 500 or variables["r3"] < 500:
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = "line_follow"
                variables["init1"], variables["init2"] = True, True
                return variables

            # do some p controlling
            variables = p_speed(variables, 2, 0.09)

            # if it's doing the long sweep
            if variables["long"]:
                # if last fmir is closer than 40 cm and fmir is closer
                if variables["fmir"] <= variables["last_fmir"] < 0.4 and not variables["flag"]:
                    # turn flag to true and start counter
                    variables["flag"] = True
                    variables["counter"] = 1
                    variables["wall_distance"] = variables["fmir"]

                # if flag is true'd
                elif variables["flag"]:
                    # increment counter
                    variables["counter"] += 1

                    # if counter has reached that and still is that case, then can start sweeping
                    if variables["counter"] >= 10:
                        if variables["fmir"] < variables["last_fmir"] < variables["wall_distance"]:
                            variables["left_speed"], variables["right_speed"] = 0, 0
                            variables["init1"], variables["init2"] = True, True
                            variables["phase"] = "turn"
                            variables["next_phase"] = "sweep"

                            # if next turn is right
                            if variables["right"]:
                                variables["goal"] = 80
                            else:
                                variables["goal"] = -80

                            # make long false as next one will be short movement straight
                            variables["long"] = False

                        # if that is not the case, turn flag to false
                        else:
                            variables["flag"] = False

            # if it's short move
            else:
                # decrement goal by distance traveled
                variables["goal"] -= variables["distance"]

                # TODO: add some check for wall
                # if it has reached the distance
                if variables["goal"] < 0:
                    # stop bot and turn
                    variables["left_speed"], variables["right_speed"] = 0, 0
                    variables["init1"], variables["init2"] = True, True
                    variables["phase"] = "turn"
                    variables["next_phase"] = "sweep"

                    # if next turn is right
                    if variables["right"]:
                        variables["goal"] = 80
                        variables["right"] = False
                    else:
                        variables["goal"] = -80
                        variables["right"] = True

                    # make long true as next one will be long
                    variables["long"] = True

    elif variables["phase"] == "line_follow":
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
    variables["line"] = False

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