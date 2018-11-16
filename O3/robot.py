"""O2."""
from PiBot import PiBot
import rospy
from math import cos, sin, sqrt, asin, pi
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


def p_speed(variables, method, l_target_speed, r_target_speed=None):  # target speed should be in meters/second
    """
    Control left and right wheel speed with P control method.

    :param variables: dictionary with all the variables
    :param method: 1 for clockwise turning, 2 for moving straight, 3 for counterclockwise turning
    :param l_target_speed: left wheel speed to aim for
    :param r_target_speed: right wheel speed to aim for. If missing, assume it's same as left
    :return: dictionary with new left and right wheel speeds
    """
    # if no r target speed was given, then prolly not needed and make them equal
    if r_target_speed is None:
        r_target_speed = l_target_speed

    # calculate distance the bot has traveled during the past cycle
    r_dist = math.pi * robot.WHEEL_DIAMETER * ((variables["right_enc"] - variables["last_right_enc"]) / 360)
    l_dist = math.pi * robot.WHEEL_DIAMETER * ((variables["left_enc"] - variables["last_left_enc"]) / 360)

    # not used currently
    # variables["distance"] = (r_dist + l_dist) / 2

    # time between this and last cycle
    time_diff = variables["current_time"] - variables["last_time"]

    # calculate wheel speeds based on v = s / t
    r_speed = r_dist / time_diff
    l_speed = l_dist / time_diff
    variables["l_speed"], variables["r_speed"] = l_speed, r_speed

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


def decide(variables, left_distance, left_encoder, left_second_encoder, middle_distance, middle_encoder, middle_second_encoder,
           right_distance, right_encoder, right_second_encoder):
    """Decide, which object is which and what to do."""
    print(left_distance, left_encoder, left_second_encoder, middle_distance, middle_encoder, middle_second_encoder,
          right_distance, right_encoder, right_second_encoder)
    print("ümbermõõt", variables["wheel circumference"])
    if right_distance < middle_distance and middle_distance > left_distance:
        arc_length = ((right_encoder - left_second_encoder) * variables["wheel circumference"]) / 360
        angle_between_two_closest_objects = arc_length / (robot.AXIS_LENGTH / 2)
        distance_between_two_closest_objects = sqrt(
            left_distance ** 2 + right_distance ** 2 - 2 * left_distance * right_distance * cos(
                angle_between_two_closest_objects))
        beta = asin((left_distance * sin(
            angle_between_two_closest_objects)) / distance_between_two_closest_objects)  # nurk mida vaja d arvutamiseks
        d = sqrt((distance_between_two_closest_objects / 2) ** 2 + right_distance ** 2 - 2 * (
            distance_between_two_closest_objects / 2) * right_distance * cos(beta))  # palju sõitma peab mediaanini

    elif left_distance < right_distance and middle_distance < right_distance:
        arc_length = ((middle_encoder - left_second_encoder) * variables["wheel circumference"]) / 360
        angle_between_two_closest_objects = arc_length / (robot.AXIS_LENGTH / 2)
        distance_between_two_closest_objects = sqrt(
            left_distance ** 2 + middle_distance ** 2 - 2 * left_distance * middle_distance * cos(
                angle_between_two_closest_objects))
        beta = asin((left_distance * sin(
            angle_between_two_closest_objects)) / distance_between_two_closest_objects)  # nurk mida vaja d arvutamiseks
        d = sqrt((distance_between_two_closest_objects / 2) ** 2 + middle_distance ** 2 - 2 * (
            distance_between_two_closest_objects / 2) * middle_distance * cos(beta))  # palju sõitma peab mediaanini

    else:
        arc_length = ((right_encoder - middle_second_encoder) * variables["wheel circumference"]) / 360
        angle_between_two_closest_objects = arc_length / (robot.AXIS_LENGTH / 2)
        distance_between_two_closest_objects = sqrt(
            middle_distance ** 2 + right_distance ** 2 - 2 * middle_distance * right_distance * cos(
                angle_between_two_closest_objects))
        beta = asin((left_distance * sin(
            angle_between_two_closest_objects)) / distance_between_two_closest_objects)  # nurk mida vaja d arvutamiseks
        d = sqrt((distance_between_two_closest_objects / 2) ** 2 + right_distance ** 2 - 2 * (
            distance_between_two_closest_objects / 2) * right_distance * cos(beta))  # palju sõitma peab mediaanini

    if distance_between_two_closest_objects < robot.AXIS_LENGTH + 0.05:  # kui robot läbi ei mahu +5cm roboti laiusele
        # TODO "phase" = drive to other side of triangle
        return variables
    else:
        print("arc_length", arc_length)
        print("angle between two closest objects", angle_between_two_closest_objects)
        print("beta", beta)
        print("d", d)
        # palju robot peab kõige parempoolsest pöörama et suund oleks mediaan radiaanides
        gamma = asin(((distance_between_two_closest_objects / 2) * sin(beta)) / d)
        gamma = (180 * gamma / pi) % 360
        print("gamma", gamma)
        distance = (pi * robot.AXIS_LENGTH) * (gamma / 360)
        degrees_to_spin = (360 * distance / variables["wheel circumference"])
        target = right_second_encoder - degrees_to_spin
        print("target", target)
        variables["target_turn"] = target
        variables["distance"] = d
        variables["phase"] = "turn"
        return variables

# TODO second 360 scan, find furthest object, turn to it, drive 2/1
# TODO in gold second 360 scan uses rear ir scanner


def plan(variables):
    """Do all the planning in variable dict and then return it. Because Python."""
    object_count = variables["object_count"]
    on_object = variables["on_object"]
    # scanning phase
    if variables["phase"] == "scanning":
        # if condition that is filled every time scanning is started, starts the turning
        if variables["scan_progress"] == 0:
            variables["left_speed"], variables["right_speed"] = 12, -12
            variables["scan_progress"] = 1

        # if scanning is already in progress
        else:
            # last and current fmir sensor reading difference, used for object detection
            diff = variables["last_fmir"] - variables["fmir"]

            # run p controller
            variables = p_speed(variables, 1, 0.035)
            print("lspeed", variables["l_speed"], "rspeed", variables["r_speed"])

            # output for checking the difference, wheel speeds and the buffer
            print("Differnece is: " + str(diff))
            print(variables["left_speed"], variables["right_speed"])
            print(variables["fmir_buffer"])
            print("------------------------------------------------------")

            # if diff is more than 20cm, then it most likely has detected an object
            if diff > 0.20 and object_count == 0 and on_object == 0:
                variables["first_object_first_distance"] = variables["fmir"]  # + robot.AXIS_LENGTH / 2
                variables["first_object_first_encoder"] = variables["last_left_enc"]
                variables["object_count"] = 1
                variables["on_object"] = 1
            elif (diff < -0.20 or variables["fmir"] > variables["first_object_first_distance"] + 0.05) and object_count == 1 and on_object == 1:
                variables["first_object_second_distance"] = variables["fmir"] + robot.AXIS_LENGTH / 2  # vb pole vaja, kuna vale
                variables["first_object_second_encoder"] = variables["last_left_enc"]
                variables["on_object"] = 0
            elif diff > 0.20 and object_count == 1 and on_object == 0:
                variables["second_object_first_distance"] = variables["fmir"]  # + robot.AXIS_LENGTH / 2
                variables["second_object_first_encoder"] = variables["last_left_enc"]
                variables["object_count"] = 2
                variables["on_object"] = 1
            elif (diff < -0.20 or variables["fmir"] > variables["second_object_first_distance"] + 0.05) and object_count == 2 and on_object == 1:
                variables["second_object_second_distance"] = variables["fmir"] + robot.AXIS_LENGTH / 2  # vb pole vaja, kuna vale
                variables["second_object_second_encoder"] = variables["last_left_enc"]
                variables["on_object"] = 0
            elif diff > 0.20 and object_count == 2 and on_object == 0:
                variables["third_object_first_distance"] = variables["fmir"]  # + robot.AXIS_LENGTH / 2
                variables["third_object_first_encoder"] = variables["last_left_enc"]
                variables["object_count"] = 3
                variables["on_object"] = 1
            elif (diff < -0.20 or variables["fmir"] > variables["third_object_first_distance"] + 0.05) and object_count == 3 and on_object == 1:
                variables["third_object_second_distance"] = variables["fmir"] + robot.AXIS_LENGTH / 2  # vb pole vaja, kuna vale
                variables["third_object_second_encoder"] = variables["last_left_enc"]
                variables["on_object"] = 0
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["scan_progress"] = 0
                variables["phase"] = "decide"

    # decide, which object is which and what to do
    elif variables["phase"] == "decide":
        arc_length_1 = ((variables["second_object_first_encoder"] - variables["first_object_second_encoder"]) * variables["wheel circumference"]) / 360
        angle_between_second_and_first = arc_length_1 / (robot.AXIS_LENGTH / 2)
        arc_length_2 = ((variables["third_object_first_encoder"] - variables["second_object_second_encoder"]) * variables["wheel circumference"]) / 360
        angle_between_third_and_second = arc_length_2 / (robot.AXIS_LENGTH / 2)
        if angle_between_second_and_first > 120:  # second left, third middle, first right
            variables = decide(variables, variables["second_object_first_distance"], variables["second_object_first_encoder"], variables["second_object_second_encoder"], variables["third_object_first_distance"], variables["third_object_first_encoder"], variables["third_object_second_encoder"], variables["first_object_first_distance"], variables["first_object_first_encoder"], variables["first_object_second_encoder"])
        elif angle_between_third_and_second > 120:  # third left, first middle, second right
            variables = decide(variables, variables["third_object_first_distance"], variables["third_object_first_encoder"], variables["third_object_second_encoder"], variables["first_object_first_distance"], variables["first_object_first_encoder"], variables["first_object_second_encoder"], variables["second_object_first_distance"], variables["second_object_first_encoder"], variables["second_object_second_encoder"])
        else:  # first left, second, middle, third right
            variables = decide(variables, variables["first_object_first_distance"], variables["first_object_first_encoder"], variables["first_object_second_encoder"], variables["second_object_first_distance"], variables["second_object_first_encoder"], variables["second_object_second_encoder"], variables["third_object_first_distance"], variables["third_object_first_encoder"], variables["third_object_second_encoder"])

    # turn to median, between the two closest objects
    elif variables["phase"] == "turn":
        if variables["turning"] == 0:
            variables["left_speed"], variables["right_speed"] = -12, 12
            variables["turning"] = 1
        elif variables["left_speed"]:  # meaning it's currently turning, therefore the phase has started
            variables = p_speed(variables, 3, 0.035)
            print("leftenc", variables["left_enc"])
            if variables["left_enc"] < variables["target_turn"]:
                variables["left_speed"], variables["right_speed"] = 0, 0
                variables["phase"] = "drive"

    # drive to median, between the two closest objects
    elif variables["phase"] == "drive":
        if variables["driving"] == 0:
            degrees_to_target = 360 * variables["distance"] / (pi * robot.WHEEL_DIAMETER)
            print("left enc", variables["left_enc"], "deg to target", degrees_to_target, "distance", variables["distance"], "wheel dia", robot.WHEEL_DIAMETER)
            variables["target_drive"] = variables["left_enc"] + degrees_to_target
            variables["left_speed"], variables["right_speed"] = 12, 12
            variables["driving"] = 1
        else:
            print("left_enc", variables["left_enc"], "target_drive", variables["target_drive"])
            variables = p_speed(variables, 2, 0.035)
            if variables["left_enc"] > variables["target_drive"]:
                variables["left_speed"], variables["right_speed"] = 0, 0
    # return dictionary with all the new values
    return variables


def act(variables):
    """Change left and right wheel speed based on plan()."""
    robot.set_left_wheel_speed(variables["left_speed"])
    robot.set_right_wheel_speed(variables["right_speed"])


def main():
    """Create some variables used in the loop and then run the loop of sense, plan, act."""
    variables = dict()
    variables["wheel circumference"] = robot.WHEEL_DIAMETER * pi
    variables["object_count"] = 0
    variables["on_object"] = 0
    variables["turning"] = 0
    variables["driving"] = 0
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
    variables["max_fmir"] = float("inf")
    variables["first_object_first_distance"] = float("inf")
    variables["second_object_first_distance"] = float("inf")
    variables["third_object_first_distance"] = float("inf")
    while True:
        variables = sense(variables)
        variables = plan(variables)
        act(variables)
        rospy.sleep(0.05)


if __name__ == "__main__":
    main()

"""
left_object_distance = left_object_distance + robot.AXIS_LENGTH / 2
middle_object_distance = middle_object_distance + robot.AXIS_LENGTH / 2
right_object_distance = right_object_distance + robot.AXIS_LENGTH / 2



wheel_circumference = robot.WHEEL_DIAMETER * pi
arc_length = ((right_object_encoder - left_object_encoder) * wheel_circumference) / 360
alpha = arc_length / (robot.AXIS_LENGTH / 2)
top = sqrt(left_object_distance**2 + right_object_distance**2 - 2 * left_object_distance * right_object_distance * cos(alpha))
beta = asin((left_object_distance * sin(alpha)) / top) #nurk mida vaja d arvutamiseks
d = sqrt((top / 2)**2 + right_object_distance**2 - 2 * (top / 2) * right_object_distance * cos(beta)) #palju ta sõtma peab mediaanini
gamma = asin(((top / 2) * sin(beta)) / d) #palju robot peab kõige parempoolsest pöörama et suund oleks mediaan
gamma = 180 * gamma / pi
distance = (pi * robot.AXIS_LENGTH) * (gamma / 360)
degrees_to_spin = (360 * distance / wheel_circumference)
right_target = right_object_encoder - degrees_to_spin
while robot.get_left_wheel_encoder() > right_target:
    robot.set_right_wheel_speed(15)
    robot.set_left_wheel_speed(-15)
robot.set_wheels_speed(0)
# sõida mediaanini
last_encoder = robot.get_left_wheel_encoder()
print(last_encoder)
degrees_to_target = 360 * (pi * robot.AXIS_LENGTH) * d
target_to_spin = last_encoder + degrees_to_target
while robot.get_left_wheel_encoder() < target_to_spin:
    robot.set_wheels_speed(15)
robot.set_wheels_speed(0)
# TODO kui vasak/parem ja keskmine objekt on lähemal kui parem/vasak ja/või kui ei mahu läbi, siis sõidab 120 kraadi ümber objektide
"""
