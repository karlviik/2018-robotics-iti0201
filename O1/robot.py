import rospy
from PiBot import PiBot

robot = PiBot()

set_speed = robot.set_wheels_speed
set_rspeed = robot.set_right_wheel_speed
set_lspeed = robot.set_left_wheel_speed
get_flir = robot.get_front_left_ir
get_fmir = robot.get_front_middle_ir
get_frir = robot.get_front_right_ir


def turn(speed, side):
    if not side:
        speed = -speed
    robot.set_left_wheel_speed(speed)
    robot.set_right_wheel_speed(- speed)


def turn_precise(degrees, side, speed):
    wheelturngoal = (degrees * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER)
    multiplier = 1
    if side == 0:
        multiplier = -1
    wheelturngoal = wheelturngoal * multiplier
    lencgoal = robot.get_left_wheel_encoder() + wheelturngoal

    if side == 1:
        turn(speed, 1)
        while lencgoal > robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
    else:
        turn(speed, 0)
        while lencgoal < robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
    set_speed(0)


def scan_for_object():
    robot.set_left_wheel_speed(13)
    robot.set_right_wheel_speed(- 13)
    left_encoder = robot.get_left_wheel_encoder()
    wheelturngoal = left_encoder + (360 * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER)  # full 360 degree turn
    turn(16, 1)  # does turning with speed 13 clockwise
    last_middle_ir = get_fmir()
    while left_encoder < wheelturngoal:
        middle_ir = get_fmir()
        if abs(last_middle_ir - middle_ir) > 0.1:
            set_speed(20)
            break
        last_middle_ir = middle_ir
        rospy.sleep(0.005)
        left_encoder = robot.get_left_wheel_encoder()


def check_cache_for_object(cache):
    print(cache)
    if cache[0] <= cache[1] <= cache[2] > cache[3] < cache[4]:
        return True, -1
    # if cache[0] <= cache[1] > cache[2] > cache[3] < cache[4]:
    #     return True, -1
    if cache[0] > cache[1] > cache[2] < cache[3] >= cache[4]:
        return True, -2
    # if cache[0] > cache[1] < cache[2] < cache[3] >= cache[4]:
    #     return True, -3
    return False, 0


def scan_for_object_vol2():
    print("Started scanning")
    left_encoder = robot.get_left_wheel_encoder()
    step = (360 * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER) / 30  # step of turning because some idea
    wheelturngoal = left_encoder + step  # full 360 degree turn
    turn(16, 1)  # does turning with speed 13 clockwise
    fmir = get_fmir()
    counter = 0
    cache = [fmir, fmir, fmir, fmir, fmir]
    while counter < 100:  # does 5 turns
        if wheelturngoal < left_encoder:  # if left wheel has gone above goal encoder

            # add new fmir to cache and remove oldest
            fmir = get_fmir()
            cache.pop(0)
            cache.append(fmir)

            # get if past 5 things have had an object
            check, backstep = check_cache_for_object(cache)

            # if so, stop and turn back backstep amount of steps to center on the object, hopefully
            if check:
                set_speed(0)
                wheelturngoal = left_encoder + step * backstep  # this sets the goal encoder, backstep is negative
                turn(16, 0)

                # while bot hasn't rotated back to that point just keep doing it
                while wheelturngoal < left_encoder:
                    rospy.sleep(0.05)
                    left_encoder = robot.get_left_wheel_encoder()
                break  # break the loop if it reaches the object thingy line

            # if no check was detected add a step to goal and counter
            wheelturngoal += step
            counter += 1
        rospy.sleep(0.05)
        left_encoder = robot.get_left_wheel_encoder()


def scan_for_object_vol3():
    print("do something stuff hahahahaha")
    left_encoder = robot.get_left_wheel_encoder()
    wheelturngoal = left_encoder + (360 * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER)  # full 360 degree turn
    turn(16, 1)  # does turning with speed 13 clockwise
    closest, encoder = float("inf"), 0
    while left_encoder < wheelturngoal:
        middle_ir = get_fmir()
        if closest > middle_ir:
            closest = middle_ir
            encoder = left_encoder
            print(closest)
        rospy.sleep(0.005)
        left_encoder = robot.get_left_wheel_encoder()
    set_speed(0)
    turn(16, 0)
    while left_encoder > encoder:
        rospy.sleep(0.01)
        left_encoder = robot.get_left_wheel_encoder()
    set_speed(0)


def move_towards_object():
    last_fmir = get_fmir()
    fmir = get_fmir()
    while fmir > 0.17:
        if fmir > last_fmir:
            set_speed(0)
            return False
        last_fmir = fmir
        rospy.sleep(0.005)
        fmir = get_fmir()
    return True


def move_towards_object_vol2():
    print("Started moving towards!")
    set_speed(20)
    rspeed, lspeed = 20, 20
    last_fmir = get_fmir()
    fmir = get_fmir()
    while True:
        while last_fmir >= fmir > 0.17:
            print("I should be moving straight forward ight now!")
            last_fmir = fmir
            rospy.sleep(0.05)
            fmir = get_fmir()
        while 0.17 < last_fmir < fmir:
            print("Ohnoes I started second loop!")
            if rspeed < lspeed:
                lspeed = 15
                rspeed = 20
            else:
                lspeed = 20
                rspeed = 15
            set_lspeed(lspeed)
            set_rspeed(rspeed)
            last_fmir = fmir
            rospy.sleep(0.05)
            fmir = get_fmir()
        if fmir < 0.17:
            break
        print("I ended main loop!")
        set_speed(20)
        rspeed, lspeed = 20, 20
    return True


while True:
    print("I should have started!")
    scan_for_object_vol3()
    if move_towards_object_vol2():
        print("Has science gone too far?")
        rospy.sleep(0.3)
        set_speed(0)
        break
