import rospy
from PiBot import PiBot

robot = PiBot()

set_speed = robot.set_wheels_speed
set_rspeed = robot.set_right_wheel_speed
set_lspeed = robot.set_left_wheel_speed
get_flir = robot.get_front_left_ir
get_fmir = robot.get_front_middle_ir
get_frir = robot.get_front_right_ir
get_lenc = robot.get_left_wheel_encoder
get_renc = robot.get_right_wheel_encoder


def error_correction(lspeed, rspeed, last_tlenc, last_trenc, mode, side = 0):
    """
    Do error correction by adjusting speed by one.

    :param lspeed: current left wheel speed
    :param rspeed: current right wheel speed
    :param last_tlenc: last left wheel encoder when this function ran
    :param last_trenc: last right wheel encoder when this function ran
    :param mode:
    :param side: which side to turn if mode = 1
    :return: new lspeed, rspeed, (last_)tlenc, (last_)trenc
    """
    trenc = get_renc()
    tlenc = get_lenc()

    # if we're doing turning at one spot (lspeed == - rspeed), minspeed -inf, maxspeed 16
    if mode == 1:
        if abs(trenc - last_trenc) > abs(tlenc - last_tlenc):
            if lspeed < 16:
                lspeed += 1
            else:
                rspeed -= 1
        else:
            if rspeed < 16:
                rspeed += 1
            else:
                lspeed -= 1
        turn(lspeed, rspeed, side)

    # if we're moving straight forward (lspeed == rpseed), minspeed -inf, maxspeed 23
    elif mode == 2:
        if abs(trenc - last_trenc) > abs(tlenc - last_tlenc):
            if lspeed < 23:
                lspeed += 1
            else:
                rspeed -= 1
        else:
            if rspeed < 23:
                rspeed += 1
            else:
                lspeed -= 1
        set_lspeed(lspeed)
        set_rspeed(rspeed)

    # if we're doing... Something else?
    return lspeed, rspeed, tlenc, trenc


def turn(lspeed, rspeed, side):
    if not side:
        lspeed = -lspeed
    else:
        rspeed = -rspeed
    robot.set_left_wheel_speed(lspeed)
    robot.set_right_wheel_speed(rspeed)


def turn_precise(degrees, side, speed):
    wheelturngoal = (degrees * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER)
    multiplier = 1
    lspeed, rspeed = speed, speed
    if side == 0:
        multiplier = -1
    wheelturngoal = wheelturngoal * multiplier
    lencgoal = robot.get_left_wheel_encoder() + wheelturngoal

    last_tlenc = get_lenc
    last_trenc = get_renc

    if side == 1:
        turn(lspeed, rspeed, 1)
        while lencgoal > robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
            lspeed, rspeed, last_tlenc, last_trenc = error_correction(lspeed, rspeed, last_tlenc, last_trenc, 1, 1)
    else:
        turn(lspeed, rspeed, 0)
        while lencgoal < robot.get_left_wheel_encoder():
            rospy.sleep(0.05)
            lspeed, rspeed, last_tlenc, last_trenc = error_correction(lspeed, rspeed, last_tlenc, last_trenc, 1, 0)
    set_speed(0)


def scan_for_object():
    lspeed, rspeed = 14, 14
    print("Started scanning")
    last_trenc = get_renc()  # used for error correction
    last_tlenc = get_lenc()  # used for error correction and also places where left encoder is needed
    sectorsinfullcircle = 20  # how many sectors in full 360 degree turn
    step = (360 * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER) / sectorsinfullcircle  # how much to turn for one sector
    degstep = step
    wheelturngoal = last_tlenc + step  # where first sector ends
    sectorcounter = 0  # which sector is in progress
    total = 0 # total of all the measurements in the sector
    closestsector = 0  # which sector had the closest measurement
    closestmeasure = float("inf")  # what was the closest measurement
    measurecounter = 0  # how many measurements have been made in sector so far
    turn(lspeed, rspeed, 1)  # starts clockwise turning

    lenc = last_tlenc
    renc = last_trenc
    lrenc = abs(lenc - renc)

    while sectorcounter < sectorsinfullcircle:  # does a full 360 degree turn
        # add current fmir to total and add one to measurecounter
        fmir = get_fmir()
        total += fmir
        measurecounter += 1
        # for when bot has reached end of sector
        if (abs(last_trenc - last_tlenc) - lrenc) > (sectorcounter + 1) * degstep * 2:  # wheelturngoal < last_tlenc:
            tempmeasure = total / measurecounter  # average measurement of fmir during sector
            total, measurecounter = 0, 0  # zeroes them for next sector

            # if this average measure is less than current closest measure, make it the closest measure and save sector
            if tempmeasure < closestmeasure:
                closestmeasure = tempmeasure
                closestsector = sectorcounter
                closestdiff = (abs(last_trenc - last_tlenc) - lrenc)
            print(closestmeasure, tempmeasure, sectorcounter)

            wheelturngoal += step  # end of next sector
            sectorcounter += 1
        rospy.sleep(0.02)

        # function for error correction
        lspeed, rspeed, last_tlenc, last_trenc = error_correction(lspeed, rspeed, last_tlenc, last_trenc, 1, 1)

    set_speed(0)  # stops the bot
    last_trenc = get_renc()
    last_tlenc = get_lenc()
    cdiff = abs(last_trenc - last_tlenc) - lrenc
    turn(lspeed, rspeed, 0)  # starts turning counterclockwise
    wheelturngoal = last_tlenc - step * (sectorcounter - closestsector + 0.5)  # aim for middle of closest sector
    while (closestdiff - 0.5 * step) < cdiff:
        rospy.sleep(0.02)
        lspeed, rspeed, last_tlenc, last_trenc = error_correction(lspeed, rspeed, last_tlenc, last_trenc, 1, 0)
        cdiff = abs(last_trenc - last_tlenc) - lrenc
    set_speed(0)


def move_towards_object():
    print("Started moving towards!")
    rspeed, lspeed = 20, 20
    total = 0
    for i in range(20):
        total += get_fmir()
        rospy.sleep(0.05)
    last_fmir = total / (i + 1)
    last_trenc = get_renc()
    last_tlenc = get_lenc()
    measurementcounter = 0
    total = 0
    set_speed(20)
    while True:
        measurementcounter += 1
        total += get_fmir()
        if measurementcounter == 10:
            fmir = total / measurementcounter
            total, measurementcounter = 0, 0
            if fmir < 0.2:
                set_speed(0)
                return True
            if fmir > last_fmir:
                set_speed(0)
                return False
        print("I ended main loop!")
        rospy.sleep(0.05)
        lspeed, rspeed, last_tlenc, last_trenc = error_correction(lspeed, rspeed, last_tlenc, last_trenc, 2)


while True:
    print("I should have started!")
    scan_for_object()
    break
    if move_towards_object():
        set_speed(15)
        print("Has science gone too far?")
        rospy.sleep(0.2)
        set_speed(0)
        break
