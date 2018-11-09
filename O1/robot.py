"""Fid object and move to it."""
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


def error_correction(lspeed, rspeed, last_tlenc, last_trenc, mode, side=0):
    """
    Do error correction by adjusting speed by one.

    :param lspeed: current left wheel speed
    :param rspeed: current right wheel speed
    :param last_tlenc: last left wheel encoder when this function ran
    :param last_trenc: last right wheel encoder when this function ran
    :param mode: 1 for turning on the spot, 2 for moving straight forward
    :param side: which side to turn if mode = 1
    :return: new lspeed, rspeed, (last_)tlenc, (last_)trenc
    """
    trenc = get_renc()
    tlenc = get_lenc()

    if mode == 1:  # if we're doing turning at one spot (lspeed == - rspeed), minspeed -inf, maxspeed 16
        maxspeed = 17
    elif mode == 2:  # if we're moving straight forward (lspeed == rspeed)
        maxspeed = 20

    if abs(trenc - last_trenc) > abs(tlenc - last_tlenc):
        if lspeed < maxspeed:
            lspeed += 1
        else:
            rspeed -= 1
    else:
        if rspeed < maxspeed:
            rspeed += 1
        else:
            lspeed -= 1

    if mode == 1:
        turn(lspeed, rspeed, side)
    elif mode == 2:
        set_lspeed(lspeed)
        set_rspeed(rspeed)

    return lspeed, rspeed, tlenc, trenc


def turn(lspeed, rspeed, side):
    """
    Make bot turn at one spot.

    :param lspeed: speed to give to left wheel
    :param rspeed: speed to give to right wheel
    :param side: where to turn. 0 is left, 1 is right.
    """
    if not side:
        lspeed = -lspeed
    else:
        rspeed = -rspeed
    robot.set_left_wheel_speed(lspeed)
    robot.set_right_wheel_speed(rspeed)


def object_in_cache(cache):
    if (cache[0] - cache[1]) > 0.2:
        return True, 0
    if (cache[1] - cache[0]) > 0.2:
        return True, 1
    return False, 0


def scan_for_object():
    """Scan for object."""
    lspeed, rspeed = 15, 15  # initial speeds for left and right wheel
    print("Started scanning")
    total = 0
    for i in range(10):
        total += get_fmir()
        rospy.sleep(0.05)
    last_fmir = total / (i + 1)
    cache = [last_fmir, last_fmir]
    last_trenc = get_renc()  # used for error correction
    last_tlenc = get_lenc()  # used for error correction and also places where left encoder is needed
    sectorsinfullcircle = 30  # how many sectors in full 360 degree turn
    step = (360 * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER) / sectorsinfullcircle  # how much to turn for one sector
    degstep = step
    wheelturngoal = last_tlenc + step  # where first sector ends
    sectorcounter = 0  # which sector is in progress
    total = 0  # total of all the measurements in the sector
    closestmeasure = float("inf")  # what was the closest measurement
    measurecounter = 0  # how many measurements have been made in sector so far
    turn(lspeed, rspeed, 1)  # starts clockwise turning
    lrenc = abs(last_trenc - last_tlenc)  # the encoder difference at the beginning

    while sectorcounter < sectorsinfullcircle:  # does a full 360 degree turn
        # add current fmir to total and add one to measurecounter
        fmir = get_fmir()
        total += fmir
        measurecounter += 1
        # for when bot has reached end of sector
        if (abs(last_trenc - last_tlenc) - lrenc) > (sectorcounter + 1) * degstep * 2:  # if encoder difference has grown more than it would to scan one sector
            tempmeasure = total / measurecounter  # average measurement of fmir during sector
            total, measurecounter = 0, 0  # zeroes them for next sector

            cache.append(tempmeasure)
            cache.pop(0)
            flag, sector = object_in_cache(cache)
            if flag:
                break
            # if this average measure is less than current closest measure, make it the closest measure and save sector
            #if tempmeasure < closestmeasure:
            #    closestmeasure = tempmeasure
            #    closestdiff = (abs(last_trenc - last_tlenc) - lrenc)
            print(closestmeasure, tempmeasure, sectorcounter)  # just printing stuff

            wheelturngoal += step  # end of next sector
            sectorcounter += 1
        rospy.sleep(0.02)

        # function for error correction
        lspeed, rspeed, last_tlenc, last_trenc = error_correction(lspeed, rspeed, last_tlenc, last_trenc, 1, 1)
    set_speed(0)
    if not flag:
        return False



    cdiff = abs(last_trenc - last_tlenc) - lrenc  # gets current encoder difference
    goaldiff = cdiff - (flag + 0.5) * 2 * step
    turn(lspeed, rspeed, 0)  # starts turning counterclockwise
    while (goaldiff) < cdiff:  # while difference is bigger than the difference of middle of goal sector (try 1 multiplier instead of 0.5 if doesn't turn enough)
        rospy.sleep(0.02)
        lspeed, rspeed, last_tlenc, last_trenc = error_correction(lspeed, rspeed, last_tlenc, last_trenc, 1, 0)
        cdiff = abs(last_trenc - last_tlenc) - lrenc
    set_speed(0)


def move_towards_object():
    """Move bot towards object."""
    print("Started moving towards!")
    rspeed, lspeed = 18, 18  # initial speeds

    # get value of fmir encoder (average to combat noise)
    total = 0
    for i in range(10):
        total += get_fmir()
        rospy.sleep(0.05)
    last_fmir = total / (i + 1)

    last_trenc = get_renc()  # for error correction
    last_tlenc = get_lenc()  # for error correction
    measurementcounter = 0
    total = 0
    set_speed(20)
    while True:
        # get average value of fmir over 10 steps
        measurementcounter += 1
        print(measurementcounter)
        total += get_fmir()
        if measurementcounter == 10:
            fmir = total / measurementcounter
            print(fmir)
            total, measurementcounter = 0, 0
            if fmir < 0.22:  # if new value places bot at closer than this value
                set_speed(0)
                return True
            if fmir > last_fmir:  # if last value is somehow smaller than new value, meaning lost object.
                set_speed(0)
                return False
        rospy.sleep(0.02)
        lspeed, rspeed, last_tlenc, last_trenc = error_correction(lspeed, rspeed, last_tlenc, last_trenc, 2)
        print(lspeed, rspeed)


if __name__ == "__main__":
    while True:
        print("I should have started!")
        scan_for_object()
        if move_towards_object():  # if movement reached object correctly
            print("Has science gone too far?")
            # get value of fmir encoder (average to combat noise)
            total = 0
            for i in range(10):
                total += get_fmir()
                rospy.sleep(0.05)
            fmir = total / (i + 1)
            sleep = round((fmir - 0.05), 3) * 2
            set_lspeed(16)
            set_rspeed(16)
            rospy.sleep(sleep)
            set_speed(0)
            break
