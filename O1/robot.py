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


def scan_for_object_vol4():
    lspeed, rspeed = 14, 14
    print("Started scanning")
    lenc = get_lenc()
    sectionsinfullcircle = 20
    step = (360 * robot.AXIS_LENGTH / robot.WHEEL_DIAMETER) / sectionsinfullcircle  # step of turning because some idea
    wheelturngoal = lenc + step  # full 360 degree turn
    sectioncounter = 0
    total = 0
    closestcounter, closestmeasure = 0, float("inf")
    measurecounter = 0
    last_trenc = get_renc()
    last_tlenc = get_lenc()
    turn(lspeed, rspeed, 1)  # does turning with speed 13 clockwise
    while sectioncounter < 5 * sectionsinfullcircle:  # does 5 turns
        fmir = get_fmir()
        total += fmir
        measurecounter += 1
        if wheelturngoal < lenc:  # if left wheel has gone above goal encoder
            tempmeasure = total / measurecounter
            total, measurecounter = 0, 0
            fmir = get_fmir()
            total += fmir
            measurecounter += 1
            if tempmeasure < closestmeasure:
                closestmeasure = tempmeasure
                closestsector = sectioncounter
            print(closestmeasure)

            # if no check was detected add a step to goal and counter
            wheelturngoal += step
            sectioncounter += 1
        rospy.sleep(0.05)

        # this whole part is for error correction
        trenc = get_renc()
        tlenc = get_lenc()
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
        turn(lspeed, rspeed, 1)
        last_trenc = trenc
        last_tlenc = tlenc
        # end of error correction

        lenc = get_lenc()
    set_speed(0)
    last_trenc = get_renc()
    last_tlenc = get_lenc()
    turn(lspeed, rspeed, 0)  # does turning with speed 13 clockwise
    lenc = get_lenc()
    wheelturngoal = lenc - step * closestsector # full 360 degree turn
    while wheelturngoal < lenc:
        rospy.sleep(0.05)
        trenc = get_renc()
        tlenc = get_lenc()
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
        turn(lspeed, rspeed, 1)
        last_trenc = trenc
        last_tlenc = tlenc
        lenc = get_lenc()



def move_towards_object():
    set_speed(20)
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
    scan_for_object_vol4()
    break
    #if move_towards_object():
    #    print("Has science gone too far?")
    #    rospy.sleep(0.3)
    #    set_speed(0)
    #    break
