import rospy
from PiBot import PiBot
import math


class Robot:

    def __init__(self):
        self.robot = PiBot()
        self.problem_solved = False

        # used for p-controller
        self.gain = 50

        # first letter, side: l = left, r = right
        # second letter, position: f = front, d = diagonal, s = side
        self.rfir, self.rdir, self.rsir, self.lfir, self.ldir, self.lsir = 0, 0, 0, 0, 0, 0
        self.rfir_buf, self.rdir_buf, self.rsir_buf, self.lfir_buf, self.ldir_buf, self.lsir_buf = [], [], [], [], [], []

        # not sure if these are needed, also in distance_sensors()
        self.last_rfir, self.last_rdir, self.last_rsir, self.last_lfir, self.last_ldir, self.last_lsir = 0, 0, 0, 0, 0, 0

        # create lists for easier usage in distance_sensors()
        self.short_distance_sensors = [self.rfir, self.rdir, self.rsir, self.lfir, self.ldir, self.lsir]
        self.buffers = [self.rfir_buf, self.rdir_buf, self.rsir_buf, self.lfir_buf, self.ldir_buf, self.lsir_buf]

        # the amount of cycles sets the length of the buffers of sensors
        self.initialiser = True
        for _ in range(3):
            self.distance_sensors()
        self.initialiser = False

        # initialise other variables used in sense so that no issues occur with fresh values
        self.last_left_enc = 0
        self.last_right_enc = 0
        self.right_enc = - self.robot.get_left_wheel_encoder()
        self.left_enc = - self.robot.get_right_wheel_encoder()
        self.turn_amount = 0
        self.la_time = 0
        self.cu_time = rospy.get_time()
        self.r_dist = 0
        self.l_dist = 0
        self.dist = 0

        # and rest of variables
        self.l_speed = 0
        self.r_speed = 0
        self.p_ignore = False  # used to bypass p-controller
        self.init = True

        self.state = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaah"

    def distance_sensors(self):
        # get new sensor readings
        readings = self.robot.get_rear_irs()

        # not sure if necessary anywhere
        self.last_rfir, self.last_rdir, self.last_rsir, self.last_lfir, self.last_ldir, self.last_lsir = self.rfir, self.rdir, self.rsir, self.lfir, self.ldir, self.lsir

        # read new readings into buffers while removing oldest and put new sensor value if it is within +-15%
        for i in range(6):
            # don't pop if initialising
            if not self.initialiser:
                self.buffers[i].pop(0)
            self.buffers[i].append(readings[i])

            # condition to avoid division by zero during initialising
            if len(self.buffers[i]):
                average = sum(self.buffers[i]) / len(self.buffers[i])
            else:
                average = sum(self.buffers[i])

            # if within error, sensor value goes as current
            if average * 0.85 < readings[i] < average * 1.15:
                self.short_distance_sensors[i] = readings[i]
        self.rsir, self.rdir, self.rfir, self.lfir, self.ldir, self.lsir = self.short_distance_sensors

    def sense(self):
        # put last encoder values into respective dict keys
        self.last_left_enc = self.left_enc
        self.last_right_enc = self.right_enc

        # read new values in, treats front as back
        self.right_enc = - self.robot.get_left_wheel_encoder()
        self.left_enc = - self.robot.get_right_wheel_encoder()

        # calculate how much robot has turned during the tick in degrees, clockwise
        self.turn_amount = self.robot.WHEEL_DIAMETER * ((self.left_enc - self.right_enc) - (
                self.last_left_enc - self.last_right_enc)) / (2 * self.robot.AXIS_LENGTH)

        # update distance sensors
        self.distance_sensors()

        # put last time in and update current time
        self.la_time = self.cu_time
        self.cu_time = rospy.get_time()

        # calculate distance the bot has traveled during the past tick
        self.r_dist = math.pi * self.robot.WHEEL_DIAMETER * ((self.right_enc - self.last_right_enc) / 360)
        self.l_dist = math.pi * self.robot.WHEEL_DIAMETER * ((self.left_enc - self.last_left_enc) / 360)
        self.dist = (self.r_dist + self.l_dist) / 2

    def p_speed(self, l_target_speed, r_target_speed=None):  # target speed should be in meters/second
        # just a check to not do anything if speed is 0 or last speed was 0 or p_ignore
        if (self.r_speed == 0 and self.l_speed == 0) or self.p_ignore:
            self.p_ignore = False
            return

        # get which method it'll be:
        if self.l_speed < 0 < self.r_speed:  # counterclockwise turning
            method = 3
        elif self.l_speed > 0 > self.r_speed:  # clockwise turning
            method = 1
        elif self.l_speed > 0 < self.r_speed:  # moving forward
            method = 2
        else:   # elif self.l_speed < 0 > self.r_speed:  # moving backwards
            method = 4

        # if no r target speed was given, then prolly not needed and make them equal
        if r_target_speed is None:
            r_target_speed = l_target_speed

        # time between this and last cycle
        time_diff = self.cu_time - self.la_time

        # calculate wheel speeds based on v = s / t
        r_veloc = self.r_dist / time_diff
        l_veloc = self.l_dist / time_diff

        # get left wheel speed error
        if method == 1 or method == 2:  # clockwise turning or moving straight
            l_error = l_target_speed - l_veloc
        else:  # elif method == 3 or method == 4:  # counterclockwise turning or moving backwards
            l_error = - l_target_speed - l_veloc

        # get right wheel speed error, two separate versions because right wheel turns backwards during turning
        if method == 1 or method == 4:  # clockwise turning or moving backwards
            r_error = - r_target_speed - r_veloc
        else:  # elif method == 2 or method == 3:  # moving straight or counterclockwise turning
            r_error = r_target_speed - r_veloc

        # calculate new right and left wheel speeds by adding rounded value of GAIN constant times wheel speed error
        self.r_speed = self.r_speed + round(self.gain * r_error)
        self.l_speed = self.l_speed + round(self.gain * l_error)

    def plan(self):
        """
        CURRENT PLAN:
        
        phase 1: move forward (tilted) until it detects a wall with any of the sensors
            if side sensor is closest, go to wall following phase
            if diagonal sensor is closest, turn bot so side sensor is closest and then go to wall follow
            if front sensor is closest:
                turn a bit on the spot and start the phase 1 again
        phase 2: wall following
            if bot has gotten too close to the wall or is starting to get closer, adjust bot wheel speed to be slower
            if is optimal spacing, adjust speeds to be equal
            if has gotten too far or is starting to drift away, adjust other wheel to be slower
            
            if has detected a wall with front sensors:
                adjust so both front IR sensors are more or less equal
                turn 45 degrees so front IR would be facing corner
                back up a tiny bit
                turn 45 + 90 degrees in the other direction
                (potentially could do turn 90, move forward a bit, do 45)
                start from phase 1? Or if it still is close enough to wall, from phase 2
            
            if has lost wall with side sensor suddenly:
                move forward a tiny bit
                turn 45 degrees
                move forward a tiny bit
                turn 45 degrees
                start from either phase 1 or phase 2 again
        """
        if self.state == "turn":
            # initialisation
            if self.init:
                self.init = False
                # zero the turn amount
                self.turn_progress = 0
                # if goal is positive aka clockwise
                if self.goal > 0:
                    # set speeds as so
                    self.l_speed, self.r_speed = 12, -12

                # if goal is negative aka counterclockwise
                if self.goal < 0:
                    # set speeds and p controller as so
                    self.l_speed, self.r_speed = -12, 12

            # if not initialisation
            else:
                # do p controlling and update how much bot has turned
                self.p_speed(0.025)
                self.turn_progress += self.turn_amount

                # if has turned enough, stop bot and go to next phase
                if abs(self.goal) - abs(self.turn_progress) < 0:
                    self.l_speed, self.r_speed = 0, 0
                    self.state = self.next_state
                    self.init = True

        elif self.state == "move":
            if self.init:
                self.init = False
                self.move_progress = 0
                if self.goal < 0:
                    self.l_speed, self.r_speed = -12, -12
                else:
                    self.l_speed, self.r_speed = 12, 12
            else:
                self.p_speed(0.025)
                self.move_progress += self.dist

                # if has turned enough, stop bot and go to next phase
                if abs(self.goal) - abs(self.move_progress) < 0:
                    self.l_speed, self.r_speed = 0, 0
                    self.state = self.next_state
                    self.init = True

        elif self.state == "blind forward":
            if self.init:
                self.init = False
                self.l_speed, self.r_speed = 14, 12
            else:
                self.p_speed(0.05, 0.025)
                print(self.rsir, self.rdir, self.rfir)
                if self.rsir < 0.03 or self.rdir < 0.03 or self.rfir < 0.02:
                    if self.rsir < 0.03:
                        self.init = True
                        self.state = "wall follow"
                    elif self.rdir < 0.03:
                        self.init = True
                        self.state = "rota for side"
                    elif self.rfir < 0.02:
                        self.init = True
                        self.state = "turn"
                        self.goal = -30
                        self.next_state = "blind forward"

        elif self.state == "rota for side":
            if self.init:
                self.init = False
                self.l_speed, self.r_speed = -12, 12
            else:
                self.p_speed(0.025)
                if self.rsir + 0.01 < self.rdir:
                    self.init = True
                    self.l_speed, self.r_speed = 0, 0
                    self.state = "wall follow"

        elif self.state == "wall follow":
            if self.init:
                self.init = False
                self.l_speed, self.r_speed = 12, 12
            else:
                if self.rsir < 0.025:
                    self.p_speed(0.025, 0.035)
                    if self.rsir < 0.0175:
                        self.init = True
                        self.state = "turn"
                        self.next_state = "wall follow"
                        self.goal = - 15
                elif self.rsir > 0.035:
                    self.p_speed(0.05, 0.025)
                    if self.rsir > 0.045:
                        self.init = True
                        self.state = "turn"
                        self.next_state = "wall follow"
                        self.goal = 15

                if self.lfir < 0.02 and self.rfir < 0.02:
                    self.l_speed, self.r_speed = 0, 0
                    self.init = True
                    self.state = "turn"
                    self.goal = 45
                    self.next_state = "back up"

                elif self.rsir > 0.045:
                    self.l_speed, self.r_speed = 0, 0
                    self.init = True
                    self.state = "move"
                    self.goal = 0.03
                    self.next_state = "turn_wait"

        elif self.state == "back up":
            if self.init:
                self.init = False
                self.goal = - 0.03
                self.l_speed, self.r_speed = -12, -12
            else:
                self.p_speed(-0.025)
                self.goal -= self.dist
                if self.goal > 0:
                    self.l_speed, self.r_speed = 0, 0
                    self.state = "turn"
                    self.next_state = "blind forward"
                    self.goal = - (90 + 45)
                    self.init = True

        elif self.state == "turn_wait":
            self.state = "turn"
            self.goal = 45
            self.next_state = "move_wait"

        elif self.state == "move_wait":
            self.state = "move"
            self.goal = 0.03
            self.next_state = "turn_wait_part_2"

        elif self.state == "turn_wait_part_2":
            self.state = "turn"
            self.goal = 45
            self.next_state = "blind forward"

        print("I reached the end of plan", self.state)

    def act(self):
        # flips it so back is forward
        self.robot.set_right_wheel_speed(-self.l_speed)
        self.robot.set_left_wheel_speed(-self.r_speed)

    def main(self):
        # just in case robot doesn't have claw properly put away
        self.robot.close_grabber(100)
        self.robot.set_grabber_height(100)
        self.state = "blind forward"
        print("I should have started", self.state)
        # main loop
        while not self.problem_solved:
            self.sense()
            self.plan()
            self.act()
            rospy.sleep(0.05)  # 20 Hz


if __name__ == "__main__":
    robot = Robot()
    robot.main()
