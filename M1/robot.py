"""M1."""

from PiBot import PiBot
import rospy


class Robot:
    """Robot for robot stuff."""

    def __init__(self):
        """Initialize robot."""
        # read robot into class var
        self.robot = PiBot()

        # different speeds for sim for not falling asleep
        if self.robot.is_simulation():
            self.is_simulation = True
            self.speed = 15
        else
            self.is_simulation = False
            self.speed = 13

        # initialise phase and set problem solved as false
        self.problem_solved = False
        self.state = "working"

        # initialise adjusters to zero
        self.adjust_left = 0
        self.adjust_right = 0

        # initialise IR variables because good coding practises
        self.left_straight = 0
        self.left_diagonal = 0
        self.left_side = 0
        self.right_straight = 0
        self.right_diagonal = 0
        self.right_side = 0
        self.front_middle = 0
        self.front_right = 0

    def set_speed(self, percentage):
        """Set wheel speeds, but treats backwards as forwards and vice versa."""
        self.robot.set_wheels_speed(-percentage)

    def set_speed_l(self, percentage):
        """Set right wheel speed as negative of percentage because treating back as front."""
        self.robot.set_right_wheel_speed(-percentage)

    def set_speed_r(self, percentage):
        """Set left wheel speed as negative of percentage because treating back as front."""
        self.robot.set_left_wheel_speed(-percentage)

    def close_and_lift_grabber(self):
        """Close and raise grabber out of the way."""
        self.robot.close_grabber(95)
        self.robot.set_grabber_height(95)

    def sense(self):
        """Read in some sensors."""
        self.left_straight = self.robot.get_rear_left_straight_ir()
        self.left_diagonal = self.robot.get_rear_left_diagonal_ir()
        self.left_side = self.robot.get_rear_left_side_ir()

        self.right_straight = self.robot.get_rear_right_straight_ir()
        self.right_diagonal = self.robot.get_rear_right_diagonal_ir()
        self.right_side = self.robot.get_rear_right_side_ir()

        self.front_middle = self.robot.get_front_middle_ir()
        self.front_right = self.robot.get_front_right_ir()

    def rotate(self, degrees):
        """
        Turn given amount of degrees in required direction.

        If degree > 0: turn right.
        If degree < 0: turn left.
        """
        # calculate amount of degrees the wheel has to spin for required degrees (simplified)
        degrees_to_spin = self.robot.AXIS_LENGTH * degrees / self.robot.WHEEL_DIAMETER

        # 1 if deg is pos, -1 if deg is neg
        sign = (abs(degrees) // degrees)

        # save starting encoders for wheels
        left_start = self.robot.get_left_wheel_encoder()
        right_start = self.robot.get_right_wheel_encoder()

        # keep rotating until encoders have both reached required difference, adjusting wheel speeds meanwhile
        while abs(self.robot.get_left_wheel_encoder() - left_start) < abs(degrees_to_spin) and abs(self.robot.get_right_wheel_encoder() - right_start) < abs(degrees_to_spin):
            self.adjust_speed(right_start, left_start, sign, -1)

            # sleep for a while to not react to data it has already reacted to
            rospy.sleep(0.01)

        # reset made adjustments and stop the robot
        self.reset_adjust()
        self.set_speed(0)

    def plan(self):
        """Follow wall on the left (AKA right of moving direction)."""
        # if any of the sensors of the wall side detect a wall within 4.3cm, do rotating
        if self.left_straight < 0.043 or self.left_diagonal < 0.043 or self.left_side < 0.043:
            self.state = "rotate"

        # otherwise just move forward
        else:
            self.state = "move forward"

        # if on the non-wall side all close range sensors have no wall and back (AKA front) middle and non-wall sensors
        # detect no walls (meaning outside of maze), set problem solved key to True, stopping the robot.
        if self.right_straight >= 0.049 and self.right_diagonal >= 0.049 and self.right_side >= 0.049 and self.robot.front_middle >= 0.99 and self.robot.front_right >= 0.99:
            self.problem_solved = False

    def act(self):
        """Act based on phase."""
        # if key got put as rotate, do that
        if self.state == "rotate":
            self.rotate(-15)

        # otherwise just move in an arc tilting right towards the wall
        else:
            self.set_speed_r(15)
            self.set_speed_l(27)

    def adjust_speed(self, right_start, left_start, sign, rotation=1):
            """Adjust wheel speeds dynamically."""
            # get the difference of encoder differences against (rotation) start encoder values
            diff_encoders = abs(self.robot.get_right_wheel_encoder() - right_start) - abs(self.robot.get_left_wheel_encoder() - left_start)

            # if the difference is above 5 degrees (so it wouldn't react to very small differences)
            if diff_encoders > 5:
                print("Adjust left.")

                # only adjust if current adjustment is below 7 to not go to infinity and beyond
                if self.adjust_left < 7:
                    self.adjust_left += 1

                # and erase right adjustment
                self.adjust_right = 0

            # if difference below -5 deg
            elif diff_encoders < -5:
                print("Adjust right.")

                # only adjust if adjustment is below 7 absolute percents
                if self.adjust_right < 7:
                    self.adjust_right += 1

                # and erase left adjustment
                self.adjust_left = 0

            # unused in current version as at least one is always 0
            if self.adjust_left > 0 and self.adjust_right > 0:
                self.adjust_left -= 1
                self.adjust_right -= 1

            # set speeds to wheels, if rotation is -1, right (left if front is front) turns opposite to other wheel
            self.set_speed_r(rotation * sign * (self.speed + self.adjust_left))
            self.set_speed_l(sign * (self.speed + self.adjust_right))

    def reset_adjust(self):
        """Reset wheel speed adjust variables used in speed adjusting."""
        self.adjust_left = 0
        self.adjust_right = 0

    def main(self):
        """Main sense plan act loop."""
        # deal with the grabber just in case
        self.close_and_lift_grabber()

        # do loop while not solved
        while not self.problem_solved:
            self.sense()
            self.plan()
            self.act()
            rospy.sleep(0.05)  # 20 Hz


if __name__ == "__main__":
    robot = Robot()
    robot.main()
