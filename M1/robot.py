"""M1."""

from PiBot import PiBot
import rospy


class Robot:
    """Do robot."""

    def __init__(self):
        """Initialisation"""
        self.robot = PiBot()

        if self.robot.is_simulation():
            self.is_simulation = True
            self.speed = 15
        else:
            self.is_simulation = False
            self.speed = 13
        self.speed_r = self.speed
        self.speed_l = self.speed

        self.problem_solved = False
        self.state = "working"

        self.adjust_left = 0
        self.adjust_right = 0

    def set_speed(self, percentage):
        """
        Set wheel speeds, but treats backwards as forwards and vice versa. Mostly for shorter typing.

        :param percentage: percentage
        :return: None
        """
        self.robot.set_wheels_speed(-percentage)

    def set_speed_r(self, percentage):
        """
        Set right wheel speed as negative of perc(entage) because treating back as front.

        :param percentage: percentage
        :return: None
        """
        self.robot.set_right_wheel_speed(-percentage)

    def set_speed_l(self, percentage):
        """
        Set left wheel speed as negative or perc(entage) because treating back as front.

        :param percentage: percentage
        :return: None
        """
        self.robot.set_left_wheel_speed(-percentage)

    def close_and_lift_grabber(self):
        """Function sets robot's grabber in a highest position possible."""
        self.robot.close_grabber(95)
        self.robot.set_grabber_height(95)

    def sense(self):
        """sensing."""
        self.left_straight = self.robot.get_rear_left_straight_ir()
        self.left_diagonal = self.robot.get_rear_left_diagonal_ir()
        self.left_side = self.robot.get_rear_left_side_ir()

        self.right_straight = self.robot.get_rear_right_straight_ir()
        self.right_diagonal = self.robot.get_rear_right_diagonal_ir()
        self.right_side = self.robot.get_rear_right_side_ir()

        self.front_middle = self.robot.get_front_middle_ir()
        self.front_right = self.robot.get_front_right_ir()

    def rotate_x_degrees(self, degrees):
        """
        Robot does a pivot [degree] to the chosen direction.

        If degree > 0 --> pivot to the right.
        If degree < 0 --> pivot to the left.

        :param degrees: int: power of pivot.
        """
        # Math part.
        print("ROTATING FOR " + str(degrees))
        distance = (3.141592 * 2 * self.robot.AXIS_LENGTH) * (degrees / 360)
        wheel_circumference = self.robot.WHEEL_DIAMETER * 3.141592
        degrees_to_spin = (360 * distance / wheel_circumference) / 2
        sign = (abs(degrees) // degrees)

        left_start = self.robot.get_left_wheel_encoder()
        right_start = self.robot.get_right_wheel_encoder()

        self.reset_adjust()

        while abs(self.robot.get_left_wheel_encoder() - left_start) < abs(degrees_to_spin) and abs(
                self.robot.get_right_wheel_encoder() - right_start) < abs(degrees_to_spin):
            self.adjust_speed(right_start, left_start, sign, -1)
        self.reset_adjust()
        self.set_speed(0)

    def plan(self):
        """follow the wall on the left."""
        self.reset_adjust()

        if self.left_straight < 0.043 or self.left_diagonal < 0.043 or self.left_side < 0.043:
            self.state = "rotating"
        else:
            self.state = "moving forward"

        if self.right_straight >= 0.049 and self.right_diagonal >= 0.049 and self.right_side >= 0.049 and self.robot.get_front_middle_ir() >= 0.99 and self.robot.get_front_right_ir() >= 0.99:
            self.problem_solved = True
            self.robot.set_grabber_height(50)

    def act(self):
        """acting."""
        print("Acting...")
        if self.state == "rotating":
            self.rotate_x_degrees(-15)
        else:
            self.set_speed_l(15)
            self.set_speed_r(27)

    def adjust_speed(self, right_start, left_start, sign, rotation=1):
            """adjust motors' speed."""
            diff_encoders = abs(self.robot.get_right_wheel_encoder() - right_start) - abs(
                self.robot.get_left_wheel_encoder() - left_start)

            if diff_encoders > 5:
                print('left too slow, adjust left')
                if self.adjust_left < 7:
                    self.adjust_left += 1
                self.adjust_right = 0
            elif diff_encoders < -5:
                print('right too slow, adjust right')
                if self.adjust_right < 7:
                    self.adjust_right += 1
                self.adjust_left = 0
            if self.adjust_left > 0 and self.adjust_right > 0:
                self.adjust_left -= 1
                self.adjust_right -= 1

            self.set_speed_l(rotation * sign * (self.speed + self.adjust_left))
            self.set_speed_r(sign * (self.speed + self.adjust_right))

    def reset_adjust(self):
        """Reset wheel adjustments."""
        self.adjust_left = 0
        self.adjust_right = 0

    def main(self):
        """Main loop function."""
        self.close_and_lift_grabber()

        while not self.problem_solved:
            self.sense()
            self.plan()
            self.act()
            rospy.sleep(0.05)


if __name__ == "__main__":
    robot = Robot()
    robot.main()
