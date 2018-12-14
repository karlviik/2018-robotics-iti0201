"""M1."""

from PiBot import PiBot
import rospy


class Robot:
    """
    The three primitives robotic paradigm: sense, plan, act [1].

    Sense - gather information using the sensors
    Plan - create a world model using all the information and plan the next move
    Act - carry out the next step of the plan using actuators

    [1] https://en.wikipedia.org/wiki/Robotic_paradigm
    """

    def __init__(self):
        """initialisation method."""
        print("Initializing...")
        self.robot = PiBot()

        if self.robot.is_simulation():
            self.is_simulation = True
            self.speed = 15
        else:
            self.is_simulation = False
            self.speed = 13
        self.speed_r = self.speed
        self.speed_l = self.speed

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

    def move_x_cm_forward(self, distance, limit=None):
        """
        Move x centimeters forward.

        :param distance: [float] distance to travel in centimeters
        """
        print("MOVE FORWARD FOR " + str(distance))
        degrees_to_spin = 360 * (distance / 100) / (self.robot.WHEEL_DIAMETER * 3.141592)
        sign = (abs(degrees_to_spin) // degrees_to_spin)

        left_start = self.robot.get_left_wheel_encoder()
        right_start = self.robot.get_right_wheel_encoder()

        self.reset_adjust()

        while abs(self.robot.get_left_wheel_encoder() - left_start) < abs(degrees_to_spin) and abs(
                self.robot.get_right_wheel_encoder() - right_start) < abs(degrees_to_spin):
            self.adjust_speed(right_start, left_start, sign)
            if limit is not None and self.robot.get_rear_right_straight_ir() < limit:
                check = self.is_noise(self.robot.get_rear_right_straight_ir(), self.robot.get_rear_right_straight_ir)
                if check[0] is True and check[1] <= limit:
                    return True

        self.reset_adjust()
        self.set_speed(0)

    def close_and_lift_grabber(self):
        """Function sets robot's grabber in a highest position possible."""
        self.robot.close_grabber(95)
        self.robot.set_grabber_height(95)

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

    def adjust_speed(self, right_start, left_start, sign, rotation=1):
        """Adjust motors' speed."""
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

        print(
            "diff: " + str(diff_encoders) + ", left enc " + str(self.robot.get_left_wheel_encoder()) + " adj: " + str(
                self.adjust_left) + ", right enc: " + str(self.robot.get_right_wheel_encoder()) + " adj: " + str(
                self.adjust_right))

        self.set_speed_l(rotation * sign * (self.speed + self.adjust_left))
        self.set_speed_r(sign * (self.speed + self.adjust_right))
        rospy.sleep(0.005)

    def wall_follow(self, start_value):
        """follow the wall on the left."""
        left_start = self.robot.get_left_wheel_encoder()
        right_start = self.robot.get_right_wheel_encoder()
        self.reset_adjust()

        while True:
            if self.robot.get_rear_right_straight_ir() < 0.04 or self.robot.get_rear_left_diagonal_ir() < 0.04:
                print('forward too close')
                check = self.is_noise(self.robot.get_rear_right_straight_ir(), self.robot.get_rear_right_straight_ir)
                check2 = self.is_noise(self.robot.get_rear_left_diagonal_ir(), self.robot.get_rear_left_diagonal_ir)

                if (check[0] is True and check[1] <= 0.04) or (check2[0] is True and check2[1] <= 0.04):
                    print("MAKING LEFT TURN")
                    self.move_x_cm_forward(-3)
                    self.rotate_x_degrees(90)
                    self.move_x_cm_forward(-3)
                    self.rotate_x_degrees(-180)
                    start_value = self.approach_wall()
            elif self.robot.get_rear_left_diagonal_ir() >= 0.0499:
                print('no right ')
                #                 and self.robot.get_rear_left_diagonal_ir() >= 0.05
                check = self.is_noise(self.robot.get_rear_left_diagonal_ir(), self.robot.get_rear_left_diagonal_ir)
                if check[0] is True and check[1] >= 0.048:
                    self.move_x_cm_forward(20)
                    self.rotate_x_degrees(85)
                    self.move_x_cm_forward(25)
                    self.rotate_x_degrees(85)
                    if self.move_x_cm_forward(20, 0.048):
                        self.set_speed(self.speed)
                        self.rotate_x_degrees(-80)
                    start_value = self.approach_wall()
            elif self.robot.get_rear_left_side_ir() > start_value + 0.003:
                print('too far from wall')
                self.rotate_x_degrees(5)
                self.reset_adjust()
            elif self.robot.get_rear_left_side_ir() < start_value - 0.003:
                print('too close to wall')
                self.rotate_x_degrees(-5)
                self.reset_adjust()

            print("forward: " + str(self.robot.get_rear_left_straight_ir()) + ", left side:" + str(self.robot.get_rear_left_side_ir()))
            self.adjust_speed(right_start, left_start, 1)

    def is_noise(self, value, sensor, max_error=0.005):
        """check if that is a noise."""
        self.set_speed(0)
        errors = 0
        print('is noise')
        for _ in range(10):
            sensor_value = sensor()
            print("new is " + str(sensor_value) + " old is " + str(value) + " difference is: " + str(
                abs(sensor_value - value)))
            if abs(sensor_value - value) > max_error:
                errors += 1
            rospy.sleep(0.05)
        if errors > 3:
            print('THAT WAS A NOISE, errors: ' + str(errors))
            return (False, value)
        print('FINE, THAT WAS NOT NOISE, errors: ' + str(errors))
        return (True, value)

    def reset_adjust(self):
        """reset wheel speed adjusts."""
        self.adjust_left = 0
        self.adjust_right = 0

    def approach_wall(self):
        """find the wall."""
        self.reset_adjust()

        self.rotate_x_degrees(45)

        left_start = self.robot.get_left_wheel_encoder()
        right_start = self.robot.get_right_wheel_encoder()

        while True:
            if self.robot.get_rear_left_straight_ir() < 0.03 or self.robot.get_rear_left_diagonal_ir() < 0.03:
                check = self.is_noise(self.robot.get_rear_left_diagonal_ir(), self.robot.get_rear_left_diagonal_ir)
                if check[0] is True and check[1] < 0.04:
                    self.set_speed(0)
                    self.rotate_x_degrees(-45)
                    return check[1]
            self.adjust_speed(right_start, left_start, 1)

    def main(self):
        """main funcition."""
        self.close_and_lift_grabber()
        start_value = self.approach_wall()
        self.wall_follow(start_value)


if __name__ == "__main__":
    robot = Robot()
    robot.main()
