import rospy
from PiBot import PiBot


class Robot:
    """
    The three primitives robotic paradigm: sense, plan, act [1].
    Sense - gather information using the sensors
    Plan - create a world model using all the information and plan the next move
    Act - carry out the next step of the plan using actuators
    [1] https://en.wikipedia.org/wiki/Robotic_paradigm
    """

    def __init__(self):
        print("Initializing...")
        self.robot = PiBot()
        self.start_timestamp = rospy.get_time()
        self.problem_solved = False
        self.goal = 123
        self.state = "working"

    def sense(self):
        print("Sensing...")

    def plan(self):
        print("Planning...")
        if self.state == "working":
            print("Working...")
            self.left_speed = 15
            self.right_speed = 15

    def act(self):
        print("Acting...")
        self.robot.set_left_wheel_speed(self.left_speed)
        self.robot.set_left_wheel_speed(self.left_speed)

    def main(self):
        while not self.problem_solved and rospy.get_time() - self.start_timestamp < 180:
            self.sense()
            self.plan()
            self.act()
            rospy.sleep(0.05)  # 20 Hz
        if self.problem_solved:
            print("Solved! Good job, robot!")
        else:
            print("Unable to solve! :(")


if __name__ == "__main__":
    robot = Robot()
    robot.main()
