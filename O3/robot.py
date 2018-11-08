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


set_speed(20)
while True:
    print(get_lenc)
    rospy.sleep(0.01)
