import rospy
from PiBot import PiBot

robot = PiBot()

# TODO: function that scans the field for the object
# TODO: function that moves towards the object until required distance

def scan_for_object():
    pass
    # TODO: implement a way of finding the object, perhaps based on difference of sensors.
    # Just one sensor or all three sensors? Use a buffer?
    # Prolly do full circle? Or not?
    # If sensor step 1 is far, step 2 is closer, step 3 is far
    # then step 2 has object. Prolly 10cm difference is okay?
