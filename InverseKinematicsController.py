import numpy as np

import robosuite as suite
import robosuite.utils.transform_utils as T
from robosuite.devices import *
from robosuite.models.robots import *
from robosuite.robots import *
from robosuite.controllers.ik import PyBulletServer
import pybullet as p

def choose_controller():
    """
    Prints out controller options, and returns the requested controller name
    Returns:
        str: Chosen controller name
    """
    # get the list of all controllers
    controllers_info = suite.controllers.CONTROLLER_INFO
    controllers = list(suite.ALL_CONTROLLERS)

    # Select controller to use
    print("Here is a list of controllers in the suite:\n")

    for k, controller in enumerate(controllers):
        print("[{}] {} - {}".format(k, controller, controllers_info[controller]))
    print()
    try:
        s = input("Choose a controller for the robot " + "(enter a number from 0 to {}): ".format(len(controllers) - 1))
        # parse input into a number within range
        k = min(max(int(s), 0), len(controllers) - 1)
    except:
        k = 0
        print("Input is not valid. Use {} by default.".format(controllers)[k])

    # Return chosen controller
    return controllers[k]

print(choose_controller())
