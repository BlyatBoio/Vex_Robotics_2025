# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Stefan                                                       #
# 	Created:      9/23/2025, 9:30:45 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
from ControllerHelpers import *

# Brain should be defined by default
brain=Brain()

# nescesary objects
controller = Controller(PRIMARY);

# define controller profiles
stefanProfile = ControllerProfile(controller).setDriveMode("Tank").bindAxisOne(controller.axis2).bindAxisTwo(controller.axis3).addTelemetry("Cortex Battery:", lambda: brain.battery.voltage())
currentProfile = stefanProfile

# initialize helper classes
robotController =RobotController(RobotProfile(Motor(1), False, Motor(2), False, Motor(3), False, Motor(4), False))
driveContoller = DriveContoller(currentProfile, robotController)

# Main loop
while True:
    driveContoller.update()
    wait(5, MSEC)
