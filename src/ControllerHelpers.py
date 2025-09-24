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

# Helper config class holding telemetry and control bindings
class ControllerProfile:
    def __init__(self, controller):
        self.controller = controller
        self.leftAxis = None
        self.rightAxis = None
        self.driveMode = "arcade" # Arcade or Tank
        self.telemetryLables = [];
        self.telemetrySuppliers = [];
        return self

    # Set drive mode to either Arcade or tank
    def setDriveMode(self, mode):
        self.driveMode = mode
        return self

    # Bind the left or directional movement axis
    def bindAxisOne(self, axis):
        self.leftAxis = lambda: axis.position()
        return self
    
    # Bind the right or turning movement axis
    def bindAxisTwo(self, axis):
        self.leftAxis = lambda: axis.position()
        return self
    
    # Add a telemetry label and a supplier function to get the value
    def addTelemetry(self, label, valueSupplier):
        self.telemetryLables.append(label)
        self.telemetrySuppliers.append(valueSupplier)
        return self
       
    # Display all telemetry on the controller screen 
    def displayTelemetry(self):
        self.controller.screen.clear_screen()
        for i in range(len(self.telemetryLables)):
            self.controller.screen.set_cursor(1, i+1)
            self.controller.screen.print(self.telemetryLables[i] + " " + str(self.telemetrySuppliers[i]()))
        return self

# Robot profile holding config for motors and sensors
class RobotProfile:
    def __init__(self, leftMotor, reverseLeft, rightMotor, reverseRight, leftBucketMotor, reverseLeftBucket, rightBucketMotor, reverseRightBucket):
        self.leftMotor = leftMotor
        self.reverseLeft = reverseLeft
        self.rightMotor = rightMotor
        self.reverseRight = reverseRight
        self.leftBucketMotor = leftBucketMotor
        self.reverseLeftBucket = reverseLeftBucket
        self.rightBucketMotor = rightBucketMotor
        self.reverseRightBucket = reverseRightBucket
        return self

# Helper class to control the robot's movement given a robot profile
class RobotController:
    def __init__(self, profile):
        self.profile = profile
        return self
    
    def driveLeftWheel(self, speed, direction):
        # Adjust speed based on motor direction configuration
        self.profile.leftMotor.spin(direction, speed if not self.profile.reverseLeft else -speed, PERCENT)
        return self
    
    def driveRightWheel(self, speed, direction):
        # Adjust speed based on motor direction configuration
        self.profile.rightMotor.spin(direction, speed if not self.profile.reverseRight else -speed, PERCENT)
        return self
    
    def driveAllWheels(self, speed, direction):
        # Adjust speed based on motor direction configuration
        self.driveLeftWheel(speed if not self.profile.reverseLeft else -speed, direction)
        self.driveRightWheel(speed if not self.profile.reverseRight else -speed, direction)
        return self

# Main drive class handles driving logic and telemetry updates
class DriveContoller:
    def __init__(self, controllerProfile, robotController):
        self.controllerProfile = controllerProfile
        self.robotController = robotController
        return self
    
    def update(self):
        self.controllerProfile.displayTelemetry() # Update telemetry display
        
        # Drive logic
        if(self.controllerProfile.driveMode == "Arcade"):
            # Define values for forward and turning motion directly from the axes
            forward = self.controllerProfile.leftAxis()
            turn = self.controllerProfile.rightAxis()
            
            # Calculate wheel speeds and directions
            leftSpeed = forward + turn
            rightSpeed = forward - turn
            leftDirection = FORWARD if leftSpeed >= 0 else REVERSE
            rightDirection = FORWARD if rightSpeed >= 0 else REVERSE
            
            # Apply speeds and directions to motors
            self.robotController.driveLeftWheel(abs(leftSpeed), leftDirection)
            self.robotController.driveRightWheel(abs(rightSpeed), rightDirection)
            
        elif(self.controllerProfile.driveMode == "Tank"):
            # Get speeds and directions directly from the axes
            leftSpeed = self.controllerProfile.leftAxis()
            rightSpeed = self.controllerProfile.rightAxis()
            leftDirection = FORWARD if leftSpeed >= 0 else REVERSE
            rightDirection = FORWARD if rightSpeed >= 0 else REVERSE
            
            # Apply speeds and directions to motors
            self.robotController.driveLeftWheel(abs(leftSpeed), leftDirection)
            self.robotController.driveRightWheel(abs(rightSpeed), rightDirection)
