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
    """ Helper config class holding telemetry and control bindings
        :param Controller: The vex controller object whose inputs are read and supply 
    """
    ARCADE = "Arcade"
    TANK = "Tank"
    
    def __init__(self, controller):
        self.controller = controller
        self.axisOne = None
        self.axisTwo = None
        self.driveMode = "Arcade" # Arcade or Tank
        self.telemetryLables = []
        self.telemetrySuppliers = []
        self.rumbleConditions = []
        self.conditionalTelemetrySuppliers = []
        self.conditionalTelemetryLables = []
        self.conditionalTelemetryTriggers = []

    def setDriveMode(self, mode):
        """ Set drive mode to either Arcade or tank
            :param Mode: Controller profile constant, either TANK or ARCADE
            :return ControllerProfile object: 
        """
        self.driveMode = mode
        return self

    def bindAxisOne(self, axis):
        """ Bind the left or directional movement axis
            :param Mode: Axis to assign as the first axis
            :return ControllerProfile object: 
        """
        self.axisOne = lambda: axis.position()
        return self
    
    def bindAxisTwo(self, axis):
        """ Bind the right or turning movement axis
            :param Mode: Axis to assign as the second axis
            :return ControllerProfile object: 
        """
        self.axisTwo = lambda: axis.position()
        return self
    
    def bindAutoRoutine(self, autoRoutine, button):
        """ Add a binding for a button and a given auto routine object
            :param autoRoutine: AutoRoutine object to be bound to the provided button
            :param Button: Button which when pressed will schedule the auto routine
            :return ControllerProfile object: 
        """
        self.bindButton(lambda: autoRoutine.schedule(), button)
        return self
    
    def bindButton(self, callback, button):
        """ Add a binding for a button and a given lambda
            :param Callback: lambda to be bound to the provided button
            :param Button: Button which when pressed will schedule the auto routine
            :return ControllerProfile object: 
        """
        button.pressed(callback)
        return self
    
    def addRumbleCondition(self, condition, pattern=".", duration=200):
        """ Define a condition on which the controller will vibrate with a given pattern
            :param Condition: Lambda that provides a boolean outcome for when to rumble the controller
            :param Pattern: String seqence of .'s and _'s defining length of a rumble
            :param Duration: The duration of which to rumble the controller when the condition is met
            :return ControllerProfile object: 
        """
        self.rumbleConditions.append([condition, pattern, duration])
        return self
    
    def checkRumbleConditions(self):
        """ Check all added rumble conditions and rumble the controller if a condition is met
            :return ControllerProfile object: 
        """
        for condition in self.rumbleConditions:
            if(condition[0]()):
                self.controller.rumble(condition[1])
        return self
    
    def addConditionalTelemetry(self, label, condition, valueSupplier=lambda: ""):
        """ Define a piece of telemetry that is drawn when a condition is met
            :param Label: String which will be displayed along the telemtry value itself
            :param Condition: Lambda that provides a boolean outcome for when to display the telemtry
            :param valueSupplier: Lambda that supplies the value which will be displayed
            :return ControllerProfile object: 
        """
        self.conditionalTelemetryLables.append(label)
        self.conditionalTelemetrySuppliers.append(valueSupplier)
        self.conditionalTelemetryTriggers.append(condition)
        return self
    
    def checkConditionalTelemetry(self):
        """ Check conditions and if any are met, display their telemetry
            :return ControllerProfile object: 
        """
        totalConditionsDrawn = 0
        for i in range(len(self.conditionalTelemetryTriggers)):
            if(self.conditionalTelemetryTriggers[i]()):
                totalConditionsDrawn += 1
                self.controller.screen.set_cursor(len(self.telemetryLables)+totalConditionsDrawn, 1)
                self.controller.screen.print(self.conditionalTelemetryLables[i] + " " + str(self.conditionalTelemetrySuppliers[i]()))
        return self
    
    def addTelemetry(self, label, valueSupplier):
        """ Add a telemetry label and a supplier function to get the value
            :param Label: String which will be displayed along the telemtry value itself
            :param valueSupplier: Lambda that supplies the value which will be displayed
            :return ControllerProfile object: 
        """
        self.telemetryLables.append(label)
        self.telemetrySuppliers.append(valueSupplier)
        return self
       
    def displayTelemetry(self):
        """ Display all telemetry on the controller screen
            :return ControllerProfile object: 
        """
        for i in range(len(self.telemetryLables)):
            self.controller.screen.set_cursor(1, i+1)
            self.controller.screen.print(self.telemetryLables[i] + " " + str(self.telemetrySuppliers[i]()))
        return self

    def update(self):
        """ Update all conditional checks and display telemetry
            :return ControllerProfile object: 
        """
        self.checkConditionalTelemetry()
        self.checkRumbleConditions()
        self.displayTelemetry()
        return self
        
# Robot profile holding config for motors and sensors
class RobotProfile:
    """ Robot profile holding config for motors and sensors
        :param leftMotor: Vex motor located on the left side of the robot
        :param reverseLeft: Boolean determining if the left motor should spin inversed or not
        :param rightMotor: Vex motor located on the right side of the robot
        :param reverseRight: Boolean determining if the right motor should spin inversed or not
        :param spinMotor: Vex motor used to run the spintake
        :param reverseSpin: Boolean determining if the spin motor should spin inversed or not
    """
    def __init__(self, leftMotor, reverseLeft, rightMotor, reverseRight, spinMotor, reverseSpin):
        self.leftMotor = leftMotor
        self.reverseLeft = reverseLeft
        self.rightMotor = rightMotor
        self.reverseRight = reverseRight
        self.spinMotor = spinMotor
        self.reverseSpin = reverseSpin
    
isSlowMode = False
def toggleSlowMode(): 
    global isSlowMode
    isSlowMode = not isSlowMode
# Helper class to control the robot's movement given a robot profile
class RobotController:
    """ Helper class to control the robot's movement given a robot profile
        :param Profile: Robot profile object defining motor objects and their inversions
    """
    def __init__(self, profile):
        self.profile = profile
    
    def driveLeftWheel(self, speed, direction):
        """ Drive the left wheel at a given speed and direction
            :param Speed: Value from 0 to 100 determining the % speed of the left motor
            :param Direction: Direction value that is also affected by the profile.reverseLeft value
            :return RobotController object: 
        """
        self.profile.leftMotor.spin(direction, (isSlowMode if 0.5 else 1) * (speed if not self.profile.reverseLeft else -speed), PERCENT)
        return self
    
    def driveRightWheel(self, speed, direction):
        """ Drive the right wheel at a given speed and direction
            :param Speed: Value from 0 to 100 determining the % speed of the right motor
            :param Direction: Direction value that is also affected by the profile.reverseRight value
            :return RobotController object: 
        """
        self.profile.rightMotor.spin(direction, (isSlowMode if 0.5 else 1) * (speed if not self.profile.reverseRight else -speed), PERCENT)
        return self
    
    def driveAllWheels(self, speed, direction):
        """ Drive the both wheels at a given speed and direction
            :param Speed: Value from 0 to 100 determining the % speed of both motors
            :param Direction: Direction value that is also affected by the profile.reverseLeft and profile.reverseRight value
            :return RobotController object: 
        """
        self.driveLeftWheel(speed if not self.profile.reverseLeft else -speed, direction)
        self.driveRightWheel(speed if not self.profile.reverseRight else -speed, direction)
        return self
    def driveSpinMotor(self, speed, direction):
        """ Drive the the spintake at a given speed and direction
            :param Speed: Value from 0 to 100 determining the % speed of both motors
            :param Direction: Direction value that is also affected by the profile.reverseLeft and profile.reverseRight value
            :return RobotController object: 
        """
        self.profile.spinMotor.spin((isSlowMode if 0.5 else 1) * (speed if not self.profile.reverseSpin else -speed), direction)
        return self

# Main drive class handles driving logic and telemetry updates
class DriveContoller:
    """ Main drive class handles driving logic and telemetry updates
        :param controllerProfile: Controller Profile object for controller input refference
        :param robotController: Robot Controller object required to run motors relative to the controller input
    """
    def __init__(self, controllerProfile, robotController):
        self.controllerProfile = controllerProfile
        self.robotController = robotController
        self.controllerProfile.bindButton(self.robotController.driveSpinMotor(100, FORWARD), self.controllerProfile.controller.buttonR1)
        self.controllerProfile.bindButton(self.robotController.driveSpinMotor(100, REVERSE), self.controllerProfile.controller.buttonR2)
    
    # Update telemetry and run drive controlls with the provided controller and profile
    def update(self):
        """ Update telemetry and run drive controlls with the provided controller and profile"""
        self.controllerProfile.controller.screen.clear_screen() # Clear the controller screen
        self.controllerProfile.displayTelemetry() # Update telemetry display
        self.controllerProfile.checkRumbleConditions() # Check rumble conditions
        self.controllerProfile.checkConditionalTelemetry() # Check conditional telemetry
        
        
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

# Class holding an array of AutoCommands that can run through each command sequentially
class AutoRoutine:
    """ Class holding an array of AutoCommands that can run through each command sequentially
        :param robotController: Robot Contoller object for running auto commands
    """

    def __init__(self, robotController):
        self.commands = []
        self.currentCommand = 0
        self.isFinishedRunning = False
        self.robotController = robotController
    
    # Add an auto command
    def addCommand(self, baseCommand, decorators, duration):
        """ Add an auto command
            :param baseCommand: AutoCommands value either Drive or Turn
            :param Decorators: Array of AutoCommands values defining direction and motion
            :param Duration: Integer defining how long to run the given command
            :return AutoRoutine object: 
        """
        self.commands.append(AutoCommand(baseCommand, decorators, duration))
        return self
        
    # Itterate over all commands until there are none left, then cancel
    # If there are no arguments, the auto never cancels as the default after cancel is an empty auto routine
    def runAuto(self):
        """ Itterate over all commands until there are none left, then cancel
            :return AutoRoutine object: 
        """
        if len(self.commands) == 0: return
        if not self.isFinishedRunning:
            self.commands[self.currentCommand].run(self.robotController)
            if self.commands[self.currentCommand].isFinished(): self.currentCommand += 1
            if self.currentCommand > len(self.commands): self.isFinishedRunning = True
        else: self.cancel()
        return self
            
    # Set the current auto routine to itself
    def schedule(self): 
        """ Set the current auto routine to itself
            :return AutoRoutine object: 
        """
        global currentAutoRoutine
        currentAutoRoutine = self
        return self
    # Cancel this auto routine and set the auto routine to do nothing
    def cancel(self): 
        """ Cancel this auto routine and set the auto routine to do nothing
            :return AutoRoutine object: 
        """
        global currentAutoRoutine
        currentAutoRoutine = AutoRoutine(self.robotController)
        return self
 
# Class that takes in a base command and decroators and runs said command with said decorators       
class AutoCommand:
    """ Class that takes in a base command and decroators and runs said command with said decorators
        :param baseCommand: AutoCommands value either Drive or Turn
        :param Decorators: Array of AutoCommands values defining direction and motion
        :param Duration: Integer defining how long to run the given command
    """      
    
    # Shoutout vex + python for not allowing imports on the cortex but making enums an importable instead of a native capability
    DRIVE = "Drive"
    TURN = "Turn"
    DIRECTION_LEFT = "Left"
    DIRECTION_RIGHT = "Right"
    DIRECTION_FORWARD = "Forward"
    DIRECTION_BACKWARD = "Backward"
     
    def __init__(self, baseCommand, decorators, duration):
        self.baseCommand = baseCommand
        self.decorators = decorators
        self.duration = duration
        self.runtime = 0
    
    # Run the given command
    def run(self, robotController):
        """ Run the given command
            :param robotController: Robot Controller object on which the auto routine is run
        """
        self.runtime += 1
        if self.runtime < self.duration:
            if self.baseCommand == AutoCommand.DRIVE:
                if AutoCommand.DIRECTION_FORWARD in self.decorators:
                    robotController.driveAllWheels(100, FORWARD)
                elif AutoCommand.DIRECTION_BACKWARD in self.decorators:
                    robotController.driveAllWheels(100, REVERSE)
            elif self.baseCommand == AutoCommand.TURN:
                if AutoCommand.DIRECTION_LEFT in self.decorators:
                    robotController.driveLeftWheel(100, REVERSE)
                    robotController.driveRightWheel(100, FORWARD)
                elif AutoCommand.DIRECTION_RIGHT in self.decorators:
                    robotController.driveLeftWheel(100, FORWARD)
                    robotController.driveRightWheel(100, REVERSE)
            
    # Return whether or not the command's duration has exceeded the runtime
    def isFinished(self):
        """ Return whether or not the command's duration has exceeded the runtime"""
        return self.runtime > self.duration

# Helper class to manage and display cortex telemetry
class CortexTelemetry:
    """ Helper class to manage and display cortex telemetry"""
    def __init__(self):
        self.telemetryLables = []
        self.telemetrySuppliers = []
 
    # Add the cortex's battery telemetry
    def addCortexBattery(self):
        """ Add the cortex's battery telemetry
            :return CortexTelemetry object: 
        """
        self.telemetryLables.append("Cortex Battery:")
        self.telemetrySuppliers.append(lambda: brain.battery.capacity())
        return self
 
    #Add the a given motor's temperature labled by the motor name
    def addMotorTemperature(self, motor, motorName=""):
        """ Add the a given motor's temperature labled by the motor name
            :param Motor: The vex motor object from which to pull the temperature
            :param motorName: Label to display the temperature under in the telemetry
            :return CortexTelemetry object: 
        """
        self.telemetryLables.append(motorName + " Temperature:")
        self.telemetrySuppliers.append(lambda: motor.temperature())
        return self
    
    # Display all telemetry values on the cortex's screen
    def displayTelemetry(self, cortex):
        """ Display all telemetry values on the cortex's screen
            :param Cortex: Vex brain object on which the telemetry is drawn
        """
        for i in range(len(self.telemetryLables)):
            cortex.screen.set_cursor(1+i, 1)
            cortex.screen.print(self.telemetryLables[i] + " " + str(self.telemetrySuppliers[i]()))
    
# Helper class to log provided values to the python output terminal
class Logger:
    """ Helper class to log provided values to the python output terminal"""
    
    def __init__(self):
        self.printValues = []
        self.pastValues = []
    
    # Add a value to log in the terminal with a given condition
    def addLogValue(self, valueSupplier, condition=lambda: ""):
        """ Add a value to log in the terminal with a given condition
            :param valueSupplier: Lambda that supplies the value which will be printed
            :param Condition: Lambda that determines when the value will be printed
            :return Logger object:
        """
        self.printValues.append([valueSupplier, condition])
        return self
    
    # Add a value to log in the terminal when that value changes
    def addLogValueOnChange(self, valueSupplier):
        """ Add a value to log in the terminal when that value changes
            :param valueSupplier: Lambda that supplies the value which will be printed
            :return Logger object:
        """
        self.pastValues.append(None)
        self.printValues.append([valueSupplier, lambda: (self.pastValues[len(self.pastValues)-1] != valueSupplier())])
        return self

    # Check all conditions and log all added values
    def update(self):
        """ Check all conditions and log all added values
            :return Logger object:
        """
        for value in self.printValues:
            if value[1](): print(value[0]())

        for i in range (len(self.pastValues)):
            self.pastValues[i] = self.printValues[i][0]()
        
# Brain should be defined by default
brain = Brain()

# nescesary objects
controller = Controller(PRIMARY)
logger = Logger()

# define controller profiles
defaultArcadeProfile = ControllerProfile(controller).setDriveMode(ControllerProfile.ARCADE).bindAxisOne(controller.axis3).bindAxisTwo(controller.axis1).bindButton(lambda: toggleSlowMode(), controller.buttonB)
defaultTankProfile = ControllerProfile(controller).setDriveMode(ControllerProfile.TANK).bindAxisOne(controller.axis3).bindAxisTwo(controller.axis1)
currentProfile = defaultArcadeProfile

# initialize helper classes
robotController = RobotController(RobotProfile(Motor(Ports.PORT1), False, Motor(Ports.PORT2), True, Motor(Ports.PORT3), False))
driveContoller = DriveContoller(currentProfile, robotController)
cortexTelemetry = CortexTelemetry().addCortexBattery()

# define auto routine variables
currentAutoRoutine = AutoRoutine(robotController)
testAuto = AutoRoutine(robotController)
testAuto.addCommand(AutoCommand.DRIVE, [AutoCommand.DIRECTION_FORWARD], 100)

# remove basic telemetry on the controller
controller.screen.clear_screen()

# Main loop
while True:
    try:    
        driveContoller.update()
        cortexTelemetry.displayTelemetry(brain)
        currentAutoRoutine.runAuto()
        wait(120, MSEC) # normalize timestep (telemetry will flicker without this)
    except Exception as e:
        print("An Error Occured: {e}") 