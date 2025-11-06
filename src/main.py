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

controllerProfiles = []
controllerProfileNames = []
currentControllerProfile = 0
hasButtonRBeenPressed = False
hasButtonLBeenPressed = False
hasSelectedProfile = False
isSlowMode = False
isInAuto = False
clockCyclesPast = 0
intakeIsSpinning = False

def toggleSlowMode(): 
    global isSlowMode
    isSlowMode = not isSlowMode
def exitAutoRoutine():
    global currentAutoRoutine
    currentAutoRoutine.cancel()
def getCurProfileName():
    return controllerProfileNames[currentControllerProfile]
def runSelectProfile():
    global controller
    global currentControllerProfile
    global controllerProfiles
    global hasButtonRBeenPressed
    global hasButtonLBeenPressed
    global hasSelectedProfile
    global driveContoller
    
    if controller.buttonRight.pressing(): 
        if not hasButtonRBeenPressed: 
            currentControllerProfile +=1
        hasButtonRBeenPressed = True
    else: hasButtonRBeenPressed = False
    if controller.buttonLeft.pressing(): 
        if not hasButtonLBeenPressed: 
            currentControllerProfile -=1
        hasButtonLBeenPressed = True
    else: hasButtonLBeenPressed = False
        
    if currentControllerProfile >= len(controllerProfiles): currentControllerProfile = 0
    if currentControllerProfile < 0: currentControllerProfile = len(controllerProfiles)-1
    driveContoller.controllerProfile = controllerProfiles[currentControllerProfile]

    if controller.buttonA.pressing(): hasSelectedProfile = True

# Helper config class holding telemetry and control bindings
class ControllerProfile:
    """ Helper config class holding telemetry and control bindings
        :param Controller: The vex controller object whose inputs are read and supply 
    """
    ARCADE = "Arcade"
    TANK = "Tank"
    
    def __init__(self, controller, name):
        self.controller = controller
        self.axisOne = None
        self.axisTwo = None
        self.isAxisOneAbsolute = False
        self.isAxisTwoAbsolute = False
        self.driveMode = "Arcade" # Arcade or Tank
        self.telemetryLables = []
        self.telemetrySuppliers = []
        self.rumbleConditions = []
        self.pressButtons = []
        self.buttonsArePressed = []
        self.boundFunctions = []
        self.onPressOnlys = []
        self.onReleaseOnlys = []
        self.isSpintakeOn = False
        self.spintakeDirection = FORWARD
        self.buttonsHaveBeenPressed = []
        self.reBoundFunctions = []
        self.conditionalTelemetrySuppliers = []
        self.conditionalTelemetryLables = []
        self.conditionalTelemetryTriggers = []
        self.name = name
        self.addConditionalTelemetry("Current Profile: ", lambda: (not hasSelectedProfile), lambda: getCurProfileName())
        self.addConditionalTelemetry("Selected Profile", lambda: (hasSelectedProfile), lambda: getCurProfileName())
        global controllerProfiles
        global controllerProfileNames
        controllerProfiles.append(self)
        controllerProfileNames.append(self.name)

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
    
    def setAxisOneAbsolute(self):
        self.isAxisOneAbsolute = True
        return self
    
    def setAxisTwoAbsolute(self):
        self.isAxisTwoAbsolute = True
        return self
    
    def bindAutoRoutine(self, autoRoutine, button):
        """ Add a binding for a button and a given auto routine object
            :param autoRoutine: AutoRoutine object to be bound to the provided button
            :param Button: Button which when pressed will schedule the auto routine
            :return ControllerProfile object: 
        """
        self.bindButton(lambda: autoRoutine.schedule(), button)
        return self
    
    def bindSpintake(self, robotController, onButton, offButton, toggle):
        if toggle: 
            self.bindButton(lambda: self.toggleSpintake(robotController, FORWARD), onButton, True)
            self.bindButton(lambda: self.toggleSpintake(robotController, REVERSE), offButton, True)
        else:
            self.bindButton(lambda: self.spinSpintake(robotController, 255, FORWARD), onButton, True)
            self.bindButton(lambda: self.spinSpintake(robotController, 0, FORWARD), onButton, False, True)
            self.bindButton(lambda: self.spinSpintake(robotController, -255, FORWARD), offButton, True)
            self.bindButton(lambda: self.spinSpintake(robotController, 0, FORWARD), offButton, False, True)
        
        return self
    
    def toggleSpintake(self, robotController, direction):
        global intakeIsSpinning
        if direction != self.spintakeDirection: self.isSpintakeOn = False
        self.spintakeDirection = direction
        
        if(self.isSpintakeOn):
            robotController.driveSpinMotor(0, direction)
            self.isSpintakeOn = False
        else:
            robotController.driveSpinMotor(255, direction)
            self.isSpintakeOn = True
            
    def spinSpintake(self, robotController, speed, direction):
            robotController.driveSpinMotor(speed, direction)
            if speed is 0: self.isSpintakeOn = False
            else: self.isSpintakeOn = True
        
    
    def bindButton(self, callback, button, onPressOnly=True, onReleaseOnly=False):
        """ Add a binding for a button and a given lambda
            :param Callback: lambda to be bound to the provided button
            :param Button: Button which when pressed will schedule the auto routine
            :return ControllerProfile object: 
        """
        self.boundFunctions.append(callback)
        self.pressButtons.append(button)
        self.onPressOnlys.append(onPressOnly)
        self.onReleaseOnlys.append(onReleaseOnly)
        self.buttonsArePressed.append(False)
        self.buttonsHaveBeenPressed.append(False)
        return self
    
    def checkButtons(self):
        for i in range(len(self.boundFunctions)):
            if self.pressButtons[i].pressing(): 
                if not self.onReleaseOnlys[i] and (not self.onPressOnlys[i] or not self.buttonsArePressed[i]): 
                    self.boundFunctions[i]()
                self.buttonsArePressed[i] = True
            else:
                if self.onReleaseOnlys[i] and self.buttonsArePressed[i]:
                    self.boundFunctions[i]()
                self.buttonsArePressed[i] = False
                
    
    def addRumbleCondition(self, condition, pattern="_", duration=10.0):
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
        global clockCyclesPast
        
        for condition in self.rumbleConditions:
            if(condition[0]() and clockCyclesPast % 2 is 0):
                self.controller.rumble("..")
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
        self.controller.screen.clear_screen() # Clear the controller screen
        for i in range(len(self.telemetryLables)):
            self.controller.screen.set_cursor(i+1, 1)
            self.controller.screen.print(self.telemetryLables[i] + " " + str(self.telemetrySuppliers[i]()))
        return self

    def update(self):
        """ Update all conditional checks and display telemetry
            :return ControllerProfile object: 
        """
        global clockCyclesPast
        self.checkButtons()
        self.checkConditionalTelemetry()
        self.checkRumbleConditions()
        if (clockCyclesPast % 10) == 0: self.displayTelemetry()
        return self
   
# Subclass uesd to bind callback lamdbas to the press of a button   
class boundButton:
    """ Subclass uesd to bind callback lamdbas to the press of a button   
        :param Button: The vex controller button object to be bound to the callback
        :param Callback: Lambda to be bound to the button press
        :param onPressOnly: if the callback should be called while the button is held or only on the initial press
    """
    def __init__(self, button, callback, onPressOnly):
        self.button = button
        self.callback = callback
        self.onPressOnly = onPressOnly
        self.hasBeenPressed = False
    
    def update(self):
        """Handle the calling and interpretation of button presses provided to the boundButton"""
        if self.button.pressing():
            if not self.onPressOnly or not self.hasBeenPressed: self.callback()
            self.hasBeenPressed = True
        else: self.hasBeenPressed = False
        
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
    def __init__(self, leftMotor, reverseLeft, rightMotor, reverseRight, spinMotorLeft, reverseSpinLeft, spinMotorRight, reverseSpinRight):
        self.leftMotor = leftMotor
        self.reverseLeft = reverseLeft
        self.rightMotor = rightMotor
        self.reverseRight = reverseRight
        self.spinMotorLeft = spinMotorLeft
        self.reverseSpinLeft = reverseSpinLeft
        self.spinMotorRight = spinMotorRight
        self.reverseSpinRight = reverseSpinRight
        
    
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
        self.profile.leftMotor.spin(direction, (0.5 if isSlowMode else 1) * (speed if not self.profile.reverseLeft else -speed))
        return self
    
    def driveRightWheel(self, speed, direction):
        """ Drive the right wheel at a given speed and direction
            :param Speed: Value from 0 to 100 determining the % speed of the right motor
            :param Direction: Direction value that is also affected by the profile.reverseRight value
            :return RobotController object: 
        """
        self.profile.rightMotor.spin(direction, (0.5 if isSlowMode else 1) * (speed if not self.profile.reverseRight else -speed))
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
        self.profile.spinMotorLeft.spin(direction, (0.5 if isSlowMode else 1) * (speed if not self.profile.reverseSpinLeft else -speed))
        self.profile.spinMotorRight.spin(direction, (0.5 if isSlowMode else 1) * (speed if not self.profile.reverseSpinRight else -speed))
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
        
    # Update telemetry and run drive controlls with the provided controller and profile
    def update(self):
        """ Update telemetry and run drive controlls with the provided controller and profile"""
        global isInAuto
        self.controllerProfile.update()
    
        if isInAuto: return # Only run drive code if the robot is not running an auto routine
        
        # Drive logic
        if(self.controllerProfile.driveMode == "Arcade"):
            # Define values for forward and turning motion directly from the axes
            forward = self.controllerProfile.axisOne()*2.55
            turn = self.controllerProfile.axisTwo()*2.55
            
            if self.controllerProfile.isAxisOneAbsolute:
                if forward > 0: forward = 255
                elif forward < 0: forward = -255
                
            if self.controllerProfile.isAxisTwoAbsolute:
                if turn > 0: turn = 255
                elif turn < 0: turn = -255
            
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
            leftSpeed = self.controllerProfile.axisOne()*2.55
            rightSpeed = self.controllerProfile.axisTwo()*2.55
                        
            if self.controllerProfile.isAxisOneAbsolute:
                if leftSpeed > 0: leftSpeed = 255
                elif leftSpeed < 0: leftSpeed = -255
                
            if self.controllerProfile.isAxisTwoAbsolute:
                if rightSpeed > 0: rightSpeed = 255
                elif rightSpeed < 0: rightSpeed = -255
            
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
        global isInAuto
        isInAuto = True
        currentAutoRoutine = self
        return self
    # Cancel this auto routine and set the auto routine to do nothing
    def cancel(self): 
        """ Cancel this auto routine and set the auto routine to do nothing
            :return AutoRoutine object: 
        """
        global currentAutoRoutine
        global isInAuto
        isInAuto = False
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
    INTAKE = "Intake"
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
                    robotController.driveAllWheels(255, FORWARD)
                elif AutoCommand.DIRECTION_BACKWARD in self.decorators:
                    robotController.driveAllWheels(255, REVERSE)
            elif self.baseCommand == AutoCommand.TURN:
                if AutoCommand.DIRECTION_LEFT in self.decorators:
                    robotController.driveLeftWheel(255, REVERSE)
                    robotController.driveRightWheel(255, FORWARD)
                elif AutoCommand.DIRECTION_RIGHT in self.decorators:
                    robotController.driveLeftWheel(255, FORWARD)
                    robotController.driveRightWheel(255, REVERSE)
            elif self.baseCommand == AutoCommand.INTAKE:
                if AutoCommand.DIRECTION_FORWARD in self.decorators:
                    robotController.driveSpinMotor(255, FORWARD)
                elif AutoCommand.DIRECTION_BACKWARD in self.decorators:
                    robotController.driveSpinMotor(255, REVERSE)
            
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
robotController = RobotController(RobotProfile(Motor(Ports.PORT1), False, Motor(Ports.PORT2), True, Motor(Ports.PORT3), True, Motor(Ports.PORT4), False))

# define controller profiles

StefanProfile = ControllerProfile(controller, "Stefan").setDriveMode(ControllerProfile.ARCADE).bindAxisOne(controller.axis3).bindAxisTwo(controller.axis4).bindSpintake(robotController, controller.buttonR2, controller.buttonR1, False).bindButton(lambda: toggleSlowMode(), controller.buttonB, True).addTelemetry("Slowmode", lambda: isSlowMode)
BrianProfile = ControllerProfile(controller, "Brian").setDriveMode(ControllerProfile.ARCADE).bindAxisOne(controller.axis3).bindAxisTwo(controller.axis1).bindSpintake(robotController, controller.buttonL2, controller.buttonL1, True)
AllanProfile = ControllerProfile(controller, "Allan").setDriveMode(ControllerProfile.ARCADE).bindAxisOne(controller.axis3).bindAxisTwo(controller.axis1).bindSpintake(robotController, controller.buttonR1, controller.buttonR2, True).bindButton(lambda: toggleSlowMode(), controller.buttonL1, True).addTelemetry("Slowmode", lambda: isSlowMode)
AllanProfile.addRumbleCondition(lambda: AllanProfile.isSpintakeOn, ".", 0.1)
WillProfile = ControllerProfile(controller, "Will").setDriveMode(ControllerProfile.ARCADE).bindAxisOne(controller.axis3).bindAxisTwo(controller.axis1).bindSpintake(robotController, controller.buttonR1, controller.buttonR2, True)
WillProfile.addRumbleCondition(lambda: WillProfile.isSpintakeOn, ".", 0.1)
OzzyProfile = ControllerProfile(controller, "Ozzy").setDriveMode(ControllerProfile.ARCADE).bindAxisOne(controller.axis2).bindAxisTwo(controller.axis4).bindSpintake(robotController, controller.buttonR1, controller.buttonR2, True).bindButton(lambda: toggleSlowMode(), controller.buttonL1, True).addTelemetry("Cortex Battery", brain.battery.capacity)
AxelProfile = ControllerProfile(controller, "Axel").setDriveMode(ControllerProfile.ARCADE).bindAxisOne(controller.axis3).bindAxisTwo(controller.axis1).bindSpintake(robotController, controller.buttonR1, controller.buttonR2, False).bindButton(lambda: toggleSlowMode(), controller.buttonX, True).addTelemetry("Cortex Battery", brain.battery.capacity).addTelemetry("Slowmode", lambda: isSlowMode)
AxelProfile.addRumbleCondition(lambda: AxelProfile.isSpintakeOn, ".", 0.1)
#defaultArcadeProfile = ControllerProfile(controller, "Arcade").setDriveMode(ControllerProfile.ARCADE).bindAxisOne(controller.axis3).bindAxisTwo(controller.axis1)
#defaultTankProfile = ControllerProfile(controller, "Tank").setDriveMode(ControllerProfile.TANK).bindAxisOne(controller.axis3).bindAxisTwo(controller.axis2)

currentProfile = StefanProfile

# initialize helper classes
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
        clockCyclesPast += 1 
        if not hasSelectedProfile: runSelectProfile() # allow drivers to select their controller proifiles
        driveContoller.update()
        cortexTelemetry.displayTelemetry(brain)
        currentAutoRoutine.runAuto()
    except Exception as e:
        print("An Error Occured:")
        print(e.with_traceback)