from wpilib.interfaces import GenericHID
from wpilib import Joystick
from wpilib import XboxController
from commands2.button import CommandXboxController, Trigger
from commands2 import Command, ParallelDeadlineGroup, Subsystem
from commands2 import InstantCommand, ConditionalCommand, WaitCommand, PrintCommand, RunCommand, SequentialCommandGroup, ParallelCommandGroup, WaitUntilCommand, StartEndCommand, DeferredCommand
from commands2.button import JoystickButton
import commands2.cmd as cmd
from CTREConfigs import CTREConfigs
from commands2 import CommandScheduler
import math


from wpimath.geometry import *
import wpimath.units as Units
from wpimath.kinematics import *
import lib.mathlib.Units as CustomUnits
from lib.mathlib.Conversions import Conversions
from Constants import Constants

from commands.TeleopSwerve import TeleopSwerve
from commands.TurnInPlace import TurnInPlace

from subsystems.Swerve import Swerve
from subsystems.Vision import Vision
from commands.SysId import DriveSysId
from commands.PathFindToTag import PathFindToTag

from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets, BuiltInLayouts
from wpilib import SendableChooser, RobotBase, DriverStation

from wpimath import applyDeadband

from pathplannerlib.auto import NamedCommands, PathConstraints, AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController


class RobotContainer:
    ctreConfigs = CTREConfigs()
    # Drive Controls
    translationAxis = XboxController.Axis.kLeftY
    strafeAxis = XboxController.Axis.kLeftX
    rotationAxis = XboxController.Axis.kRightX
    slowAxis = XboxController.Axis.kRightTrigger # This causes issues on certain controllers, where kRightTrigger is for some reason mapped to [5] instead of [3]

    driver = CommandXboxController(0)
    operator = CommandXboxController(1)

    sysId = JoystickButton(driver, XboxController.Button.kY)

    robotCentric_value = False

    # Subsystems
    s_Swerve : Swerve = Swerve()
    # s_Vision : Vision = Vision.getInstance()

    #SysId
    driveSysId = DriveSysId(s_Swerve)
    # pathFind = PathFindToTag(s_Swerve, s_Vision, 18, 10)
    # The container for the robot. Contains subsystems, OI devices, and commands.
    def __init__(self):
        # Configure driver controls
        self.zeroGyro = self.driver.back()
        self.robotCentric = self.driver.start()

        self.fastTurn = self.driver.povDown()
        self.coralStation = self.driver.leftTrigger()

        self.toPos = self.driver.x()

        self.configureButtonBindings()

        self.auton_selector = AutoBuilder.buildAutoChooser("DO NOTHING")

        Shuffleboard.getTab("Autonomous").add("Auton Selector", self.auton_selector)

        Shuffleboard.getTab("Teleoperated").addBoolean("Field Oriented", self.getFieldOriented)
        Shuffleboard.getTab("Teleoperated").addBoolean("Zero Gyro", self.zeroGyro.getAsBoolean)
        Shuffleboard.getTab("Teleoperated").addDouble("Swerve Heading", lambda: self.s_Swerve.getHeading().degrees())
        Shuffleboard.getTab("Teleoperated").addDouble("Front Right Module Speed", lambda: self.s_Swerve.mSwerveMods[1].getState().speed)

        Shuffleboard.getTab("Teleoperated").addDouble("Swerve Pose X", lambda: self.s_Swerve.getPose().X())
        Shuffleboard.getTab("Teleoperated").addDouble("Swerve Pose Y", lambda: self.s_Swerve.getPose().Y())
        Shuffleboard.getTab("Teleoperated").addDouble("Swerve Pose Theta", lambda: self.s_Swerve.getPose().rotation().degrees())
        
        Shuffleboard.getTab("Teleoperated").addBoolean("PATH FLIP", self.s_Swerve.shouldFlipPath)
        Shuffleboard.getTab("Teleoperated").addString("FMS ALLIANCE", self.getAllianceName)

    def getAllianceName(self):
        if DriverStation.getAlliance() is None:
            return "NONE"
        else:
            return DriverStation.getAlliance().name

    def configureButtonBindings(self):
        translation = lambda: -applyDeadband(self.driver.getRawAxis(self.translationAxis), 0.1)
        strafe = lambda: -applyDeadband(self.driver.getRawAxis(self.strafeAxis), 0.1)
        rotation = lambda: -applyDeadband(self.driver.getRawAxis(self.rotationAxis), 0.1)
        robotcentric = lambda: applyDeadband(self.robotCentric_value, 0.1)
        slow = lambda: applyDeadband(self.driver.getRawAxis(self.slowAxis), 0.1)
        # slow = lambda: 0.0

        self.s_Swerve.setDefaultCommand(
            TeleopSwerve(
                self.s_Swerve, 
                translation,
                strafe,
                rotation,
                robotcentric,
                slow
            )
        )

        # Driver Buttons
        self.zeroGyro.onTrue(InstantCommand(lambda: self.s_Swerve.zeroHeading()))
        self.robotCentric.onTrue(InstantCommand(lambda: self.toggleFieldOriented()))
        
        turnInPlaceCmd = TurnInPlace(
            self.s_Swerve,
            lambda: Rotation2d.fromDegrees(
                # self.m_robotState.m_gameState.getNextShotRobotAngle()
            ),
            translation,
            strafe,
            rotation,
            robotcentric
        )

        Shuffleboard.getTab("Teleoperated").addBoolean("TURN PID ON TARGET", lambda: turnInPlaceCmd.turnPID.atSetpoint())

        self.fastTurn.whileTrue(InstantCommand(lambda: self.setFastTurn(True))).whileFalse(InstantCommand(lambda: self.setFastTurn(False)))

    def toggleFieldOriented(self):
        self.robotCentric_value = not self.robotCentric_value

    def getFieldOriented(self):
        return not self.robotCentric_value

    def rumbleAll(self):
        return ParallelCommandGroup(
            self.rumbleDriver(),
            self.rumbleOperator()
        )

    def rumbleDriver(self):
        return InstantCommand(
            lambda: self.driver.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 1.0)
        ).andThen(WaitCommand(0.5)).andThen(InstantCommand(lambda: self.driver.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 0.0))).withName("Rumble")
    
    def rumbleOperator(self):
        return InstantCommand(
            lambda: self.operator.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 1.0)
        ).andThen(WaitCommand(0.5)).andThen(InstantCommand(lambda: self.operator.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 0.0))).withName("Rumble")

    """
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
    """
    def getAutonomousCommand(self) -> Command:
        auto = self.auton_selector.getSelected()
        # auto = None
        return auto
    
    def setFastTurn(self, value: bool):
        if value:
            Constants.Swerve.maxAngularVelocity = 2.5 * math.pi * 2.0
        else:
            Constants.Swerve.maxAngularVelocity = 2.5 * math.pi
