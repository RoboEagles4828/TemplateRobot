from wpilib import XboxController
from commands2.button import CommandXboxController
from commands2 import Command
from commands2 import InstantCommand, WaitCommand, SequentialCommandGroup, ParallelCommandGroup, WaitUntilCommand
from commands2.button import JoystickButton
from CTREConfigs import CTREConfigs
import math

from wpimath.geometry import *
from constants import Constants

from commands.TeleopSwerve import TeleopSwerve

from subsystems.Swerve import Swerve
from subsystems.Vision import Vision

from commands.TurnInPlace import TurnInPlace

from commands.SysId import DriveSysId

from wpilib.shuffleboard import Shuffleboard
from wpilib import DriverStation

from wpimath import applyDeadband

from pathplannerlib.auto import NamedCommands, AutoBuilder


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

    # The container for the robot. Contains subsystems, OI devices, and commands.
    def __init__(self):
        # Driver Controls
        self.zeroGyro = self.driver.back()
        self.robotCentric = self.driver.start()

        self.fastTurn = self.driver.povUp()
        # Slowmode is defined with the other Axis objects

        # Operator Controls
        self.autoHome = self.operator.rightTrigger()

        self.configureButtonBindings()
        
        # self.auton_selector = AutoBuilder.buildAutoChooser("DO NOTHING")

        # Shuffleboard.getTab("Autonomous").add("Auton Selector", self.auton_selector)

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

    """
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    """
    def configureButtonBindings(self):
        translation = lambda: -applyDeadband(self.driver.getRawAxis(self.translationAxis), 0.1)
        strafe = lambda: -applyDeadband(self.driver.getRawAxis(self.strafeAxis), 0.1)
        rotation = lambda: applyDeadband(self.driver.getRawAxis(self.rotationAxis), 0.1)
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
        # auto = self.auton_selector.getSelected()
        auto = None
        return auto
    
    def setFastTurn(self, value: bool):
        if value:
            Constants.Swerve.maxAngularVelocity = 2.5 * math.pi * 2.0
        else:
            Constants.Swerve.maxAngularVelocity = 2.5 * math.pi
