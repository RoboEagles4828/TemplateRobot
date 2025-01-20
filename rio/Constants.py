from phoenix6.signals import InvertedValue
from phoenix6.signals import NeutralModeValue
from phoenix6.signals import SensorDirectionValue
from wpimath.geometry import Rotation2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
import lib.mathlib.Units as Units
from lib.util.COTSTalonFXSwerveConstants import COTSTalonFXSwerveConstants
from lib.util.SwerveModuleConstants import SwerveModuleConstants
import math

from wpimath.units import rotationsToRadians
from wpimath.system.plant import DCMotor

from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.controller import PIDConstants
from pathplannerlib.config import RobotConfig, ModuleConfig
# from pathplannerlib.config import ReplanningConfig

class Constants:
    stickDeadband = 0.1

    class Swerve:
        pigeonID = 0

        chosenModule = COTSTalonFXSwerveConstants.MK4i.Falcon500(COTSTalonFXSwerveConstants.MK4i.driveRatios.L2)

        # Drivetrain Constants remember to check this everytime you change chassis
        trackWidth = Units.inchesToMeters(22.68)
        wheelBase = Units.inchesToMeters(22.68)
        rotationBase = Units.inchesToMeters(31.125 - 5.25)

        robotWidth = 26.0
        robotLength = 26.0 #if the robot break, understand that raza supplied these numbers and is probably the cause of fault.

        canBus = "Default Name" # Either "canivore", "Default Name", "rio" depending on what you're addressing CAN Ids with
        phoenixPro = True

        frontOffset = rotationBase - wheelBase

        wheelCircumference = chosenModule.wheelCircumference

        frontLeftLocation = Translation2d(-((wheelBase / 2.0) - frontOffset), -trackWidth / 2.0)
        frontRightLocation = Translation2d(-((wheelBase / 2.0) - frontOffset), trackWidth / 2.0)
        backLeftLocation = Translation2d(wheelBase / 2.0, -trackWidth / 2.0)
        backRightLocation = Translation2d(wheelBase / 2.0, trackWidth / 2.0)

        robotCenterLocation = Translation2d(0.0, 0.0)

        swerveKinematics = SwerveDrive4Kinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
        )

        # Module Gear Ratios
        driveGearRatio = chosenModule.driveGearRatio
        angleGearRatio = chosenModule.angleGearRatio

        # Motor Inverts
        angleMotorInvert = chosenModule.angleMotorInvert
        driveMotorInvert = chosenModule.driveMotorInvert

        # Angle Encoder Invert
        cancoderInvert = chosenModule.cancoderInvert

        # Swerve Current Limiting
        angleCurrentLimit = 25
        angleCurrentLowerLimit = 40  #previously angleCurrentThreshold
        angleCurrentLowerTime = 0.1 #angleCurrentThresholdTime
        angleEnableCurrentLimit = True

        driveCurrentLimit = 35
        driveCurrentLowerLimit = 60   #previously driveCurrentThreshold
        driveCurrentLowerTime = 0.1
        driveEnableCurrentLimit = True #previously driveCurrentThresholdTime
        
        driveStatorCurrentLimit = 130
        driveEnableStatorCurrentLimit = True

        openLoopRamp = 0.0
        closedLoopRamp = 0.0

        # Angle Motor PID Values
        angleKP = chosenModule.angleKP
        angleKI = chosenModule.angleKI
        angleKD = chosenModule.angleKD

        # Drive Motor PID Values
        driveKP = 2.5
        driveKI = 0.0
        driveKD = 0.0
        driveKF = 0.0

        driveKS = 0.2
        driveKV = 0.28
        driveKA = 0.0

        # Swerve Profiling Values
        # Meters per Second
        maxSpeed = 5.0
        maxAutoModuleSpeed = 4.5
        # Radians per Second
        maxAngularVelocity = 2.5 * math.pi

        # Neutral Modes
        angleNeutralMode = NeutralModeValue.COAST
        driveNeutralMode = NeutralModeValue.BRAKE

        holonomicPathConfig = PPHolonomicDriveController(
            PIDConstants(5.0, 0.0, 0.0),
            PIDConstants(5.0, 0.0, 0.0),
            # maxAutoModuleSpeed,
            # #distance from center to the furthest module
            # Units.inchesToMeters(16),
            0.2
        )

        # moduleConfig = ModuleConfig(
        #     0.089,
        #     16.5,
        #     1.2,
        #     DCMotor.krakenX60FOC(),
        #     60.0,
        #     1
        # )
        

    #     robotConfig = RobotConfig(
    #         52.1631,
    #         6.490,
    #         moduleConfig,
    #         None,
    #         0.576,
    #   )

        robotConfig = RobotConfig.fromGUISettings()


        

        # Slowdown speed
        ## The speed is multiplied by this value when the trigger is fully held down
        slowMoveModifier = 0.8
        slowTurnModifier = 0.8

        # Module Specific Constants
        # Front Left Module - Module 0
        class Mod0:
            driveMotorID = 1
            angleMotorID = 3    
            canCoderID = 2
            # angleOffset = Rotation2d(rotationsToRadians(-0.354492))
            angleOffset = Rotation2d(rotationsToRadians(-0.349121))
                # angleOffset = Rotation2d(rotationsToRadians(-0.352051))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        # Front Right Module - Module 1
        class Mod1:
            driveMotorID = 10
            angleMotorID = 12
            canCoderID = 11
            angleOffset = Rotation2d(rotationsToRadians(-0.2320910))
            # angleOffset = Rotation2d(rotationsToRadians(-0.233887))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        
        # Back Left Module - Module 2
        class Mod2:
            driveMotorID = 4
            angleMotorID = 6
            canCoderID = 5
            # angleOffset = Rotation2d(rotationsToRadians(0.148193))
            angleOffset = Rotation2d(rotationsToRadians(0.175781))
            # angleOffset = Rotation2d(rotationsToRadians(0.155762))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        # Back Right Module - Module 3
        class Mod3:
            driveMotorID = 7
            angleMotorID = 9
            canCoderID = 8
            angleOffset = Rotation2d(rotationsToRadians(0.068359))
            # angleOffset = Rotation2d(rotationsToRadians(0.063232))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

    class AutoConstants:
        kMaxSpeedMetersPerSecond = 3
        kMaxModuleSpeed = 4.5
        kMaxAccelerationMetersPerSecondSquared = 3
        kMaxAngularSpeedRadiansPerSecond = math.pi
        kMaxAngularSpeedRadiansPerSecondSquared = math.pi

        kPXController = 4.0
        kPYController = 4.0
        kPThetaController = 1.5
    
        kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
            kMaxAngularSpeedRadiansPerSecond, 
            kMaxAngularSpeedRadiansPerSecondSquared
        )


    class StandardDeviations:
        singleTagXY = 0.03
        multiTagXY = 0.05
        tagRot = math.radians(40.0)