from phoenix6.signals import InvertedValue
from phoenix6.signals import NeutralModeValue
from phoenix6.signals import SensorDirectionValue
from wpimath.geometry import Rotation2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
import lib.mathlib.units as Units
from lib.util.COTSTalonFXSwerveConstants import COTSTalonFXSwerveConstants
from lib.util.SwerveModuleConstants import SwerveModuleConstants
import math
from enum import Enum
from pytreemap import TreeMap

from lib.util.InterpolatingTreeMap import InterpolatingTreeMap
from wpimath.units import rotationsToRadians

from pathplannerlib.auto import HolonomicPathFollowerConfig
from pathplannerlib.controller import PIDConstants
from pathplannerlib.config import ReplanningConfig

class Constants:
    stickDeadband = 0.1

    class Swerve:
        navxID = 0

        chosenModule = COTSTalonFXSwerveConstants.MK4i.Falcon500(COTSTalonFXSwerveConstants.MK4i.driveRatios.L2)

        # Drivetrain Constants
        trackWidth = Units.inchesToMeters(20.75)
        wheelBase = Units.inchesToMeters(20.75)
        rotationBase = Units.inchesToMeters(31.125 - 5.25)

        robotWidth = 26.0
        robotLength = 31.125

        armLength = 18.5

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
        angleCurrentThreshold = 40
        angleCurrentThresholdTime = 0.1
        angleEnableCurrentLimit = True

        driveCurrentLimit = 35
        driveCurrentThreshold = 60
        driveCurrentThresholdTime = 0.1
        driveEnableCurrentLimit = True
        
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

        holonomicPathConfig = HolonomicPathFollowerConfig(
            PIDConstants(5.0, 0.0, 0.0),
            PIDConstants(5.0, 0.0, 0.0),
            maxAutoModuleSpeed,
            #distance from center to the furthest module
            Units.inchesToMeters(16),
            ReplanningConfig(),
        )

        # Slowdown speed
        ## The speed is multiplied by this value when the trigger is fully held down
        slowMoveModifier = 0.8
        slowTurnModifier = 0.8

        # Module Specific Constants
        # Front Left Module - Module 0
        class Mod0:
            driveMotorID = 2
            angleMotorID = 1
            canCoderID = 3
            # angleOffset = Rotation2d(rotationsToRadians(-0.354492))
            angleOffset = Rotation2d(rotationsToRadians(-0.349121))
                # angleOffset = Rotation2d(rotationsToRadians(-0.352051))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        # Front Right Module - Module 1
        class Mod1:
            driveMotorID = 19
            angleMotorID = 18
            canCoderID = 20
            angleOffset = Rotation2d(rotationsToRadians(-0.2320910))
            # angleOffset = Rotation2d(rotationsToRadians(-0.233887))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        
        # Back Left Module - Module 2
        class Mod2:
            driveMotorID = 9
            angleMotorID = 8
            canCoderID = 7
            # angleOffset = Rotation2d(rotationsToRadians(0.148193))
            angleOffset = Rotation2d(rotationsToRadians(0.175781))
            # angleOffset = Rotation2d(rotationsToRadians(0.155762))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        # Back Right Module - Module 3
        class Mod3:
            driveMotorID = 12
            angleMotorID = 11
            canCoderID = 10
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