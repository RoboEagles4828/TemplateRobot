from phoenix6.controls import DutyCycleOut, PositionVoltage, VelocityVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.signals import InvertedValue

from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from lib.mathlib.Conversions import Conversions
from lib.util.SwerveModuleConstants import SwerveModuleConstants
from Constants import Constants
from CTREConfigs import CTREConfigs
from phoenix6.configs import CANcoderConfiguration
from phoenix6.configs import TalonFXConfiguration 

from wpilib import RobotBase
from sim.SwerveModuleSim import SwerveModuleSim

from wpimath.units import radiansToRotations, rotationsToRadians

class SwerveModule:
    ctreConfigs = CTREConfigs()

    moduleNumber: int
    angleOffset: Rotation2d

    mAngleMotor: TalonFX
    mDriveMotor: TalonFX
    angleEncoder: CANcoder

    driveFeedForward = SimpleMotorFeedforwardMeters(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA)

    driveDutyCycle: DutyCycleOut = DutyCycleOut(0).with_enable_foc(Constants.Swerve.phoenixPro)
    driveVelocity: VelocityVoltage = VelocityVoltage(0).with_enable_foc(Constants.Swerve.phoenixPro)

    anglePosition: PositionVoltage = PositionVoltage(0).with_enable_foc(Constants.Swerve.phoenixPro)

    def __init__(self, moduleNumber: int, moduleConstants: SwerveModuleConstants):
        self.moduleNumber = moduleNumber
        self.angleOffset = moduleConstants.angleOffset

        self.angleEncoder = CANcoder(moduleConstants.cancoderID, Constants.Swerve.canBus)
        self.angleEncoder.configurator.apply(self.ctreConfigs.swerveCANcoderConfig)

        self.mAngleMotor = TalonFX(moduleConstants.angleMotorID, Constants.Swerve.canBus)
        self.mAngleMotor.configurator.apply(self.ctreConfigs.swerveAngleFXConfig)
        self.resetToAbsolute() 

        self.mDriveMotor = TalonFX(moduleConstants.driveMotorID, Constants.Swerve.canBus)
        if self.moduleNumber == 1 or self.moduleNumber == 3:
            self.ctreConfigs.swerveDriveFXConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
            self.mDriveMotor.configurator.apply(self.ctreConfigs.swerveDriveFXConfig)
            self.ctreConfigs.swerveDriveFXConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        else:
            self.mDriveMotor.configurator.apply(self.ctreConfigs.swerveDriveFXConfig)
        # self.mDriveMotor.configurator.apply(self.ctreConfigs.swerveDriveFXConfig)
        self.mDriveMotor.configurator.set_position(0.0)

        if RobotBase.isSimulation():
            self.simModule = SwerveModuleSim()

    def setDesiredState(self, desiredState: SwerveModuleState, isOpenLoop: bool):
        # desiredState = SwerveModuleState.optimize(desiredState, self.getState().angle)
        desiredState.optimize(self.getState().angle)
        self.mAngleMotor.set_control(self.anglePosition.with_position(radiansToRotations(desiredState.angle.radians())))
        self.setSpeed(desiredState, isOpenLoop)

        if RobotBase.isSimulation():
            self.simModule.updateStateAndPosition(desiredState)

    def setDesiredStateNoOptimize(self, desiredState: SwerveModuleState, isOpenLoop: bool):
        desiredState = SwerveModuleState.optimize(desiredState, self.getState().angle)
        self.mAngleMotor.set_control(self.anglePosition.with_position(radiansToRotations(desiredState.angle.radians())))
        self.setSpeed(desiredState, isOpenLoop)

    def setSpeed(self, desiredState: SwerveModuleState, isOpenLoop: bool):
        if isOpenLoop:
            self.driveDutyCycle.output = desiredState.speed / Constants.Swerve.maxSpeed
            self.mDriveMotor.set_control(self.driveDutyCycle)
        else:
            self.driveVelocity.velocity = Conversions.MPSToRPS(desiredState.speed, Constants.Swerve.wheelCircumference)
            self.driveVelocity.feed_forward = self.driveFeedForward.calculate(desiredState.speed)
            self.mDriveMotor.set_control(self.driveVelocity)

    def driveMotorVoltage(self, volts):
        self.mDriveMotor.set_control(VoltageOut(volts, enable_foc=False))

    def getCANcoder(self):
        return Rotation2d(rotationsToRadians(self.angleEncoder.get_absolute_position().value_as_double))

    def resetToAbsolute(self):
        absolutePosition = self.getCANcoder().radians() - self.angleOffset.radians()
        self.mAngleMotor.set_position(radiansToRotations(absolutePosition))

    def getState(self):
        if RobotBase.isSimulation():
            return self.simModule.getState()
        return SwerveModuleState(
            Conversions.RPSToMPS(self.mDriveMotor.get_velocity().value_as_double, Constants.Swerve.wheelCircumference),
            Rotation2d(rotationsToRadians(self.mAngleMotor.get_position().value_as_double))
        )

    def getPosition(self):
        if RobotBase.isSimulation():
            return self.simModule.getPosition()
        return SwerveModulePosition(
            Conversions.rotationsToMeters(self.mDriveMotor.get_position().value_as_double, Constants.Swerve.wheelCircumference),
            Rotation2d(rotationsToRadians(self.mAngleMotor.get_position().value_as_double))
        )
    



