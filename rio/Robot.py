from wpilib import TimedRobot
from commands2 import Command
from commands2 import CommandScheduler
from CTREConfigs import CTREConfigs
from Constants import Constants
from rio.RobotContainer import RobotContainer
from wpimath.geometry import Rotation2d

from wpilib.shuffleboard import ShuffleboardTab
from wpilib import DriverStation

class Robot(TimedRobot):
  m_autonomousCommand: Command = None

  m_robotContainer: RobotContainer

  auton_tab: ShuffleboardTab
  teleop_tab: ShuffleboardTab

  def robotInit(self):
    """ Instantiate our `RobotContainer`.  
    This will perform all button bindings and put the auton chooser on the dashboard
    """
    # wpilib.CameraServer.launch()
    self.m_robotContainer = RobotContainer()
    CommandScheduler.getInstance().setPeriod(0.02)

  def robotPeriodic(self):
    """ Runs the `CommandScheduler`.  
    This is responsible for polling buttons, adding newly-scheduled
    commands, running already-scheduled commands, removing finished or interrupted commands,
    and running subsystem `periodic()` methods.  This must be called from the robot's periodic
    block in order for anything in the Command-based framework to work.
    """
    CommandScheduler.getInstance().run()

  def autonomousInit(self):
    """Initialize autonomous code.  
    Schedules the auton to run if chosen on the auton selector.
    """
    m_autonomousCommand: Command = self.m_robotContainer.getAutonomousCommand()

    if m_autonomousCommand != None:
      m_autonomousCommand.schedule()

  def teleopInit(self):
    # Flip heading
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
      self.m_robotContainer.s_Swerve.setHeading(self.m_robotContainer.s_Swerve.getHeading().rotateBy(Rotation2d.fromDegrees(180.0)))

    # Cancels auton when teleop starts.
    # If you don't want to do this, add more logic here
    if self.m_autonomousCommand is not None:
      self.m_autonomousCommand.cancel()

  def testInit(self):
    CommandScheduler.getInstance().cancelAll()