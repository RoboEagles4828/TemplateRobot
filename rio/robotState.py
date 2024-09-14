# ===TEMPLATE CLASS===
# This file should be modified to meet your specific robot needs.
# Define new functions to meet your requirements for the game.

class RobotState:
    """
    Store robot specific information which evaluates if the robot 
    is ready to execute a specific task.
    """

    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super(RobotState, cls).__new__(cls)
        return cls.instance

    def initialize(self):
        """
        Only called from RobotContainer after the constructor to complete
        initialization of the singleton.
        """
        pass