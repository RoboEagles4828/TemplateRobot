# ===TEMPLATE CLASS===
# This file should be modified to meet your specific robot needs.
# Define new functions to meet your requirements for the game.

class GameState:
    """
    The single GameState object holds information on the current state of our gameplay. 
    """

    def __new__(cls):
        if not hasattr(cls, "instance"):
            cls.instance = super(GameState, cls).__new__(cls)
        return cls.instance