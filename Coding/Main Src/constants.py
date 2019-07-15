"""
Constants File
Save numerical constants here. This way if we need to change
them we don't need to go through all of the code for toddler
"""

# MODES
TEST_MODE = False # If test mode is true the motors will not run

# Maneuvers constants
CYCLES_FOR_180_TURN = 20
CYCLES_FOR_90_TURN = 15
STANDARD_IDLE_CYCLES = 4
BACKWARD_CYCLES_COLLISION = 10 
COLLISION_RESET_CYCLES = 20
AVOID_COLLISION_CYCLES = 20
POINT_ANTENNA_CYCLES = 20
POI_DETECTED_CYCLES = 20
POI_RESET_FORWARD = 20
TURNING_SONAR_CYCLES = 10

# Sensors constants
IR_COLLISION_THRESHOLD = 350
IR_TURNING_THRESHOLD = 260
SONAR_THRESHOLD = 20

