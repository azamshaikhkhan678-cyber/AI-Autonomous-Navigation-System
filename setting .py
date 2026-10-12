# settings.py
"""
Global Configuration parameters for the Autonomous Navigation pipeline.
Defines canvas dimensions, physics properties, and environment constants.
"""

# Visual Display Resolution Constants
WIDTH = 1000
HEIGHT = 700
FPS = 60

# Structural Mapping Matrix Mesh Details
GRID_SIZE = 25
ROWS = HEIGHT // GRID_SIZE
COLS = WIDTH // GRID_SIZE

# Visual Rendering Theme Palettes (RGB Formats)
COLOR_BG = (18, 22, 30)         # Sleek dark matte background theme 
COLOR_WALL = (58, 70, 88)       # High-contrast solid grey-blue barriers
COLOR_CAR = (0, 210, 120)       # Clean green tone for the active agent chassis
COLOR_GOAL = (255, 65, 95)      # Deep red alert color for the target node point
COLOR_PATH = (0, 160, 255)      # Electric blue line tracing the planned path
COLOR_SENSOR = (245, 185, 25)   # Amber lines for the rangefinder ray arrays
COLOR_TEXT = (242, 246, 250)    # Soft white for text readouts

# Kinematic Drive Bounds & Warning Settings
CAR_SPEED = 2.8
SENSOR_MAX_RANGE = 140
COLLISION_THRESHOLD = 22
