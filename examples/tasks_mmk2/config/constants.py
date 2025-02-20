import os

TIGHTEN_THRESHOLD = os.environ.get("TIGHTEN_THRESHOLD", default=0.5)
RELEASE_THRESHOLD = os.environ.get("LOOSE_THRESHOLD", default=0.5)

TIGHTEN_WEIGHT = os.environ.get("TIGHTEN_WEIGHT", default=-0.1)
RELEASE_WEIGHT = os.environ.get("LOOSEN_WEIGHT", default=0.1)

UP_WEIGHT = os.environ.get("UP_WEIGHT", default=-0.01)
DOWN_WEIGHT = os.environ.get("DOWN_WEIGHT", default=0.01)

ARM_SCALE = os.environ.get("ARM_SCALE", default=1.2)

LINEAR_VELOCITY = os.environ.get("LINEAR_VELOCITY", default=0.4)
ANGULAR_VELOCITY = os.environ.get("ANGULAR_VELOCITY", default=0.2)

LINEAR_THRESHOLD = os.environ.get("LINEAR_THRESHOLD", default=0.5)
ANGULAR_THRESHOLD = os.environ.get("ANGULAR_THRESHOLD", default=0.5)