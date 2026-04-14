import math
import numpy as np

# Legs are numbered as follows
#   Front
#   0   3
#  1     4
#   2   5


# all calibration measurements in mm
# minimum actuator lengths when fully retracted
minLengthShoulder = (217, 217, 217, 217, 217, 217)
minLengthElbow = (217, 217, 217, 217, 217, 217)

# fixed side lentghs of acutuator triangles
side1Shoulder = (402, 402, 402, 402, 402, 402)
side2Shoulder = (350, 350, 350, 350, 350, 350)

side1Elbow = (277, 277, 277, 277, 277, 277)
side2Elbow = (231, 231, 231, 231, 231, 231)

# angle offsets degrees
offsetShoulderSwing = (0, 0, 0, 0, 0, 0)
offsetShoulderLift = (93, 93, 93, 93, 93, 93)
offsetElbow = (20.4, 20.4, 20.4, 20.4, 20.4, 20.4)

offsetShoulderSwing = np.deg2rad(offsetShoulderSwing)
offsetShoulderLift = np.deg2rad(offsetShoulderLift)
offsetElbow = np.deg2rad(offsetElbow)


def get_len_from_law_of_cosines(a, b, theta):
    return math.sqrt((a ** 2) + (b ** 2) - (2 * a * b * math.cos(theta)))

def get_targets_full(swingAngle, shoulderAngle, elbowAngle, legNumber):
    swing = swingAngle - offsetShoulderSwing[legNumber]
    shoulder = get_len_from_law_of_cosines(side1Shoulder[legNumber], side2Shoulder[legNumber], offsetShoulderLift[legNumber] - shoulderAngle)
    elbow = get_len_from_law_of_cosines(side1Elbow[legNumber], side2Elbow[legNumber], elbowAngle - offsetElbow[legNumber])
    return (swing, shoulder, elbow)

def get_targets_displacement(swingAngle, shoulderAngle, elbowAngle, legNumber):
    (swing, shoulder, elbow) = get_targets_full(swingAngle, shoulderAngle, elbowAngle, legNumber)
    return (swing, shoulder - minLengthShoulder[legNumber], elbow - minLengthElbow[legNumber])
