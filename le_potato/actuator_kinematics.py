import math

# Legs are numbered as follows
#   Front
#   0   3
#  1     4
#   2   5


# all calibration measurements in mm
# minimum actuator lengths when fully retracted
minLengthShoulder = {217, 217, 217, 217, 217, 217}
minLengthElbow = {217, 217, 217, 217, 217, 217}

# fixed side lentghs of acutuator triangles
side1Shoulder = {402, 402, 402, 402, 402, 402}
side2Shoulder = {350, 350, 350, 350, 350, 350}

side1Elbow = {267, 267, 267, 267, 267, 267}
side2Elbow = {434, 434, 434, 434, 434, 434}

# angle offsets
offsetShoulderSwing = {0, 0, 0, 0, 0, 0}
offsetShoulderLift = {0, 0, 0, 0, 0, 0}
offsetElbow = {0, 0, 0, 0, 0, 0}


def get_targets(theta1, theta2, theta3, leg):
    
