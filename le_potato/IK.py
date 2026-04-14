import numpy as np
import math

LINK_1 = 125.58
LINK_2 = 425
LINK_3 = 618
TWIST_1 = math.pi/2
TWIST_2 = 0
TWIST_3 = 0

# Where theta_1 is swing, theta_2 is shoulder, and theta_3 is elbow
# input in mm
def IK(X, Y, Z):
    theta_1 = math.atan2(Y,X)
    X = X - (LINK_1 * math.cos(theta_1))
    Y = Y - (LINK_1 * math.sin(theta_1))
    hypotenuseSqrd = (X**2) + (Y**2) + (Z**2)
    if (math.sqrt(hypotenuseSqrd) > LINK_2+LINK_3):
        return
    if (math.sqrt(hypotenuseSqrd) < abs(LINK_2-LINK_3)):
        return
    theta_2 = math.acos(((LINK_2**2)+(hypotenuseSqrd)-(LINK_3**2)) / (2*LINK_2*math.sqrt(hypotenuseSqrd))) + math.asin(Z/math.sqrt(hypotenuseSqrd))
    theta_3 = math.acos(((LINK_2**2)+(LINK_3**2)-hypotenuseSqrd) / (2*LINK_2*LINK_3))
    return [theta_1, theta_2, theta_3]


if __name__ =="__main__":
    while (True):
        print("Input target X position:")
        X = float(input())
        print("Input target Y position:")
        Y = float(input())
        print("Input target Z position:")
        Z = float(input())

        angles = IK(X, Y, Z)
        if angles is None:
            print("Out of Range")
        else:
            angles = np.rad2deg(angles)
        
            angles[0] *= -1
            angles[1] += 0
            angles[2] += 180

            print("Angles to reach target:")
            print(angles)
