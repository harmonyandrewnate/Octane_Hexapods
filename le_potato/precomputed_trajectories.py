import numpy as np
import math

from actuator_kinematics import get_targets_full, get_targets_displacement
from IK import IK



leg0workspace = np.linspace((800, 0, -150), (800, 0, -120), num=10)
leg0workspace = np.append(leg0workspace, np.linspace((800, 0, -120), (800, 100, -120), num=20), axis=0)
leg0workspace = np.append(leg0workspace, np.linspace((800, 100, -120), (800, 100, -150), num=10), axis=0)
leg0workspace = np.append(leg0workspace, np.linspace((800, 100, -150), (800, -100, -150), num=40), axis=0)
leg0workspace = np.append(leg0workspace, np.linspace((800, -100, -150), (800, -100, -120), num=10), axis=0)
leg0workspace = np.append(leg0workspace, np.linspace((800, -100, -120), (800, 0, -120), num=20), axis=0)
leg0workspace = np.append(leg0workspace, np.linspace((800, 0, -120), (800, 0, -150), num=10), axis=0)




leg0jointspace = [IK(leg0workspace[0][0], leg0workspace[0][1], leg0workspace[0][2])]

for elem in leg0workspace[1:]:
    leg0jointspace = np.append(leg0jointspace, [IK(elem[0], elem[1], elem[2])], axis=0)




leg0actuatorspace = [get_targets_displacement(leg0jointspace[0][0], leg0jointspace[0][1], leg0jointspace[0][2], 0)]

for elem in leg0jointspace[1:]:
    leg0actuatorspace = np.append(leg0actuatorspace, [get_targets_displacement(elem[0], elem[1], elem[2], 0)], axis=0)




np.set_printoptions(suppress=True)

print("\nworkspace points:")
print(np.array2string(leg0workspace, separator=', '))
print("\njoint angles(deg):")
print(np.array2string(np.rad2deg(leg0jointspace), separator=', '))
print("\nactuator targets:")
print(np.array2string(leg0actuatorspace, separator=', '))


