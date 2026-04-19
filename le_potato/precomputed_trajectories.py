import numpy as np
import math

from actuator_kinematics import get_targets_full, get_targets_displacement
from IK import IK


def get_workspace_traj(height, lift, centerx, deltax, centery, deltay):
    legWorkspace = np.linspace((centerx, centery, -height), (centerx, centery, -height + lift), num=10)
    legWorkspace = np.append(legWorkspace, np.linspace((centerx, centery, -height + lift), (centerx + deltax, centery + deltay, -height + lift), num=20), axis=0)
    legWorkspace = np.append(legWorkspace, np.linspace((centerx + deltax, centery + deltay, -height + lift), (centerx, centery + deltay, -height - lift), num=10), axis=0)
    legWorkspace = np.append(legWorkspace, np.linspace((centerx + deltax, centery + deltay, -height - lift), (centerx - deltax, centery - deltay, -height - lift), num=40), axis=0)
    legWorkspace = np.append(legWorkspace, np.linspace((centerx - deltax, centery - deltay, -height - lift), (centerx - deltax, centery - deltay, -height + lift), num=10), axis=0)
    legWorkspace = np.append(legWorkspace, np.linspace((centerx - deltax, centery - deltay, -height + lift), (centerx, centery, -height + lift), num=20), axis=0)
    legWorkspace = np.append(legWorkspace, np.linspace((centerx, centery, -height + lift), (centerx, centery, -height), num=10), axis=0)
    
    return legWorkspace

def workspace_to_jointspace(workspace):
    legJointspace = [IK(workspace[0][0], workspace[0][1], workspace[0][2])]

    for elem in workspace[1:]:
        legJointspace = np.append(legJointspace, [IK(elem[0], elem[1], elem[2])], axis=0)

    return legJointspace

def jointspace_to_actuatorspace(jointspace):
    actuatorspace = [get_targets_displacement(jointspace[0][0], jointspace[0][1], jointspace[0][2], 0)]

    for elem in jointspace[1:]:
        actuatorspace = np.append(actuatorspace, [get_targets_displacement(elem[0], elem[1], elem[2], 0)], axis=0)
   
    return actuatorspace 


leg_workspace = get_workspace_traj(200, 30, 800, 0, 0, 100)

leg_jointspace = workspace_to_jointspace(leg_workspace)

leg_actuatorspace = jointspace_to_actuatorspace(leg_jointspace)



np.set_printoptions(suppress=True)

#print("\nworkspace points:")
#print(np.array2string(leg_workspace, separator=', '))
#print("\njoint angles(deg):")
#print(np.array2string(np.rad2deg(leg_jointspace), separator=', '))
print("\nactuator targets:")
print(np.array2string(leg_actuatorspace, separator=', '))


