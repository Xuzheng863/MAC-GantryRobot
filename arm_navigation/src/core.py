import helper
import yaml
import sys
from math import sqrt

'''
Usage: define voxelFile as the goxel generated file you want to
parse center point.
Don't forget to delete all the comments at the start of the voxel
file before passing it in!
'''
def FindBlockCenters(voxelFile, unitToMM, canHeight):
    groupedVoxels = helper.ParseSSV(voxelFile)

    blocks = []
    for color in groupedVoxels:
        voxels = groupedVoxels[color]
        blocks = blocks + helper.FindBlocks(voxels)

    centers = []
    for block in blocks:
        centers = centers + [helper.FindCenter(block)]

   
    centers = helper.CenterToWorldCoordinate(centers, unitToMM, canHeight)
    centers, bottom, right = helper.NormalizeCenters(centers, unitToMM)
    blocks = helper.BlockToWorldCoordinate(blocks, unitToMM, bottom, right)

    helper.Visualize(blocks, centers, unitToMM)
    
    return centers

'''
Assuming the robot is facing the positive y direction.
Given the robot arm parameters:
robot_position, link1_length, link2_length, vertical_limit,
filter out all points in the input array of center points 
that's not currently reachable.

TODO: Make this not rely on robot's orientation assumption
TODO: Take in an array of link length
'''
def FilterPoints(robot_pos, l1, l2, v, center):
    unreachable = []
    reachable = []
    for point in center:
        x, y, z, _, _ = point
        rx, ry, rz = robot_pos
        
        if z > (rz + v):
            unreachable.append(point)
            continue
        
        xdist = x - rx
        ydist = y - ry
        dist = sqrt(xdist**2 + ydist**2)
        if xdist <= 0:
            unreachable.append(point)
            continue
        if dist > (l1 + l2):
            unreachable.append(point)
            continue
        
        reachable.append(point)

    return reachable, unreachable


def BuildSequence(centers):
    centers.sort(key=helper.BlockBuildWeight, reverse=True)
    return centers

def CurrentBuildList(voxelFile, unitToMM, canHeight, robot_pos, l1, l2, v):
    center = FindBlockCenters(voxelFile, unitToMM, canHeight)
    reachable, unreachable = FilterPoints(robot_pos, l1, l2, v, center)
    center = BuildSequence(reachable)
    with open(r'../config/center_coords.yaml', 'w') as file:
        documents = yaml.dump(center, file)

    return yaml.dump(center)

if __name__ == '__main__':
    if (not len(sys.argv) == 2):
        sys.exit('Usage: python3 core.py <file name to be parsed>.txt')
    voxelFile = (sys.argv)[-1]
    print(CurrentBuildList(voxelFile, 66.5, 122, [-0, -0, 0], 100000, 100000, 600))
    
