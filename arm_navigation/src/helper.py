#import matplotlib 
#import matplotlib.pyplot as plt
from math import sqrt
import os

'''
Given a voxel file shaped like:
    x1 y1 z1 color1
    x2 y2 z2 color2
    ...
parse the file and group each voxel entry according 
to its color, and return the result as a dictionary.
'''
def ParseSSV(filename):
    cur_path = os.path.dirname(__file__)
    filename = os.path.join(cur_path, "..", "config/", "desired_structures/", filename)

    with open(filename) as f:
        content = f.readlines()
    content = [x.strip() for x in content] 
    
    groupedVoxels = {}
    for entry in content:
        if (entry[0] == "#"):
            continue
        x, y, z, color = entry.split()
        if color not in groupedVoxels:
            groupedVoxels[color] = []
        groupedVoxels[color].append([int(x), int(y), int(z), color])

    return groupedVoxels

'''
return if V1 and V2 are planer 4-adjacent voxels
'''
def PlanarAdjacent(V1, V2):
    x1, y1, z1, _ = V1
    x2, y2, z2, _ = V2
    
    if z1 != z1:
        return False
    if (abs(x1-x2) + abs(y1-y2) == 1):
        return True
    return False

'''
Recursively find all voxels in input voxels that is directly or
indirectly connected to v, and store all of them in block.
Assume the passed in voxels do not contain any outlier or error.
'''
def FindAllAdjacentVoxel(v, voxels, block):
    adj = []
    
    idx = 0
    for i in range(len(voxels)):
        if PlanarAdjacent(v, voxels[idx]):
            adj.append(voxels.pop(idx))
        else: idx = idx + 1
    block.append(adj)

    if not adj or not voxels:
        return voxels, block
    
    for adjv in adj:
        voxels, block = FindAllAdjacentVoxel(adjv, voxels, block)
    return voxels, block

'''
Find all blocks formed by the voxels in inputted voxels,
assuming each voxel in voxels belongs to a valid block.
'''
def FindBlocks(voxels):
    blocks = []
    while(voxels):
        v = voxels.pop()
        voxels, block = FindAllAdjacentVoxel(v, voxels, [])
        block = [v] + [item for sublist in block for item in sublist]
        blocks.append(block)
    return blocks

'''
Find the center of the block according to the accordinates in the 
six voxels in the input.
'''
def FindCenter(block):
    minX = 10000000
    minY = 10000000
    maxX = -10000000
    maxY = -10000000
    orientation = 0

    for voxel in block:
        x, y, z, color = voxel
        if x < minX: minX = x
        if x > maxX: maxX = x
        if y < minY: minY = y
        if y > maxY: maxY = y
    
    if (maxX - minX) > (maxY - minY):
        orientation = 0
    else:
        orientation = 90

    return [(minX + maxX)/2, (minY + maxY)/2, z, color, orientation]


def Visualize(blocks, centers, unitToMM):
    #fig = plt.figure() 
    #ax = fig.add_subplot(111)

    for block in blocks:
        minX = 10000000
        minY = 10000000
        maxX = -10000000
        maxY = -10000000
        for voxel in block:
            x, y, _, color = voxel
            if x < minX: minX = x
            if x > maxX: maxX = x
            if y < minY: minY = y
            if y > maxY: maxY = y
        width = maxX - minX + unitToMM
        height = maxY - minY + unitToMM
        #rect = matplotlib.patches.Rectangle((minX-unitToMM/2, minY-unitToMM/2), width, height, color = '#'+color)
        #ax.add_patch(rect)

    for center in centers:
        x, y, _, _, _ = center
        #plt.plot(x, y, 'r+')
    #plt.axis('equal')
    #plt.show()

def CenterToWorldCoordinate(centers, unitToMM, canHeight):
    for i in range(len(centers)):
        centers[i][0] = centers[i][0] * unitToMM
        centers[i][1] = centers[i][1] * unitToMM
        centers[i][2] = centers[i][2] * canHeight
    return centers

def BlockToWorldCoordinate(blocks, unitToMM, bottom, right):
    for i in range(len(blocks)):
        for j in range(len(blocks[i])):
            blocks[i][j][2] = blocks[i][j][2] * unitToMM
            blocks[i][j][0] = blocks[i][j][0] * unitToMM - bottom
            blocks[i][j][1] = blocks[i][j][1] * unitToMM - right

    return blocks

'''
Given the center point of the block, return the weight the block 
is assigned when building the structure, with bigger weight means
later build.
'''
def BlockBuildWeight(point):
    zWeight = 1000000000
    x, y, z, _, _ = point
    return sqrt(x**2 + y**2) - z * zWeight


def NormalizeCenters(centers, unitToMM):
	bottom = centers[0][0]
	right = centers[0][1]
	bottomOri = centers[0][-1]
	rightOri = centers[0][-1]
	
	for point in centers:
		if point[0] < bottom: 
			bottom = point[0]
			bottomOri = point[-1]
		if point[1] < right: 
			right = point[1]
			rightOri = point[-1]
	
	rightOffset = unitToMM
	bottomOffset = unitToMM * 1.5

	if (rightOri != 0):
		rightOffset = unitToMM * 1.5
	if (bottomOri != 0):
		bottomOffset = unitToMM

	for point in centers:
		point[0] = point[0] - bottom + bottomOffset
		point[1] = point[1] - right + rightOffset
	
	return centers, bottom - bottomOffset, right - rightOffset


if __name__ == '__main__':
    voxels = [[-13, -12, 0], [-12, -12, 0], [-13, -11, 0], [-12, -11, 0], [-13, -10, 0], [-12, -10, 0], [-8, -10, 0], [-7, -10, 0], [-8, -9, 0], [-7, -9, 0], [-12, -8, 0], [-11, -8, 0], [-10, -8, 0], [-8, -8, 0], [-7, -8, 0], [-12, -7, 0], [-11, -7, 0], [-10, -7, 0]]
    blocks = FindBlocks(voxels)
    for i in range(len(blocks)):
        print(i, blocks[i])