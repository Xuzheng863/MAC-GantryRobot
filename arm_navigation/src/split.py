import yaml

def split(data):
    minX = 10000000
    minY = 10000000
    maxX = -10000000
    maxY = -10000000

    for block in data:
        x, y, _, _, _ = block
        if x < minX: minX = x
        if x > maxX: maxX = x
        if y < minY: minY = y
        if y > maxY: maxY = y

    middleX = (minX + maxX) / 2

    left = []
    right = []

    for block in data:
        x, y, _, _, _ = block
        if x > middleX: right.append(block)
        else: left.append(block)

    with open(r'../config/center_coords_left.yaml', 'w') as file:
        yaml.dump(left, file)

    with open(r'../config/center_coords_right.yaml', 'w') as file:
        yaml.dump(right, file)

if __name__ == "__main__":
    file = "/home/vvvvv/catkin_ws/src/MAC-GantryRobot/arm_navigation/config/center_coords.yaml"
    with open(file) as file:
        data = yaml.load(file)
    split(data)