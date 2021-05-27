# Rviz Visualization Part Documentation

Be sure to build and source your workspace: ```source devel/setup.bash```

### Usage

1. ```python3 core.py <file name>.txt ```

   Run parsing script ```core.py``` under ```mac/arm_navigation/src``` and store the yaml output locally.

   The Voxel output .txt file you want to parse should be stored under                                                                                           ```mac/arm_navigation/config/desired_structures```, currently there are multiples to try out, including a CMU and TEN design. You can also create your own using the Goxel software : )

2. ```roslaunch arm_kinematics kinematics.launch```

   This will publish the world, grid, and rail frame. 

3. ```roslaunch arm_navigation grid_viz.launch```

   This will publish the block, center points, and gantry robot visualization, as well as the animation.

4. ```rviz```

   Finally run the visualization. Find ```TF``` to add the frames, and find the following messages under the topic tab to add the visualization and animation: 1. /block_viz 2. can_viz 3. gantry_viz 4. robot_viz. After doing this, you might need to redo step 3 to see the animation (terminate the process and rerun it)

### Configure the robot

To change the property of the gantry robot (length, height, width, origin, gantry length, etc.), simply goto ```mac/arm_navigation/src/grid_viz.py```, and find the init function of the Robot class.