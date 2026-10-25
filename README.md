# Lidar-based mapless navigation

A robot is placed in an unknown environment. It knows its own position and the goal position, but has no map. Its only sensor is a 2D lidar. Your task is to write the navigation logic that gets the robot to the goal while mapping as much of the area as possible along the way.

The world is a 40x40 grid with rooms, walls, and obstacles. The robot moves one cell per step: `"U"`, `"D"`, `"L"`, `"R"`. Moving into a wall counts as a collision and the robot stays put.
