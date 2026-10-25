# Lidar-based mapless navigation

A robot is placed in an unknown environment. It knows its own position and the goal position, but has no map. Its only sensor is a 2D lidar. Your task is to write the navigation logic that gets the robot to the goal while mapping as much of the area as possible along the way.

The world is a 40x40 grid with rooms, walls, and obstacles. The robot moves one cell per step: `"U"`, `"D"`, `"L"`, `"R"`. Moving into a wall counts as a collision and the robot stays put.

## What to edit

Only edit `navigator.py`. Do not modify `world.py` or `run.py`.

## Running

```bash
pip install -r requirements.txt
python run.py
python watch.py 3
```

## Scoring

- **success rate** - did the robot reach the goal
- **coverage** - fraction of reachable area the lidar scanned
- **SPL** - path efficiency (1.0 = shortest possible path)

The run ends when the robot steps on the goal. Coverage is measured at that moment.
Priority order is success first, coverage second, SPL last.

## Submission

Submit `navigator.py` and a short writeup `WRITEUP.md` (1 page) covering:

- how you decide where to move and how you plan a route through the map
- how you balanced reaching the goal vs. mapping the area
- what fails in your approach and what you would do differently with more time
