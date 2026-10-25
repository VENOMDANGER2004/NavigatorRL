# Mapless Lidar Navigation: Zero-Margin BFS/A* Orchestrator

## 1. how you decide where to move and how you plan a route through the map?
To address the mapless navigation challenge, I drew inspiration from Brian Yamauchi’s foundational research, *A Frontier-Based Approach for Autonomous Exploration* (1997). My implementation utilizes an Orchestrator that combines the spatial thoroughness of Breadth-First Search (BFS) with the path-finding precision of A*.

The agent operates in two distinct, dynamic modes based on environmental context:
* **The Explorer (BFS):** The agent uses BFS to identify "Frontiers"—the specific boundaries where known space meets the unknown. This ensures high spatial coverage by methodically guiding the agent into complex enclosures and corners that heuristic path-finders often overlook.
* **The Navigator (A star):** To transit toward these frontiers (or the final goal), the system employs an A* engine with dynamic terrain weights. By penalizing `VISITED` cells (Cost = 3.0) and discounting `UNKNOWN` cells (Cost = 0.5), the algorithm maintains smooth, directional momentum while preventing the agent from getting trapped in loops or dead-ends.

## 2. how you balanced reaching the goal vs. mapping the area?
I managed the adversarial relationship between Coverage(high) and Shortest Path Length (SPL)(low) using a **Zero-Margin Dynamic Orchestrator**. 

Instead of relying on predetermined step limits or arbitrary triggers , the robot performs a live calculation at every single update. It uses a BFS to compute the true, current mathematical step-distance required to reach the goal (`steps_required`) and compares this against the remaining 1000-step budget. As long as `remaining_budget > steps_required`, the robot remains in its aggressive exploration mode, vacuuming up map data. The exact moment those two numbers equalize, the robot abandons mapping entirely and executes a precision sprint to the goal. This guarantees a 100% Success Rate while extracting the absolute mathematical maximum of coverage possible from the available budget.

## 3. what fails in your approach and what you would do differently with more time?
**What Fails:** The primary vulnerability of this approach is inefficiency in massive, completely open spaces. BFS frontier-hunting is highly optimized for tight enclosures and grid bottlenecks, but in a wide-open map, it evaluates too many irrelevant continuous nodes. This leads to slightly less efficient paths compared to pure straight-line ray-casting algorithms.

**Improving SPL (Targeting >0.50 SPL with >70% Coverage):** 
To reach a 0.50+ SPL while maintaining 70%+ coverage, the agent needs to transition from "exhaustive" to "optimal" exploration based on volume estimation. The current implementation remains in an exhaustive BFS frontier-hunting state until the budget forces a sprint. 

With more time, I would implement **Dynamic Volume Estimation**. By processing the LiDAR scan, the agent could estimate the total reachable volume of the room. If the estimated volume exceeds a certain threshold (suggesting a wide-open space rather than a complex maze), the agent would switch to **Jump Point Search (JPS)**. JPS allows the robot to "jump" over open space in straight, efficient lines rather than evaluating every grid cell. This maintains 70%+ coverage by sampling the perimeter, but dramatically reduces the step count—and improves the SPL—by ignoring the exhaustive grid-by-grid movement that dominates the current approach. This creates a transition governed by environmental geometry rather than static, manual settings.