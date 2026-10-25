import numpy as np
import heapq
from collections import deque
from world import MOVES, FREE, WALL, scan_to_cells


class Navigator:
    def __init__(self, shape, start, goal):
        self.shape = shape
        self.goal = goal
        self.known = {}
        self.visited = set()
        self.steps = 0

        self.known[start] = FREE
        self.known[goal] = FREE

    def _get_true_distance_to_goal(self, start, goal):
        """Uses BFS to calculate the exact mathematical steps required to reach the goal."""
        queue = deque([(start, 0)])
        seen = {start}

        while queue:
            curr, dist = queue.popleft()
            if curr == goal:
                return dist

            for dr, dc in MOVES.values():
                nr, nc = curr[0] + dr, curr[1] + dc
                if 0 <= nr < self.shape[0] and 0 <= nc < self.shape[1]:
                    if (nr, nc) not in seen and self.known.get((nr, nc), FREE) != WALL:
                        seen.add((nr, nc))
                        queue.append(((nr, nc), dist + 1))
        return float('inf')

    def _get_best_utility_frontier(self, r, c, gr, gc):
        """
        Yamauchi Utility Evaluation:
        Finds all frontiers and scores them mathematically.
        High Utility = Far from the goal (hard corners) but relatively close to the robot.
        """
        best_frontier = None
        highest_utility = -float('inf')

        # O(N) scan of known map to find frontiers
        for (nr, nc), state in self.known.items():
            if state == FREE:
                is_frontier = False
                for dr, dc in MOVES.values():
                    if (nr + dr, nc + dc) not in self.known:
                        is_frontier = True
                        break

                if is_frontier:
                    # Utility Math: Weight distance from goal vs cost to reach
                    dist_from_goal = abs(nr - gr) + abs(nc - gc)
                    dist_from_robot = abs(nr - r) + abs(nc - c)

                    # Score prioritizes mapping deep corners before returning to extraction
                    utility = (dist_from_goal * 1.5) - dist_from_robot

                    if utility > highest_utility:
                        highest_utility = utility
                        best_frontier = (nr, nc)

        return best_frontier

    def _a_star_route(self, start, target, apply_inertia=False):
        """Inertia-Weighted A* routing engine for Next-Best-View (NBV) style sweeps."""
        counter = 0
        pq = [(0, counter, start, None)]
        came_from = {start: None}
        cost_so_far = {start: 0}
        move_to = {start: None}

        while pq:
            current_cost, _, curr, heading = heapq.heappop(pq)
            if curr == target:
                break

            for move_name, (dr, dc) in MOVES.items():
                nr, nc = curr[0] + dr, curr[1] + dc
                if 0 <= nr < self.shape[0] and 0 <= nc < self.shape[1]:
                    state = self.known.get((nr, nc), -1)
                    if state == WALL:
                        continue

                    if state == -1:
                        cost = 0.5
                    elif (nr, nc) in self.visited:
                        cost = 3.0
                    else:
                        cost = 1.0

                        # INERTIA (NBV Momentum): Heavily penalize turning to force long, sweeping paths
                    if apply_inertia and heading is not None and move_name != heading:
                        cost += 2.0

                    new_cost = cost_so_far[curr] + cost
                    if (nr, nc) not in cost_so_far or new_cost < cost_so_far[(nr, nc)]:
                        cost_so_far[(nr, nc)] = new_cost
                        priority = new_cost + (abs(nr - target[0]) + abs(nc - target[1]))

                        counter += 1
                        heapq.heappush(pq, (priority, counter, (nr, nc), move_name))
                        came_from[(nr, nc)] = curr
                        move_to[(nr, nc)] = move_name

        if target in came_from and came_from[target] is not None:
            curr = target
            while came_from[curr] != start:
                curr = came_from[curr]
            return move_to[curr]
        return None

    def act(self, obs):
        self.steps += 1

        # 1. Update Map
        free, occ = scan_to_cells(obs["pose"], obs["scan"])
        for c in occ: self.known[c] = WALL
        for c in free:
            if self.known.get(c) != WALL: self.known[c] = FREE

        r, c = obs["pose"]
        self.visited.add((r, c))
        gr, gc = obs["goal"]

        # 2. ZERO-MARGIN DYNAMIC ORCHESTRATOR
        steps_required = self._get_true_distance_to_goal((r, c), (gr, gc))
        remaining_budget = 1000 - self.steps

        # Phase 1: Information-Theoretic Exploration (Zero Buffer)
        if remaining_budget > steps_required:
            frontier = self._get_best_utility_frontier(r, c, gr, gc)
            if frontier:
                # Apply inertia to mimic NBV straight-line sweeps while mapping
                move = self._a_star_route((r, c), frontier, apply_inertia=True)
                if move: return move

        # Phase 2: The Precision Sprint
        # Drop the inertia penalty for the final run to guarantee absolute SPL efficiency
        move = self._a_star_route((r, c), (gr, gc), apply_inertia=False)
        if move: return move

        # 3. Fail-safe
        options = [name for name, (dr, dc) in MOVES.items()
                   if 0 <= r + dr < self.shape[0] and 0 <= c + dc < self.shape[1]
                   and self.known.get((r + dr, c + dc)) != WALL]
        return np.random.choice(options) if options else "U"