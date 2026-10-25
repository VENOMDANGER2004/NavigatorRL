import numpy as np
from collections import deque

MOVES = {"U": (-1, 0), "D": (1, 0), "L": (0, -1), "R": (0, 1)}
FREE, WALL = 0, 1


def _bfs_free_region(grid, start):
    H, W = grid.shape
    seen = {start}
    q = deque([start])
    while q:
        r, c = q.popleft()
        for dr, dc in MOVES.values():
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and grid[nr, nc] == FREE \
                    and (nr, nc) not in seen:
                seen.add((nr, nc))
                q.append((nr, nc))
    return seen


def shortest_path_len(grid, start, goal):
    H, W = grid.shape
    seen = {start}
    q = deque([(start, 0)])
    while q:
        (r, c), d = q.popleft()
        if (r, c) == goal:
            return d
        for dr, dc in MOVES.values():
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and grid[nr, nc] == FREE \
                    and (nr, nc) not in seen:
                seen.add((nr, nc))
                q.append(((nr, nc), d + 1))
    return None


def _add_room_walls(grid, rng):
    H, W = grid.shape
    for _ in range(rng.integers(2, 4)):
        if rng.random() < 0.5:
            c = rng.integers(W // 4, 3 * W // 4)
            grid[1:H - 1, c] = WALL
            door = rng.integers(2, H - 2)
            grid[door:door + 2, c] = FREE
        else:
            r = rng.integers(H // 4, 3 * H // 4)
            grid[r, 1:W - 1] = WALL
            door = rng.integers(2, W - 2)
            grid[r, door:door + 2] = FREE


def _add_obstacles(grid, rng, n):
    H, W = grid.shape
    for _ in range(n):
        r = rng.integers(2, H - 3)
        c = rng.integers(2, W - 3)
        grid[r:r + rng.integers(1, 4), c:c + rng.integers(1, 4)] = WALL


def _add_trap(grid, rng):
    H, W = grid.shape
    r = rng.integers(H // 4, 3 * H // 4)
    c = rng.integers(W // 4, 3 * W // 4)
    size = rng.integers(4, 7)
    r2, c2 = min(r + size, H - 2), min(c + size, W - 2)
    grid[r:r2, c] = WALL
    grid[r:r2, c2] = WALL
    grid[r2, c:c2 + 1] = WALL


def generate_world(seed, size=40, obstacle_count=14, traps=2):
    rng = np.random.default_rng(seed)
    for _ in range(60):
        grid = np.zeros((size, size), dtype=np.int8)
        grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = WALL
        _add_room_walls(grid, rng)
        _add_obstacles(grid, rng, obstacle_count)
        for _ in range(traps):
            _add_trap(grid, rng)
        start = (rng.integers(2, size // 3), rng.integers(2, size // 3))
        goal = (rng.integers(2 * size // 3, size - 2),
                rng.integers(2 * size // 3, size - 2))
        grid[start] = FREE
        grid[goal] = FREE
        if start != goal and shortest_path_len(grid, start, goal) is not None:
            return grid, start, goal
    raise RuntimeError("could not generate a connected world")


def ray_cells(r0, c0, angle, max_t):
    x, y = c0 + 0.5, r0 + 0.5
    dx, dy = np.cos(angle), np.sin(angle)
    cx, cy = c0, r0
    yield (cy, cx), 0.0
    stepx = 1 if dx > 0 else -1
    stepy = 1 if dy > 0 else -1
    inf = float("inf")
    tmx = ((cx + (dx > 0)) - x) / dx if dx != 0 else inf
    tmy = ((cy + (dy > 0)) - y) / dy if dy != 0 else inf
    tdx = abs(1.0 / dx) if dx != 0 else inf
    tdy = abs(1.0 / dy) if dy != 0 else inf
    while True:
        if tmx < tmy:
            cx += stepx
            t = tmx
            tmx += tdx
        else:
            cy += stepy
            t = tmy
            tmy += tdy
        if t > max_t:
            return
        yield (cy, cx), t


def _cast(grid, r0, c0, angle, max_range):
    H, W = grid.shape
    free = []
    for (r, c), t in ray_cells(r0, c0, angle, max_range):
        if not (0 <= r < H and 0 <= c < W):
            return t, True, free, None
        if grid[r, c] == WALL:
            return t, True, free, (r, c)
        free.append((r, c))
    return max_range, False, free, None


def scan_to_cells(pose, scan):
    r0, c0 = pose
    free, occ = set(), set()
    for angle, dist, hit in zip(scan["angles"], scan["ranges"], scan["hit"]):
        wall_cell = None
        for cell, t in ray_cells(r0, c0, angle, dist + 1e-9):
            if hit and t >= dist - 1e-9:
                wall_cell = cell
                break
            free.add(cell)
        if wall_cell is not None:
            occ.add(wall_cell)
    return free, occ


class World:
    def __init__(self, seed, size=40, n_beams=60, lidar_range=6.0, **kw):
        self.grid, self.start, self.goal = generate_world(seed, size, **kw)
        self.size = size
        self.n_beams = n_beams
        self.lidar_range = lidar_range
        self.pose = self.start
        self.path = [self.start]
        self.collisions = 0
        self.seen = set()
        self._free_region = _bfs_free_region(self.grid, self.start)
        self.total_free = len(self._free_region)

    def lidar_scan(self):
        r, c = self.pose
        angles = np.linspace(0, 2 * np.pi, self.n_beams, endpoint=False)
        ranges, hits = [], []
        for a in angles:
            dist, hit, free_cells, wall = _cast(self.grid, r, c, a,
                                                self.lidar_range)
            ranges.append(float(dist))
            hits.append(bool(hit))
            self.seen.update(free_cells)
            if wall is not None:
                self.seen.add(wall)
        return {"pose": self.pose, "goal": self.goal, "shape": self.grid.shape,
                "scan": {"angles": angles.tolist(), "ranges": ranges,
                         "hit": hits, "max_range": self.lidar_range}}

    def step(self, move):
        if move not in MOVES:
            return {"moved": False, "collision": False, "pose": self.pose}
        dr, dc = MOVES[move]
        nr, nc = self.pose[0] + dr, self.pose[1] + dc
        H, W = self.grid.shape
        if 0 <= nr < H and 0 <= nc < W and self.grid[nr, nc] == FREE:
            self.pose = (nr, nc)
            self.path.append(self.pose)
            return {"moved": True, "collision": False, "pose": self.pose}
        self.collisions += 1
        return {"moved": False, "collision": True, "pose": self.pose}

    def at_goal(self):
        return self.pose == self.goal

    def coverage(self):
        seen_free = len(self.seen & self._free_region)
        return seen_free / self.total_free
