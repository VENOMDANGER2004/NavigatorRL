import numpy as np
import matplotlib.pyplot as plt

from world import FREE, scan_to_cells


def draw_run(world, nav, seed):
    grid = world.grid
    H, W = grid.shape

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6.2))

    ax1.imshow(grid, cmap="Greys", origin="upper", vmin=0, vmax=1)
    path = np.array(world.path)
    ax1.plot(path[:, 1], path[:, 0], "-", color="#1f77b4", lw=1.6, alpha=0.9)
    r, c = world.pose
    scan = world.lidar_scan()["scan"]
    for a, dist, hit in zip(scan["angles"], scan["ranges"], scan["hit"]):
        x, y = c + 0.5 + np.cos(a) * dist, r + 0.5 + np.sin(a) * dist
        ax1.plot([c + 0.5, x], [r + 0.5, y], color="#d62728", lw=0.4, alpha=0.35)
        if hit:
            ax1.plot(x, y, ".", color="#d62728", ms=3)
    ax1.plot(world.start[1], world.start[0], "o", color="#2ca02c", ms=10, label="start")
    ax1.plot(world.goal[1], world.goal[0], "*", color="#ff7f0e", ms=16, label="goal")
    ax1.plot(c, r, "s", color="#1f77b4", ms=7, label="robot")
    ax1.set_title(f"true world (seed {seed})")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_xticks([]); ax1.set_yticks([])

    disc = np.full((H, W), 0.5)
    for (rr, cc), v in nav.known.items():
        if 0 <= rr < H and 0 <= cc < W:
            disc[rr, cc] = 0.0 if v == FREE else 1.0
    ax2.imshow(disc, cmap="Greys", origin="upper", vmin=0, vmax=1)
    ax2.plot(path[:, 1], path[:, 0], "-", color="#1f77b4", lw=1.2, alpha=0.8)
    ax2.plot(world.goal[1], world.goal[0], "*", color="#ff7f0e", ms=16)
    ax2.set_title(f"mapped ({world.coverage():.0%} covered)")
    ax2.set_xticks([]); ax2.set_yticks([])

    plt.tight_layout()
    out = f"run_seed{seed}.png"
    plt.savefig(out, dpi=110)
    print("saved", out)


if __name__ == "__main__":
    import sys
    from run import run_one
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    r = run_one(seed, 40, 60, 10.0, 1000)
    draw_run(r["world"], r["nav"], seed)
