import sys
import numpy as np
import matplotlib.pyplot as plt

from world import World, FREE
from navigator import Navigator


def main():
    seed = 0
    for a in sys.argv[1:]:
        if a.isdigit():
            seed = int(a)
    fast = "--fast" in sys.argv

    world = World(seed=seed)
    nav = Navigator(world.grid.shape, world.start, world.goal)
    H, W = world.grid.shape

    plt.ion()
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 6.2))

    budget, steps = 1000, 0
    while not world.at_goal() and steps < budget:
        obs = world.lidar_scan()
        move = nav.act(obs)
        world.step(move)
        steps += 1

        if steps % (5 if fast else 1) == 0 or world.at_goal():
            ax1.clear(); ax2.clear()
            ax1.imshow(world.grid, cmap="Greys", origin="upper", vmin=0, vmax=1)
            r, c = world.pose
            scan = obs["scan"]
            for ang, dist, hit in zip(scan["angles"], scan["ranges"], scan["hit"]):
                x, y = c + 0.5 + np.cos(ang) * dist, r + 0.5 + np.sin(ang) * dist
                ax1.plot([c + 0.5, x], [r + 0.5, y], color="#d62728", lw=0.3, alpha=0.3)
                if hit:
                    ax1.plot(x, y, ".", color="#d62728", ms=2)
            p = np.array(world.path)
            ax1.plot(p[:, 1], p[:, 0], "-", color="#1f77b4", lw=1.3)
            ax1.plot(world.goal[1], world.goal[0], "*", color="#ff7f0e", ms=15)
            ax1.plot(c, r, "s", color="#1f77b4", ms=6)
            ax1.set_title(f"seed {seed}  step {steps}")
            ax1.set_xticks([]); ax1.set_yticks([])

            disc = np.full((H, W), 0.5)
            for (rr, cc), v in nav.known.items():
                if 0 <= rr < H and 0 <= cc < W:
                    disc[rr, cc] = 0.0 if v == FREE else 1.0
            ax2.imshow(disc, cmap="Greys", origin="upper", vmin=0, vmax=1)
            ax2.plot(p[:, 1], p[:, 0], "-", color="#1f77b4", lw=1.0)
            ax2.set_title(f"mapped {world.coverage():.0%}")
            ax2.set_xticks([]); ax2.set_yticks([])
            plt.pause(0.001)

    print(f"done: reached={world.at_goal()} coverage={world.coverage():.0%} "
          f"steps={steps} collisions={world.collisions}")
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
