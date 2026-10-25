import argparse
import numpy as np

from world import World, shortest_path_len
from navigator import Navigator

TRAIN_SEEDS = list(range(15))


def run_one(seed, size, n_beams, lidar_range, budget, verbose=False):
    world = World(seed=seed, size=size, n_beams=n_beams, lidar_range=lidar_range)
    opt = shortest_path_len(world.grid, world.start, world.goal)
    nav = Navigator(world.grid.shape, world.start, world.goal)

    steps = 0
    while not world.at_goal() and steps < budget:
        world.step(nav.act(world.lidar_scan()))
        steps += 1

    success = world.at_goal()
    actual = len(world.path) - 1
    cov = world.coverage()
    spl = (opt / max(actual, opt)) if success else 0.0
    if verbose:
        print(f"  seed {seed:3d}  {'OK ' if success else 'FAIL'}  "
              f"cov {cov:5.0%}  spl {spl:.2f}  steps {actual:4d}  "
              f"opt {opt:3d}  collisions {world.collisions}")
    return {"seed": seed, "success": success, "coverage": cov, "spl": spl,
            "collisions": world.collisions, "world": world, "nav": nav}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seeds", type=int, nargs="*", default=TRAIN_SEEDS)
    p.add_argument("--size", type=int, default=40)
    p.add_argument("--beams", type=int, default=60, dest="n_beams")
    p.add_argument("--range", type=float, default=6.0, dest="lidar_range")
    p.add_argument("--budget", type=int, default=1000)
    p.add_argument("--viz", type=int, default=None)
    args = p.parse_args()

    res = [run_one(s, args.size, args.n_beams, args.lidar_range, args.budget,
                   verbose=True) for s in args.seeds]

    succ = np.mean([r["success"] for r in res])
    cov = np.mean([r["coverage"] for r in res])
    spl = np.mean([r["spl"] for r in res])
    coll = np.mean([r["collisions"] for r in res])
    score = np.mean([r["success"] * (0.65 * r["coverage"] + 0.35 * r["spl"])
                     for r in res])
    print("\n" + "=" * 54)
    print(f"success rate      {succ:.0%}")
    print(f"mean coverage     {cov:.0%}")
    print(f"mean SPL          {spl:.2f}")
    print(f"mean collisions   {coll:5.1f}")
    print(f"combined score    {score:.2f}")
    print("=" * 54)

    if args.viz is not None:
        from viz import draw_run
        r = run_one(args.viz, args.size, args.n_beams, args.lidar_range,
                    args.budget)
        draw_run(r["world"], r["nav"], r["seed"])


if __name__ == "__main__":
    main()
