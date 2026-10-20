"""Headless Monte Carlo benchmark for the drone swarm.

Usage:
    python benchmark.py                                   # original file, 30 runs
    python benchmark.py --module drone_rescue_simulation_fixed --runs 100

Metric definitions (state these in the viva):
  discovery latency  = mean over victims of the step at which a search drone first stood on it
  rescue turnaround  = mean over victims of (step medicine delivered - step found)
  completion time    = step at which all victims are rescued AND both responders are back IDLE
  hazard collision   = a drone ends a step on a hazard cell (counted per drone per step)
  success            = run completed within --cap steps
"""
import os, sys, argparse, random, statistics as st, importlib
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ap = argparse.ArgumentParser()
ap.add_argument("--module", default="drone_rescue_simulation")
ap.add_argument("--runs", type=int, default=30)
ap.add_argument("--cap", type=int, default=1500)
ap.add_argument("--seed0", type=int, default=0)
a = ap.parse_args()
D = importlib.import_module(a.module)

rows = []
for i in range(a.runs):
    random.seed(a.seed0 + i)
    sim = D.Simulation()
    found, resc, resp_coll, any_coll, fin = {}, {}, 0, 0, None
    for t in range(1, a.cap + 1):
        sim.update()
        for d in sim.response_drones:
            if sim.grid[d.y][d.x] == 1: resp_coll += 1
        for d in sim.response_drones + sim.search_drones:
            if sim.grid[d.y][d.x] == 1: any_coll += 1
        for v in sim.victims_found:   found.setdefault(v, t)
        for v in sim.victims_rescued: resc.setdefault(v, t)
        if len(sim.victims_rescued) == sim.num_victims and all(d.state == "IDLE" for d in sim.response_drones):
            fin = t; break
    rows.append(dict(seed=a.seed0 + i, fin=fin, resp_coll=resp_coll, any_coll=any_coll,
                     disc=st.mean(found.values()) if found else None,
                     turn=st.mean(resc[v] - found[v] for v in resc) if resc else None))

ok = [r for r in rows if r["fin"]]
def ms(xs):
    return f"{st.mean(xs):7.1f} +/- {st.stdev(xs) if len(xs) > 1 else 0:5.1f}   [min {min(xs):.1f}, max {max(xs):.1f}]"
print(f"\nModule: {a.module}   runs: {a.runs}   cap: {a.cap} steps")
print("-" * 78)
print(f"Success rate                : {len(ok)}/{len(rows)} = {100*len(ok)/len(rows):.1f}%")
if ok:
    print(f"Mission completion (steps)  : {ms([r['fin'] for r in ok])}")
    print(f"Victim discovery latency    : {ms([r['disc'] for r in ok])}")
    print(f"Rescue turnaround           : {ms([r['turn'] for r in ok])}")
print(f"Runs with response-drone hazard collision : {sum(1 for r in rows if r['resp_coll'])}/{len(rows)}")
print(f"Runs with ANY drone on hazard cell        : {sum(1 for r in rows if r['any_coll'])}/{len(rows)}")
bad = [r['seed'] for r in rows if not r['fin']]
if bad: print(f"Seeds that did not finish   : {bad}")
