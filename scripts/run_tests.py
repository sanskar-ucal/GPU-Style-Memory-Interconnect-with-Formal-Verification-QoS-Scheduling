#!/usr/bin/env python3
"""Dynamic verification regression: directed stress tests + seeded random runs."""
from __future__ import annotations

import argparse
import json
import random
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "traffic"))
import gen_traffic  # noqa: E402

TRACE_DIR = ROOT / "build" / "traces"

# name -> (trace spec, sim args, description)
TESTS = {
    "smoke":        (dict(profile="mixed", load=0.3, cycles=20000, seed=1), [],
                     "light mixed traffic, round-robin"),
    "starvation":   (dict(profile="hog", load=1.0, cycles=30000, seed=2), ["--watch", "3", "--max-lat", "500"],
                     "3 clients flood, DMA latency must stay bounded"),
    "qos_wrr":      (dict(profile="mixed", load=1.2, cycles=20000, seed=3), ["--policy", "wrr"],
                     "saturating traffic under weighted QoS"),
    "qos_prio":     (dict(profile="mixed", load=0.6, cycles=20000, seed=4), ["--policy", "prio"],
                     "fixed-priority arbitration correctness"),
    "backpressure": (dict(profile="mixed", load=0.6, cycles=20000, seed=5), ["--stall", "--bp", "0.3"],
                     "random memory stalls + client RREADY/BREADY backpressure"),
    "rw_hazard":    (dict(profile="rwmix", load=1.0, cycles=20000, seed=6), ["--stall", "--bp", "0.2"],
                     "simultaneous write/read-back of same lines, byte strobes"),
    "reset_mid":    (dict(profile="mixed", load=1.2, cycles=20000, seed=7), ["--reset-at", "7000"],
                     "reset asserted with bursts in flight"),
}


def random_tests(n: int, seed: int = 100):
    rng = random.Random(seed)
    for i in range(n):
        args = ["--policy", rng.choice(["rr", "wrr", "prio"])]
        if rng.random() < 0.5:
            args.append("--stall")
        args += ["--bp", f"{rng.choice([0.0, 0.1, 0.4]):.1f}", "--seed", str(rng.randrange(1 << 16))]
        if rng.random() < 0.3:
            args += ["--reset-at", str(rng.randrange(2000, 15000))]
        spec = dict(profile="mixed", load=round(rng.uniform(0.2, 1.4), 2), cycles=15000, seed=1000 + i)
        yield f"random_{i:02d}", (spec, args, "seeded random config")


def trace_for(spec: dict) -> Path:
    TRACE_DIR.mkdir(parents=True, exist_ok=True)
    path = TRACE_DIR / "{profile}_l{load}_c{cycles}_s{seed}.trace".format(**spec)
    if not path.exists():
        ops = gen_traffic.generate(spec["load"], spec["cycles"], spec["seed"], spec["profile"])
        gen_traffic.write_trace(str(path), ops, json.dumps(spec))
    return path


def run_one(sim: str, name: str, spec: dict, args: list[str], timeout: int = 120) -> dict:
    out_json = TRACE_DIR / f"{name}.json"
    cmd = [sim, "--trace", str(trace_for(spec)), "--quiet", "--json", str(out_json), *args]
    t0 = time.time()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        ok = proc.returncode == 0
        errs = [l.strip()[7:] for l in proc.stdout.splitlines() if l.strip().startswith("ERROR:")]
    except subprocess.TimeoutExpired:
        ok, errs = False, ["simulation hung (host timeout)"]
    return {"name": name, "pass": ok, "errors": errs, "secs": time.time() - t0, "cmd": " ".join(cmd)}


def run_suite(sim: str, names=None, n_random: int = 8, verbose: bool = True):
    suite = list(TESTS.items()) + list(random_tests(n_random))
    if names:
        suite = [t for t in suite if t[0] in names]
    results = []
    for name, (spec, args, desc) in suite:
        r = run_one(sim, name, spec, args)
        results.append(r)
        if verbose:
            tag = "\033[32mPASS\033[0m" if r["pass"] else "\033[31mFAIL\033[0m"
            print(f"  {tag}  {name:<14} {desc:<58} {r['secs']:5.2f}s")
            for e in r["errors"][:2]:
                print(f"          -> {e}")
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sim", default=str(ROOT / "build/sim/Vmem_subsystem_top"))
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--random", type=int, default=8)
    a = ap.parse_args()
    print(f"Running regression with {a.sim}")
    res = run_suite(a.sim, a.only, a.random)
    failed = [r for r in res if not r["pass"]]
    print(f"\n{len(res) - len(failed)}/{len(res)} tests passed")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
