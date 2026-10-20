#!/usr/bin/env python3
"""Performance verification: contention sweep + crowd-trace replay -> docs/img/*.png"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "traffic"))
import gen_traffic  # noqa: E402

OUT = ROOT / "build" / "perf"
IMG = ROOT / "docs" / "img"
POLICIES = ["rr", "wrr", "prio"]
LABEL = {"rr": "Round-robin", "wrr": "Weighted RR (QoS)", "prio": "Fixed priority"}
COLOR = {"rr": "#4ea8de", "wrr": "#80ed99", "prio": "#ff6b6b"}
CLIENTS = ["GPU0", "GPU1", "CPU", "DMA"]
LOADS = [0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0]
CYCLES = 20000


def sim(binary, trace, policy, extra=()):
    js = OUT / f"{Path(trace).stem}_{policy}.json"
    subprocess.run([binary, "--trace", str(trace), "--policy", policy, "--quiet", "--json", str(js), *extra],
                   check=True, capture_output=True)
    return json.loads(js.read_text())


def jain(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) ** 2 / (len(xs) * sum(x * x for x in xs)) if xs else 1.0


def sweep(binary):
    res = {p: [] for p in POLICIES}
    for load in LOADS:
        trace = OUT / f"mixed_{load}.trace"
        gen_traffic.write_trace(str(trace), gen_traffic.generate(load, CYCLES, seed=11), f"load={load}")
        for p in POLICIES:
            d = sim(binary, trace, p, ["--window", str(CYCLES)])
            served = [c["beats_window"] / c["offered_window"] if c["offered_window"] else None for c in d["clients"]]
            d["offered"] = sum(c["offered_window"] for c in d["clients"]) / CYCLES
            d["fairness"] = jain(served)
            res[p].append(d)
    return res


def crowd(binary, crowd_bin):
    res = {}
    for scen in ["dense", "sparse", "evac"]:
        trace = OUT / f"crowd_{scen}.trace"
        subprocess.run([crowd_bin, "--scenario", scen, "-o", str(trace)], check=True, capture_output=True)
        hdr = trace.read_text().splitlines()[0]
        bursts = [float(x) for x in hdr.split("avg_burst=[")[1].rstrip("]").split(",")]
        res[scen] = {"bursts": bursts, "runs": {p: sim(binary, trace, p) for p in POLICIES}}
    return res


def style(ax, title, xlabel, ylabel):
    ax.set_title(title, fontsize=11, pad=8)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(alpha=0.2)


def plot_dashboard(sw, cr):
    plt.style.use("dark_background")
    fig, axs = plt.subplots(2, 3, figsize=(17, 9.5))
    x = [d["offered"] for d in sw["rr"]]

    ax = axs[0, 0]
    for p in POLICIES:
        ax.plot(x, [d["throughput_window"] for d in sw[p]], "o-", color=COLOR[p], label=LABEL[p])
    ax.plot(x, x, "--", color="gray", alpha=0.6, label="offered")
    style(ax, "Delivered bandwidth vs offered load", "offered (beats/cycle)", "delivered (beats/cycle)")
    ax.legend(fontsize=8)

    ax = axs[0, 1]
    for p in POLICIES:
        ax.semilogy(x, [d["clients"][2]["lat_p99"] for d in sw[p]], "o-", color=COLOR[p], label=f"CPU {LABEL[p]}")
        ax.semilogy(x, [d["clients"][3]["lat_p99"] for d in sw[p]], "s--", color=COLOR[p], alpha=0.6, label=f"DMA {LABEL[p]}")
    style(ax, "Tail latency (p99): CPU solid, DMA dashed", "offered (beats/cycle)", "p99 latency (cycles)")
    ax.legend(fontsize=7)

    ax = axs[0, 2]
    for p in POLICIES:
        ax.plot(x, [d["fairness"] for d in sw[p]], "o-", color=COLOR[p], label=LABEL[p])
    style(ax, "Jain fairness of served/offered bandwidth", "offered (beats/cycle)", "Jain index")
    ax.set_ylim(0.4, 1.02)
    ax.legend(fontsize=8)

    ax = axs[1, 0]
    hi = LOADS.index(1.5)
    w = 0.27
    for i, p in enumerate(POLICIES):
        ax.bar([k + (i - 1) * w for k in range(4)], [c["lat_max"] for c in sw[p][hi]["clients"]], w,
               color=COLOR[p], label=LABEL[p])
    ax.set_xticks(range(4), CLIENTS)
    ax.set_yscale("log")
    style(ax, "Starvation: worst-case latency @ load 1.5", "", "max latency (cycles)")
    ax.legend(fontsize=8)

    ax = axs[1, 1]
    scens = list(cr)
    budget = 8 * 12000
    for i, p in enumerate(POLICIES):
        ax.bar([k + (i - 1) * w for k in range(3)], [cr[s]["runs"][p]["cycles"] / budget for s in scens], w,
               color=COLOR[p], label=LABEL[p])
    ax.axhline(1.0, color="white", ls="--", alpha=0.6)
    ax.set_xticks(range(3), [f"{s}\nGPU burst≈{cr[s]['bursts'][1]:.1f}" for s in scens])
    style(ax, "Crowd-sim replay: frame time / budget", "", "normalised frame time")
    ax.legend(fontsize=8)

    ax = axs[1, 2]
    for i, p in enumerate(POLICIES):
        ax.bar([k + (i - 1) * w for k in range(4)], [c["lat_p99"] for c in cr["dense"]["runs"][p]["clients"]], w,
               color=COLOR[p], label=LABEL[p])
    ax.set_xticks(range(4), CLIENTS)
    ax.set_yscale("log")
    style(ax, "Crowd-sim (dense): p99 latency per client", "", "p99 latency (cycles)")
    ax.legend(fontsize=8)

    fig.suptitle("GPU-style memory interconnect: QoS arbitration under contention", fontsize=14)
    fig.tight_layout()
    fig.savefig(IMG / "perf_dashboard.png", dpi=130)
    plt.close(fig)


def write_summary(sw, cr):
    hi = LOADS.index(1.5)
    lines = ["| policy | throughput @1.5 | CPU p99 | DMA max | fairness | crowd dense frame | crowd sparse frame |",
             "|---|---|---|---|---|---|---|"]
    for p in POLICIES:
        d = sw[p][hi]
        lines.append(f"| {LABEL[p]} | {d['throughput_window']:.3f} | {d['clients'][2]['lat_p99']} | "
                     f"{d['clients'][3]['lat_max']} | {d['fairness']:.2f} | "
                     f"{cr['dense']['runs'][p]['cycles']} | {cr['sparse']['runs'][p]['cycles']} |")
    (ROOT / "docs" / "perf_summary.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sim", default=str(ROOT / "build/sim/Vmem_subsystem_top"))
    ap.add_argument("--crowd", default=str(ROOT / "build/crowd_sim"))
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    IMG.mkdir(parents=True, exist_ok=True)
    sw = sweep(a.sim)
    cr = crowd(a.sim, a.crowd)
    plot_dashboard(sw, cr)
    write_summary(sw, cr)
    print("wrote docs/img/perf_dashboard.png and docs/perf_summary.md")


if __name__ == "__main__":
    main()
