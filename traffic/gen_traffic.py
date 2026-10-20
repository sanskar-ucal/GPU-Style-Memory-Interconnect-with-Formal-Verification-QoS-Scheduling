#!/usr/bin/env python3
"""Synthetic SoC traffic generator.

Emits a trace of AXI bursts for four clients competing for one memory port:

  client 0  GPU0  sequential streaming (16-beat bursts, mostly reads)
  client 1  GPU1  sequential streaming, different stream base
  client 2  CPU   random, short (1/4-beat) accesses, latency sensitive
  client 3  DMA   bulk 32-beat transfers in periodic bursts (copy engine)

Trace format (one burst per line):   <cycle> <client> <R|W> <byte_addr> <axi_len>

`--load` scales the offered bandwidth; 1.0 is roughly the saturation point of
the interconnect for this mix.
"""
from __future__ import annotations

import argparse
import random
from dataclasses import dataclass

REGION_BYTES = 0x4000

# baseline offered beats/cycle per client at load = 1.0
BASE_RATE = {0: 0.20, 1: 0.20, 2: 0.06, 3: 0.14}


@dataclass
class Op:
    cycle: int
    client: int
    write: bool
    addr: int
    length: int  # AXI LEN = beats - 1

    def line(self) -> str:
        return f"{self.cycle} {self.client} {'W' if self.write else 'R'} 0x{self.addr:05x} {self.length}"


def _arrivals(rng: random.Random, rate_ops: float, cycles: int):
    """Poisson arrival times for a given ops/cycle rate."""
    if rate_ops <= 0:
        return
    t = 0.0
    while True:
        t += rng.expovariate(rate_ops)
        if t >= cycles:
            return
        yield int(t)


def gpu(rng, client, load, cycles):
    beats = 16
    addr = rng.randrange(0, REGION_BYTES, 64)
    for t in _arrivals(rng, BASE_RATE[client] * load / beats, cycles):
        yield Op(t, client, rng.random() < 0.2, addr, beats - 1)
        addr = (addr + beats * 4) % REGION_BYTES


def cpu(rng, client, load, cycles):
    mean_beats = 0.75 * 1 + 0.25 * 4
    for t in _arrivals(rng, BASE_RATE[client] * load / mean_beats, cycles):
        beats = 4 if rng.random() < 0.25 else 1
        yield Op(t, client, rng.random() < 0.35, rng.randrange(0, REGION_BYTES, 4), beats - 1)


def dma(rng, client, load, cycles):
    beats, per_batch = 32, 8
    batch_rate = BASE_RATE[client] * load / (beats * per_batch)
    addr = 0
    for t in _arrivals(rng, batch_rate, cycles):
        for k in range(per_batch):  # read source, write destination
            write = k % 2 == 1
            yield Op(t + k, client, write, (addr + (0x2000 if write else 0)) % REGION_BYTES, beats - 1)
            if write:
                addr = (addr + beats * 4) % 0x2000


def hog(rng, client, load, cycles):
    """Clients 0-2 flood the fabric; DMA issues sparse probes whose latency
    must stay bounded if arbitration is starvation-free."""
    if client == 3:
        for t in _arrivals(rng, 1 / 400, cycles):
            yield Op(t, client, False, rng.randrange(0, REGION_BYTES, 64), 7)
    else:
        yield from PROFILES[client](rng, client, 3.0 * load, cycles)


def rw_mix(rng, client, load, cycles):
    """Write-then-read-back of the same lines from every client (hazard path)."""
    rate = 0.012 * load
    for t in _arrivals(rng, rate, cycles):
        beats = rng.choice([1, 2, 4, 8])
        addr = rng.randrange(0, 64) * 64
        yield Op(t, client, True, addr, beats - 1)
        yield Op(t + 1, client, False, addr, beats - 1)


PROFILES = {0: gpu, 1: gpu, 2: cpu, 3: dma}


def generate(load: float, cycles: int, seed: int, profile: str = "mixed", clients=(0, 1, 2, 3)):
    rng = random.Random(seed)
    ops = []
    for c in clients:
        gen = {"rwmix": rw_mix, "hog": hog}.get(profile, PROFILES[c])
        ops.extend(gen(random.Random(rng.random()), c, load, cycles))
    ops.sort(key=lambda o: (o.cycle, o.client))
    return ops


def write_trace(path: str, ops, header: str = ""):
    with open(path, "w") as f:
        f.write(f"# {header}\n# cycle client R/W addr len\n")
        for op in ops:
            f.write(op.line() + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--load", type=float, default=0.5)
    ap.add_argument("--cycles", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--profile", choices=["mixed", "rwmix", "hog"], default="mixed")
    ap.add_argument("--clients", default="0,1,2,3")
    args = ap.parse_args()
    clients = tuple(int(c) for c in args.clients.split(","))
    ops = generate(args.load, args.cycles, args.seed, args.profile, clients)
    write_trace(args.out, ops, f"profile={args.profile} load={args.load} cycles={args.cycles} seed={args.seed}")
    print(f"wrote {len(ops)} bursts to {args.out}")


if __name__ == "__main__":
    main()
