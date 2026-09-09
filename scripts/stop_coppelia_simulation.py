#!/usr/bin/env python3
"""Stop a local CoppeliaSim simulation while keeping the GUI open."""

import argparse
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=23000)
    args = parser.parse_args()
    sim = RemoteAPIClient(port=args.port).require("sim")
    if int(sim.getSimulationState()) != int(sim.simulation_stopped):
        sim.stopSimulation()
    deadline = time.monotonic() + 10.0
    while int(sim.getSimulationState()) != int(sim.simulation_stopped):
        if time.monotonic() >= deadline:
            raise RuntimeError("simulation did not stop within 10 seconds")
        time.sleep(0.05)
    print("[sim] stopped; GUI remains open", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
