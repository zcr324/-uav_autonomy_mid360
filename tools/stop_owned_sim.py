#!/usr/bin/env python3
"""Stop only the simulation tree whose PID was recorded by run_sim.sh."""
import os
import signal
import time
from pathlib import Path

root = Path(__file__).resolve().parents[1]
pidfile = root / "integration/logs/simulation.pid"
if not pidfile.exists():
    raise SystemExit("No simulation PID recorded by this workspace")
pid = int(pidfile.read_text().strip())

def proc_info(ident):
    try:
        text = Path("/proc/{}/stat".format(ident)).read_text()
        values = text[text.rfind(")")+2:].split()
        return int(values[1]), values[0], values[19]
    except (FileNotFoundError, ProcessLookupError):
        return None

cmdpath = Path("/proc/{}/cmdline".format(pid))
if not cmdpath.exists():
    raise SystemExit("Recorded simulation already stopped")
command = cmdpath.read_bytes().replace(bytes([0]), b" ").decode(errors="replace")
if "roslaunch" not in command or "sim_env" not in command or "autonomy.launch" not in command:
    raise SystemExit("Recorded PID is not the expected framework launch; refusing to signal it")
table = {}
for entry in Path("/proc").iterdir():
    if entry.name.isdigit():
        info = proc_info(int(entry.name))
        if info: table[int(entry.name)] = info
owned = {pid}
while True:
    children = {ident for ident, info in table.items() if info[0] in owned}
    new = owned | children
    if new == owned: break
    owned = new
starts = {ident: table[ident][2] for ident in owned if ident in table}

def alive(ident):
    info = proc_info(ident)
    return info is not None and info[1] != "Z" and info[2] == starts.get(ident)
def send(ident, sig):
    if alive(ident):
        try: os.kill(ident, sig)
        except ProcessLookupError: pass
send(pid, signal.SIGINT)
for duration, sig in [(15, signal.SIGTERM), (5, signal.SIGKILL)]:
    end = time.monotonic()+duration
    while any(alive(ident) for ident in owned) and time.monotonic()<end: time.sleep(.2)
    for ident in owned: send(ident, sig)
print("Stopped only recorded simulation PID {} and its captured descendants".format(pid))
pidfile.unlink(missing_ok=True)
