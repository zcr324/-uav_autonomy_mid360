"""Linux process-tree cleanup with PID start-time verification."""
import os,signal,time
from pathlib import Path
def info(pid):
    try:
        t=Path('/proc/%d/stat'%pid).read_text();v=t[t.rfind(')')+2:].split()
        return int(v[1]),v[0],v[19]
    except (FileNotFoundError,ProcessLookupError):return None
def stop_tree(pid,grace=10):
    table={}
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            value=info(int(entry.name))
            if value:table[int(entry.name)]=value
    owned={pid}
    while True:
        newer=owned|{p for p,v in table.items() if v[0] in owned}
        if newer==owned:break
        owned=newer
    starts={p:table[p][2] for p in owned if p in table}
    def alive(p):
        v=info(p);return v is not None and v[1]!='Z' and v[2]==starts.get(p)
    def send(p,sig):
        if alive(p):
            try:os.kill(p,sig)
            except ProcessLookupError:pass
    send(pid,signal.SIGINT)
    for seconds,sig in [(grace,signal.SIGTERM),(3,signal.SIGKILL)]:
        end=time.monotonic()+seconds
        while any(alive(p) for p in owned) and time.monotonic()<end:time.sleep(.1)
        for p in owned:send(p,sig)
    return sorted(owned)
