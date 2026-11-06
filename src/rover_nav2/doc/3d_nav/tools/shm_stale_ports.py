#!/usr/bin/env python3
"""Lists (or with --rm removes) Fast DDS SHM port segments whose listener is gone, plus their
named-semaphore mutexes. The owner of a live port runs a dds.shm.<port> thread. A segment left by
a process killed while creating it hangs every participant that opens it (2026-11-06, port 7020).
Same rule as remove_stale_shm_ports in rover_nav2/benchmark/runner.py."""
import glob, os, sys
live = set()
for comm in glob.glob('/proc/[0-9]*/task/[0-9]*/comm'):
    try:
        name = open(comm).read().strip()
    except OSError:
        continue
    if name.startswith('dds.shm.') and name[8:].isdigit():
        live.add(name[8:])
stale = [os.path.basename(p) for p in glob.glob('/dev/shm/fastrtps_port*')
         if not p.endswith('_el') and os.path.basename(p)[13:] not in live]
for s in stale:
    print(s)
    if '--rm' in sys.argv:
        for f in (s, s + '_el', 'sem.' + s + '_mutex'):
            try:
                os.remove('/dev/shm/' + f)
            except OSError:
                pass
print(f'{len(stale)} stale, {len(live)} live listeners', file=sys.stderr)
