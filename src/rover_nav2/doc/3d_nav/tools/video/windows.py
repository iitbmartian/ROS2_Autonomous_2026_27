#!/usr/bin/env python3
"""Find a top-level window whose title contains the arg, move and resize it, print its id.
Args: title_substring x y w h."""
import sys, time
from Xlib import display, X
name, x, y, w, h = sys.argv[1], *map(int, sys.argv[2:6])
d = display.Display()
def walk(win):
    try:
        t = win.get_wm_name()
        if isinstance(t, bytes): t = t.decode(errors='ignore')
        if t and name.lower() in t.lower() and win.get_attributes().map_state == X.IsViewable:
            return win
        for c in win.query_tree().children:
            r = walk(c)
            if r: return r
    except Exception:
        pass
for _ in range(120):
    win = walk(d.screen().root)
    if win: break
    time.sleep(1)
else:
    sys.exit('no window ' + name)
win.configure(x=x, y=y, width=w, height=h, stack_mode=X.Above)
d.sync(); time.sleep(1)
print(win.id)
