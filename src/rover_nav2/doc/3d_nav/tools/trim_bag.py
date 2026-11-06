#!/usr/bin/env python3
"""Copy the first N seconds of a bag to a new mcap bag. Args: in, out, seconds."""
import sys, rosbag2_py
src, dst, sec = sys.argv[1], sys.argv[2], float(sys.argv[3])
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=src), rosbag2_py.ConverterOptions('', ''))
w = rosbag2_py.SequentialWriter(); w.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
for t in r.get_all_topics_and_types(): w.create_topic(t)
t0 = None; n = 0
while r.has_next():
    tp, data, t = r.read_next(); t0 = t0 or t
    if (t - t0) * 1e-9 > sec: break
    w.write(tp, data, t); n += 1
del w; print(n, 'messages')
