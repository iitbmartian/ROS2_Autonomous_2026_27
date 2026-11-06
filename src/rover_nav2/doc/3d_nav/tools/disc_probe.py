import sys, time, rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState
rclpy.init(); n = Node('disc_probe')
c = n.create_client(GetState, '/bt_navigator/get_state')
t0 = time.time(); ok = c.wait_for_service(timeout_sec=float(sys.argv[1]))
nodes = len(n.get_node_names())
print(f"{'ok' if ok else 'MISS'} {time.time()-t0:.1f}s nodes_seen={nodes}")
