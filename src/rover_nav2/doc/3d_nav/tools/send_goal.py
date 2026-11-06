"""Send one NavigateToPose goal and wait for its result, retrying discovery with a fresh node.

    python3 send_goal.py X Y YAW TIMEOUT_S

Prints 'accepted' and 'finished STATUS'. Exit 0 on SUCCEEDED, 1 otherwise, 2 if never accepted.
A fresh CLI process sometimes never discovers bt_navigator on the 50-node graph, so each
attempt waits 60 s for the server and then starts over with a new context.
"""
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}


def attempt(x, y, yaw, timeout):
    rclpy.init()
    n = Node('send_goal_probe')
    try:
        ac = ActionClient(n, NavigateToPose, '/navigate_to_pose')
        if not ac.wait_for_server(timeout_sec=60.0):
            print('no action server after 60 s, retrying', flush=True)
            return None
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x, g.pose.pose.position.y = x, y
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        f = ac.send_goal_async(g)
        rclpy.spin_until_future_complete(n, f, timeout_sec=30.0)
        h = f.result()
        if h is None or not h.accepted:
            print('goal not accepted, retrying', flush=True)
            return None
        print('accepted', flush=True)
        r = h.get_result_async()
        rclpy.spin_until_future_complete(n, r, timeout_sec=timeout)
        if not r.done():
            h.cancel_goal_async()
            print('finished TIMEOUT', flush=True)
            return 1
        st = r.result().status
        print(f'finished {STATUS.get(st, st)}', flush=True)
        return 0 if st == GoalStatus.STATUS_SUCCEEDED else 1
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


def main():
    x, y, yaw, timeout = (float(v) for v in sys.argv[1:5])
    for _ in range(4):
        rc = attempt(x, y, yaw, timeout)
        if rc is not None:
            return rc
        time.sleep(5)
    print('never accepted', flush=True)
    return 2


if __name__ == '__main__':
    sys.exit(main())
