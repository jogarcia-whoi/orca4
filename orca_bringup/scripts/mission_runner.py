#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 Clyde McQueen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Run the "huge loop" mission

Code inspired by https://github.com/ros2/ros2cli/blob/rolling/ros2action/ros2action/verb/send_goal.py

Usage:
-- ros2 run orca_bringup mission_runner.py
"""

from enum import Enum

import rclpy
import rclpy.logging
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from orca_msgs.action import TargetMode
from rclpy.action import ActionClient
from std_msgs.msg import Header
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

import sys

# A simple client that queries the base_controller node for the values of the lawnmower params that we need.
# Adapted from https://daobook.github.io/ros2-docs/xin/Tutorials/Writing-A-Simple-Py-Service-And-Client.html
class ParamGetter(Node):
    def __init__(self):
        super().__init__('ParamGetter')
        self.cli = self.create_client(GetParameters, '/base_controller/get_parameters')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            print("WAITING FOR SERVICE")
            self.get_logger().info('service not available, waiting again...')
        self.req = GetParameters.Request()

    def send_request(self):
        self.req.names = ["area_h","area_w","lane_spacing"]
        self.future = self.cli.call_async(self.req)

class SendGoalResult(Enum):
    SUCCESS = 0     # Goal succeeded
    FAILURE = 1     # Goal failed
    CANCELED = 2    # Goal canceled (KeyboardInterrupt exception)

def make_pose(x: float, y: float, z: float):
    return PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=x, y=y, z=z)))


# Go to AUV mode
go_auv = TargetMode.Goal()
go_auv.target_mode = TargetMode.Goal.ORCA_MODE_AUV

# Go to ROV mode
go_rov = TargetMode.Goal()
go_rov.target_mode = TargetMode.Goal.ORCA_MODE_ROV

# Go home (1m deep)
go_home = FollowWaypoints.Goal()
go_home.poses.append(make_pose(x=0.0, y=0.0, z=-1.0))

# Dive to 8m
dive = FollowWaypoints.Goal()
dive.poses.append(make_pose(x=0.0, y=0.0, z=-8.0))

# Big loop, will eventually result in a loop closure
lawnmower = FollowWaypoints.Goal()
lawnmower.poses.append(make_pose(x=0.0, y=0.0, z=-7.0))

# Send a goal to an action server and wait for the result.
# Cancel the goal if the user hits ^C (KeyboardInterrupt).
def send_goal(node, action_client, send_goal_msg) -> SendGoalResult:
    goal_handle = None

    try:
        action_client.wait_for_server()

        print('Sending goal...')
        goal_future = action_client.send_goal_async(send_goal_msg)
        rclpy.spin_until_future_complete(node, goal_future)
        goal_handle = goal_future.result()

        if goal_handle is None:
            raise RuntimeError('Exception while sending goal: {!r}'.format(goal_future.exception()))

        if not goal_handle.accepted:
            print('Goal rejected')
            return SendGoalResult.FAILURE

        print('Goal accepted with ID: {}'.format(bytes(goal_handle.goal_id.uuid).hex()))
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)

        result = result_future.result()

        if result is None:
            raise RuntimeError('Exception while getting result: {!r}'.format(result_future.exception()))

        print('Goal completed')
        return SendGoalResult.SUCCESS

    except KeyboardInterrupt:
        # Cancel the goal if it's still active
        # TODO(clyde): this seems to work, but a second exception is generated -- why?
        if (goal_handle is not None and
                (GoalStatus.STATUS_ACCEPTED == goal_handle.status or
                 GoalStatus.STATUS_EXECUTING == goal_handle.status)):
            print('Canceling goal...')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future)
            cancel_response = cancel_future.result()

            if cancel_response is None:
                raise RuntimeError('Exception while canceling goal: {!r}'.format(cancel_future.exception()))

            if len(cancel_response.goals_canceling) == 0:
                raise RuntimeError('Failed to cancel goal')
            if len(cancel_response.goals_canceling) > 1:
                raise RuntimeError('More than one goal canceled')
            if cancel_response.goals_canceling[0].goal_id != goal_handle.goal_id:
                raise RuntimeError('Canceled goal with incorrect goal ID')

            print('Goal canceled')
            return SendGoalResult.CANCELED


def main():
    node = None
    set_target_mode = None
    follow_waypoints = None
    
    LANE_SPACING = 0.0
    AREA_W = 0.0
    AREA_H = 0.0

    rclpy.init()

    try:
        node = rclpy.create_node("mission_runner")

        ## Get Parameters that we need to build the lawnmower pattern
        minimal_client = ParamGetter()
        minimal_client.send_request()
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info('Service call failed %r' % (e,))
            else:
                print("Parameters obtained")
                AREA_H = response.values[0].double_value
                AREA_W = response.values[1].double_value
                LANE_SPACING = response.values[2].double_value

        ## Create Lawnmower pattern
        
        # calc waypoints
        num_waypoints = ((AREA_W // LANE_SPACING) + 1) * 2
        print("Num of waypoints: {}".format(num_waypoints))
        # TODO: Go to start point
        # start pattern
        bot_x_pos = 0.0
        bot_y_pos = 0.0
        
        y_offset = AREA_H * 1.0
        for wp in range(int(num_waypoints)):
            if wp == 0:
                bot_y_pos = AREA_H
                lawnmower.poses.append(make_pose(x=bot_x_pos, y=bot_y_pos, z=-7.0))
                print("{}, {}, {}, {}".format(bot_x_pos, bot_y_pos, wp, y_offset))
            elif wp % 2 != 0:
                bot_x_pos += LANE_SPACING
                lawnmower.poses.append(make_pose(x=bot_x_pos, y=bot_y_pos, z=-7.0))
                print("{}, {}, {}, {}".format(bot_x_pos, bot_y_pos, wp, y_offset))
            else:
                y_offset *= -1.0
                bot_y_pos += y_offset
                lawnmower.poses.append(make_pose(x=bot_x_pos, y=bot_y_pos, z=-7.0))
                print("{}, {}, {}, {}".format(bot_x_pos, bot_y_pos, wp, y_offset))

        ### Creating Action clients
        set_target_mode = ActionClient(node, TargetMode, '/set_target_mode')
        follow_waypoints = ActionClient(node, FollowWaypoints, '/follow_waypoints')

        print('>>> Setting mode to AUV <<<')
        if send_goal(node, set_target_mode, go_auv) == SendGoalResult.SUCCESS:
            print('>>> Executing mission <<<')
            send_goal(node, follow_waypoints, lawnmower)

            print('>>> Setting mode to ROV <<<')
            send_goal(node, set_target_mode, go_rov)

            print('>>> Mission complete <<<')
        else:
            print('>>> Failed to set mode to AUV, quit <<<')

    finally:
        if set_target_mode is not None:
            set_target_mode.destroy()
        if follow_waypoints is not None:
            follow_waypoints.destroy()
        if node is not None:
            node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
