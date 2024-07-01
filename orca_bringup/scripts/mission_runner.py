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
Run the "huge loop" mission with QR code detection

Code inspired by https://github.com/ros2/ros2cli/blob/rolling/ros2action/ros2action/verb/send_goal.py

Usage:
-- ros2 run orca_bringup mission_runner.py
"""

from enum import Enum

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav2_msgs.action import FollowWaypoints
from orca_msgs.action import TargetMode
from rclpy.action import ActionClient
from std_msgs.msg import Header, String


class SendGoalResult(Enum):
    SUCCESS = 0     # Goal succeeded
    FAILURE = 1     # Goal failed
    CANCELED = 2    # Goal canceled (KeyboardInterrupt exception or QR code detected)


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
delay_loop = FollowWaypoints.Goal()
delay_loop.poses.append(make_pose(x=0.0, y=0.0, z=-7.0))
for _ in range(2):
    delay_loop.poses.append(make_pose(x=20.0, y=-13.0, z=-7.0))
    delay_loop.poses.append(make_pose(x=10.0, y=-23.0, z=-7.0))
    delay_loop.poses.append(make_pose(x=-10.0, y=-8.0, z=-7.0))
    delay_loop.poses.append(make_pose(x=0.0, y=0.0, z=-7.0))


class MissionRunner(Node):
    def __init__(self):
        super().__init__('mission_runner')
        self.set_target_mode = ActionClient(self, TargetMode, '/set_target_mode')
        self.follow_waypoints = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.qr_subscription = self.create_subscription(
            String,
            '/qr_code_detected',
            self.qr_callback,
            10)
        self.qr_code_detected = False

    def qr_callback(self, msg):
        self.get_logger().info(f'QR Code detected: {msg.data}')
        self.qr_code_detected = True

    def send_goal(self, action_client, send_goal_msg) -> SendGoalResult:
        goal_handle = None

        try:
            action_client.wait_for_server()

            print('Sending goal...')
            goal_future = action_client.send_goal_async(send_goal_msg)
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()

            if goal_handle is None:
                raise RuntimeError('Exception while sending goal: {!r}'.format(goal_future.exception()))

            if not goal_handle.accepted:
                print('Goal rejected')
                return SendGoalResult.FAILURE

            print('Goal accepted with ID: {}'.format(bytes(goal_handle.goal_id.uuid).hex()))
            result_future = goal_handle.get_result_async()

            while rclpy.ok():
                rclpy.spin_once(self)
                if result_future.done():
                    break
                if self.qr_code_detected:
                    goal_handle.cancel_goal_async()
                    return SendGoalResult.CANCELED

            result = result_future.result()

            if result is None:
                raise RuntimeError('Exception while getting result: {!r}'.format(result_future.exception()))

            print('Goal completed')
            return SendGoalResult.SUCCESS

        except KeyboardInterrupt:
            if (goal_handle is not None and
                    (GoalStatus.STATUS_ACCEPTED == goal_handle.status or
                     GoalStatus.STATUS_EXECUTING == goal_handle.status)):
                print('Canceling goal...')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
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

    def run_mission(self):
        print('>>> Setting mode to AUV <<<')
        if self.send_goal(self.set_target_mode, go_auv) == SendGoalResult.SUCCESS:
            print('>>> Executing mission <<<')
            result = self.send_goal(self.follow_waypoints, delay_loop)

            if result == SendGoalResult.CANCELED:
                print('>>> QR Code found, mission complete <<<')
            else:
                print('>>> Mission complete, returning home <<<')
                self.send_goal(self.follow_waypoints, go_home)

            print('>>> Setting mode to ROV <<<')
            self.send_goal(self.set_target_mode, go_rov)

            print('>>> Mission complete <<<')
        else:
            print('>>> Failed to set mode to AUV, quit <<<')


def main():
    rclpy.init()

    try:
        mission_runner = MissionRunner()
        mission_runner.run_mission()
    finally:
        mission_runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
