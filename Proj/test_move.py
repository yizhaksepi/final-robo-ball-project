#!/usr/bin/env python3
import time

import rclpy
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

TRAJECTORIES = {
    "traj0": [
        {
            "positions": [0.287, -0.0436, -0.4911, -1.073, -1.3184, 5.039713],
            "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
            "time_from_start": Duration(sec=4, nanosec=int(0.4*10**9)),
        },
        {
            "positions": [0.087, -0.0436, -0.4911, -1.073, -1.3184, 5.039713],
            "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
            "time_from_start": Duration(sec=5, nanosec=int(0.4*10**9)),
        },
        {
            "positions": [0.087, -0.2436, -0.4911, -1.073, -1.3184, 5.039713],
            "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
            "time_from_start": Duration(sec=8, nanosec=int(0.4*10**9)),
        },
    ],
    # "traj1": [
    #     {
    #         "positions": [0.974, -0.62, 0.803, -1.916, -1.725, 1.01],
    #         "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
    #         "time_from_start": Duration(sec=3, nanosec=0),
    #     },
    #     {
    #         "positions": [1.186, -0.715, 0.611, -1.34215, -1.69737, 2.724445],
    #         "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
    #         "time_from_start": Duration(sec=6, nanosec=0),
    #     },
    # ],

    # "traj2": [
    #     {
    #         "positions": [1.186, -0.915, 0.811, -0.83215, -1.69737, 2.724445],
    #         "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
    #         "time_from_start": Duration(sec=12, nanosec=0),
    #     },
    #     {
    #         "positions": [1.186, -0.915, 0.4511, -1.54215, -1.69737, 2.724445],
    #         "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
    #         "time_from_start": Duration(sec=12, nanosec=int(0.22*10**9)),
    #     },
    # ],
    "traj2": [
        {
            "positions": [1.5167,-0.3852,-0.5329,-0.2634,-1.5495,3.1128],
            "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
            "time_from_start": Duration(sec=16, nanosec=0),
        },
        {
            "positions": [1.5167,-0.3052,-0.5029,-0.1634,-1.5495,3.1128],
            "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
            "time_from_start": Duration(sec=16, nanosec=int(0.32*10**9)),
        },
        {
            "positions": [1.5167,-0.4852,-0.7229,-0.43270,-1.5495,3.1128],
            "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
            "time_from_start": Duration(sec=16, nanosec=int(0.42*10**9)),
        },
    ],
    # "traj3": [
    #     {
    #         "positions": [1.186, -0.915, 0.811, -0.83215, -1.69737, 2.724445],
    #         "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
    #         "time_from_start": Duration(sec=12, nanosec=0),
    #     },
    #     {
    #         "positions": [1.186, -0.915, 0.811, -0.83215, -1.69737, 2.724445],
    #         "velocities": [0.0, 0.0, 0.0, 0.0, 0.0,0.0],
    #         "time_from_start": Duration(sec=12, nanosec=int(0.26*10**9)),
    #     },
    # ],
}

class JTCClient(rclpy.node.Node):
    """Small test client for the jtc"""

    def __init__(self):
        super().__init__("jtc_client")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self.parse_trajectories()
        self.i = 0
        self._send_goal_future = None
        self._get_result_future = None
        self.execute_next_trajectory()

    def parse_trajectories(self):
        self.goals = {}

        for traj_name in TRAJECTORIES:
            goal = JointTrajectory()
            goal.joint_names = self.joints
            for pt in TRAJECTORIES[traj_name]:
                point = JointTrajectoryPoint()
                point.positions = pt["positions"]
                point.velocities = pt["velocities"]
                point.time_from_start = pt["time_from_start"]
                goal.points.append(point)

            self.goals[traj_name] = goal

    def execute_next_trajectory(self):
        if self.i >= len(self.goals):
            self.get_logger().info("Done with all trajectories")
            return
        traj_name = list(self.goals)[self.i]
        self.i = self.i + 1
        if traj_name:
            self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        self.get_logger().info(f"Executing trajectory {traj_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goals[traj_name]
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Done with result: {self.error_code_to_str(result.error_code)}")
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            time.sleep(5)
            self.execute_next_trajectory()

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"


def main(args=None):
    rclpy.init(args=args)

    jtc_client = JTCClient()
    rclpy.spin(jtc_client)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
