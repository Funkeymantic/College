import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from action_msgs.msg._goal_status import GoalStatus

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, RotateAngle, Dock
from irobot_create_msgs.msg import HazardDetectionVector

from pynput.keyboard import KeyCode
from key_commander import KeyCommander
from threading import Lock
from rclpy.executors import MultiThreadedExecutor
import random

# Lock for multithreading
lock = Lock()

class Slash(Node):
    def __init__(self, namespace):
        super().__init__('slasher')

        # Callback Groups
        cb_Subscription = MutuallyExclusiveCallbackGroup()
        cb_Action = MutuallyExclusiveCallbackGroup()

        # Subscriptions
        self.subscription = self.create_subscription(
            HazardDetectionVector, f'/{namespace}/hazard_detection',
            self.listener_callback, qos_profile_sensor_data,
            callback_group=cb_Subscription
        )

        # Action Clients
        self._undock_ac = ActionClient(self, Undock, f'/{namespace}/undock', callback_group=cb_Action)
        self._drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance', callback_group=cb_Action)
        self._rotate_ac = ActionClient(self, RotateAngle, f'/{namespace}/rotate_angle', callback_group=cb_Action)
        self._dock_ac = ActionClient(self, Dock, f'/{namespace}/dock', callback_group=cb_Action)

        # Variables
        self._goal_uuid = None

    def send_action_goal(self, action_client, goal):
        with lock:
            handle = action_client.send_goal_async(goal)
            while not handle.done():
                pass
            self._goal_uuid = handle.result()
        
        while self._goal_uuid is not None and self._goal_uuid.status in (GoalStatus.STATUS_UNKNOWN, GoalStatus.STATUS_ACCEPTED):
            pass
        
        self._goal_uuid = None

    def undock(self):
        self._undock_ac.wait_for_server()
        goal = Undock.Goal()
        self.send_action_goal(self._undock_ac, goal)
        self.get_logger().info("Robot undocked successfully.")

    def drive_random_distance(self):
        distance = random.uniform(1.0, 4.0)
        self._drive_ac.wait_for_server()
        goal = DriveDistance.Goal()
        goal.distance = distance
        self.send_action_goal(self._drive_ac, goal)
        self.get_logger().info(f"Robot drove {distance:.2f} meters.")

    def rotate_random_angle(self):
        angle = random.uniform(90.0, 270.0)
        self._rotate_ac.wait_for_server()
        goal = RotateAngle.Goal()
        goal.angle = angle
        self.send_action_goal(self._rotate_ac, goal)
        self.get_logger().info(f"Robot rotated {angle:.2f} degrees.")

    def dock(self):
        self._dock_ac.wait_for_server()
        goal = Dock.Goal()
        self.send_action_goal(self._dock_ac, goal)
        self.get_logger().info("Robot docked successfully.")

    def execute(self):
        self.undock()
        for _ in range(2):
            self.drive_random_distance()
            self.rotate_random_angle()
        self.dock()

    def listener_callback(self, msg):
        if self._goal_uuid is None:
            return
        for detection in msg.detections:
            if detection.type == 1:  # Bump
                self.get_logger().warning("Hazard detected. Canceling goal.")
                with lock:
                    self._goal_uuid.cancel_goal_async()
                break


def main():
    rclpy.init()
    namespace = 'create3_05F8'
    node = Slash(namespace)
    executor = MultiThreadedExecutor(2)
    executor.add_node(node)

    try:
        node.get_logger().info("Starting robot sequence...")
        node.execute()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
