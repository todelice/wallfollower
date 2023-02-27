import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Odometry
from real_robot_interfaces.action import OdomRecord
from geometry_msgs.msg import Point32
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import numpy as np
import copy
import time


class OdomRecorderServer(Node):

    def __init__(self):
        super().__init__('Odom_recorder')

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.Odom_Recorder_action_server = ActionServer(
            self,
            OdomRecord,
            'odom_record',
            self.execute_callback,
            callback_group=self.group1
        )
        self.odomsubscruber = self.create_subscription(Odometry,
                                                       '/odom',
                                                       self.odom_callback,
                                                       QoSProfile(
                                                           depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE),
                                                       callback_group=self.group1)
        self.complete_lap_timer = self.create_timer(
            1.0, self.check_complete_lap, callback_group=self.group2)

        self.first_odom_saved = False
        self.first_odom = Point32()
        self.last_odom = Point32()
        self.last_point = Point32()
        self.first_movement = False
        self.odom_record = []
        self.lap_finished = False
        self.total_distance = 0.0

    def check_complete_lap(self):
        if self.first_movement:
            self.distance_to_start = np.sqrt(
                (self.first_odom.x - self.last_odom.x)**2 + (self.first_odom.y-self.last_odom.y)**2)
            self.get_logger().info("Distance started: '%s' " % str(self.distance_to_start))
            if self.distance_to_start <= 0.5 and len(self.odom_record) > 40:
                self.lap_finished = True
                self.last_point = copy.deepcopy(self.last_odom)
                self.get_logger().info('Lap finished')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = OdomRecord.Feedback()
        last_odom = self.first_odom
        self.get_logger().info('Last odom: "%s"' % str(last_odom))
        self.odom_record.append(last_odom)
        while not self.lap_finished:
            time.sleep(1)
            current_odom = self.last_odom
            del_x = np.power(current_odom.x - last_odom.x, 2)
            del_y = np.power(current_odom.y - last_odom.y, 2)
            dist = np.sqrt(del_x + del_y)

            self.total_distance += dist

            self.get_logger().info('Current distance feedback: "%s"' %
                                   str(self.total_distance))
            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)
            self.odom_record.append(current_odom)

            last_odom = copy.deepcopy(current_odom)

            if not self.first_movement:
                self.first_movement = True
                self.get_logger().info('Action server started to check distance')
        self.odom_record.append(self.last_point)
        goal_handle.succeed()

        result = OdomRecord.Result()
        result.list_of_odoms = self.odom_record
        self.get_logger().info('Action Finished, Recorded Points: "%s"' %
                               str(len(self.odom_record)))
        return result

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not self.first_odom_saved:
            self.first_odom_saved = True
            self.first_odom.x = x
            self.first_odom.y = y
            self.get_logger().info('First odom got')
        else:
            self.last_odom.x = x
            self.last_odom.y = y

    def get_odom(self):
        return self.last_odom


def main(args=None):
    rclpy.init(args=args)

    odom_recorder_node = OdomRecorderServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(odom_recorder_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        odom_recorder_node.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
