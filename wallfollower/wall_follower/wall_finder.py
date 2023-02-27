from real_robot_interfaces.srv import FindWall
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import rclpy
from nav_msgs.msg import Odometry
import time
import numpy as np
from rclpy.node import Node
import math
# to do select turning side
# to do there is a problem with read value of lidar and selection of that need to solve it


class FindWallService(Node):

    def __init__(self):
        super().__init__('find_wall_service')
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()

        self.srv = self.create_service(
            FindWall, 'find_wall', self.find_wall_callback, callback_group=self.group1)

        self.cmdpubliser_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lasersubscriber_ = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE), callback_group=self.group2)
        self.odomsubscruber_ = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE), callback_group=self.group3)
        self.min_radian_idx = 0
        self.distance_to_wall = 0.0
        self.distance_to_nearest_wall = 10.0
        self.yaw = 0.0
        self.angle_to_rotate = 0.1
        self.rightdistance_to_wall = 0.0
        self.degree_to_turn = 0.1
        # action triggers
        self.first_turn_done = False
        self.nearest_wall_found = False
        # deneme
        self.ranges_array = []

    def find_wall_callback(self, request, response):
        self.get_logger().info('find_wall_callback working')
        msg = Twist()
        while not self.nearest_wall_found:
            self.get_logger().info('Waiting for wall info')
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmdpubliser_.publish(msg)
            time.sleep(0.5)

        if self.nearest_wall_found == True and self.first_turn_done == False:
            self.degree_to_turn = (
                (self.yaw+np.pi+self.degree_to_turn) % (np.pi*2))
            self.get_logger().info('Turning to wall')
            while round(self.yaw, 2) != round(self.degree_to_turn, 2):
                msg.linear.x = 0.0
                msg.angular.z = 0.2
                self.cmdpubliser_.publish(msg)
                # self.get_logger().info('yaw: %s' %
                #                       str(round(self.yaw, 2)))
                # self.get_logger().info('angle to rotate: %s' %
                #                       str(round(self.degree_to_turn, 2)))
            self.first_turn_done = True
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmdpubliser_.publish(msg)

        if self.first_turn_done == True:
            while self.distance_to_wall > 0.3:
                msg.linear.x = 0.03
                msg.angular.z = 0.0
                self.cmdpubliser_.publish(msg)
            while self.rightdistance_to_wall > 0.3:
                msg.linear.x = 0.0
                msg.angular.z = 0.2
                self.cmdpubliser_.publish(msg)
                #self.get_logger().info('Rightdistance_to_wall "%s"' %
                #                       str(self.rightdistance_to_wall))
            self.get_logger().info('Wall found and turned right to it!')
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmdpubliser_.publish(msg)
            response.wallfound = True
            self.get_logger().info('Response: "%s"' % str(response.wallfound))
            return response

    def laser_callback(self, msg):
        if self.nearest_wall_found == False:
            for i in range(len(msg.ranges)):
                self.ranges_array.append(msg.ranges[i])
                if self.distance_to_nearest_wall > msg.ranges[i]:
                    self.distance_to_nearest_wall = msg.ranges[i]
                    self.degree_to_turn = (i*msg.angle_increment)
            #self.min_degree_idx = self.ranges_array.index(mindegreevalue)
            self.get_logger().info('Should turn to this radian is: "%s"' %
                                   str((self.degree_to_turn+np.pi+self.yaw) % np.pi))
            self.get_logger().info('yaw degree is "%s"' % str(self.yaw))

            self.nearest_wall_found = True

            #self.distance_to_nearest_wall = msg.ranges[min_degree_idx]
        self.distance_to_wall = msg.ranges[360]
        self.rightdistance_to_wall = msg.ranges[180]

    def odom_callback(self, msg):
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        siny_cosp = 2*(w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp)
        if self.yaw < 0:
            self.yaw = (2*np.pi)+(self.yaw)


def main(args=None):
    rclpy.init(args=args)

    wall_finding_node = FindWallService()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(wall_finding_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        wall_finding_node.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
