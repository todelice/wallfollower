# import the empty module from std_servs Service interface
from real_robot_interfaces.srv import FindWall
# import the ROS2 Python find_wall_client libraries
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from real_robot_interfaces.action import OdomRecord
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import time


class WallFollowing(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_client
        super().__init__('wall_following')
        self.group1 = ReentrantCallbackGroup()
        self.group2 = ReentrantCallbackGroup()
        self.group3 = ReentrantCallbackGroup()
        self.group4 = ReentrantCallbackGroup()
        self.publisher_ = self.create_publisher(
            Twist, 'cmd_vel', 10, callback_group=self.group2)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), callback_group=self.group1)
        self.odom_recorder_client = ActionClient(
            self, OdomRecord, 'odom_record', callback_group=self.group4)

        self.timer_period = 0.2
        # define the variable to save the received info
        self.laser_right = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)
        # create the Service Client object
        # defines the name and type of the Service Server you will work with.
        self.find_wall_client = self.create_client(
            FindWall, 'find_wall', callback_group=self.group3)
        # checks once per second if a Service matching the type and name of the Client is available.
        while not self.find_wall_client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')

        # create an Empty request
        self.req = FindWall.Request()
        self.find_wall_response = FindWall.Response()
        self.wall_finding = False
        self.wall_finding_completed = False
        self.action_called = False

    def send_request(self):
        self.future = self.find_wall_client.call(self.req)

    def laser_callback(self, msg):
        if self.wall_finding_completed == False:
            pass
        else:
            self.laser_right = msg.ranges[180]
            self.front = msg.ranges[359]

    #Main motion control end#
    def motion(self):
        if not self.wall_finding:
            self.wall_finding = True
            self.send_request()
            self.get_logger().info('service should start')
            self.wall_finding_completed = True
            self.get_logger().info('Wall Following started')

        elif self.wall_finding_completed == True:
            # print the data
            if not self.action_called:
                self.get_logger().info("Action started")
                self.action_called = True
                self.action_send_goal()
            # Logic of move
            if self.front < 0.5:
                self.cmd.linear.x = 0.04
                self.cmd.angular.z = 0.3
            elif self.laser_right > 0.3:
                self.cmd.linear.x = 0.04
                self.cmd.angular.z = -0.1
            elif self.laser_right < 0.2:
                self.cmd.linear.x = 0.04
                self.cmd.angular.z = 0.2
            else:
                self.cmd.linear.x = 0.04
                self.cmd.angular.z = 0.0

            # Publishing the cmd_vel values to a Topic
            self.publisher_.publish(self.cmd)
        else:
            pass
    #Main motion control end#

    #action client functions start#

    def action_send_goal(self):
        goal_msg = OdomRecord.Goal()
        self.odom_recorder_client.wait_for_server()
        self._send_goal_future = self.odom_recorder_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected ')
            return

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Total distance feedback "%s"' %
                               str(feedback.current_total))
    #action client functions end#


def main(args=None):
    rclpy.init(args=args)

    wall_following_node = WallFollowing()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(wall_following_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        wall_following_node.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()
