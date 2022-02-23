# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Int32
import time
import array as arr




class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/current_goal', 10)
        
        self.CurrentGoal = PoseStamped()
        self.goal_pose1 = PoseStamped()
        self.goal_pose1.header.frame_id = 'map'
        self.goal_pose1.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose1.pose.position.x = 10.0
        self.goal_pose1.pose.position.y = -2.0
        self.goal_pose1.pose.position.z = 0.0
        self.goal_pose1.pose.orientation.x = 0.0
        self.goal_pose1.pose.orientation.y = 0.0
        self.goal_pose1.pose.orientation.z = 0.0
        self.goal_pose1.pose.orientation.w = 1.0

        self.goal_pose2 = PoseStamped()
        self.goal_pose2.header.frame_id = 'map'
        self.goal_pose2.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose2.pose.position.x = -6.0
        self.goal_pose2.pose.position.y = 7.0
        self.goal_pose2.pose.position.z = 0.0
        self.goal_pose2.pose.orientation.x = 0.0
        self.goal_pose2.pose.orientation.y = 0.0
        self.goal_pose2.pose.orientation.z = 0.0
        self.goal_pose2.pose.orientation.w = 1.0
        self.path=[self.goal_pose1,self.goal_pose2, self.goal_pose1,self.goal_pose2, self.goal_pose1,self.goal_pose2, self.goal_pose1,self.goal_pose2]
        self.iter=0


        self.subscription = self.create_subscription(Int32,'/goal_reached',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
    def listener_callback(self, msg):
        if msg.data== 1: 
            self.CurrentGoal=self.path[self.iter]
            self.iter+=1
            self.pub()
        # self.get_logger().info('I heard: "%d"' % msg)    

    def pub(self):
        self.publisher_.publish(self.CurrentGoal)



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
