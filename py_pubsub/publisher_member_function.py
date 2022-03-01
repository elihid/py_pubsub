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
        self.goal_pose1.pose.position.x = -10.0
        self.goal_pose1.pose.position.y = 10.0
        self.goal_pose1.pose.position.z = 0.0
        self.goal_pose1.pose.orientation.x = 0.0
        self.goal_pose1.pose.orientation.y = 0.0
        self.goal_pose1.pose.orientation.z = 0.0
        self.goal_pose1.pose.orientation.w = 1.0

        self.goal_pose2 = PoseStamped()
        self.goal_pose2.header.frame_id = 'map'
        self.goal_pose2.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose2.pose.position.x = 0.0
        self.goal_pose2.pose.position.y = 10.0
        self.goal_pose2.pose.position.z = 0.0
        self.goal_pose2.pose.orientation.x = 0.0
        self.goal_pose2.pose.orientation.y = 0.0
        self.goal_pose2.pose.orientation.z = 0.0
        self.goal_pose2.pose.orientation.w = 1.0
        
        self.goal_pose3 = PoseStamped()
        self.goal_pose3.header.frame_id = 'map'
        self.goal_pose3.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose3.pose.position.x = 10.0
        self.goal_pose3.pose.position.y = 10.0
        self.goal_pose3.pose.position.z = 0.0
        self.goal_pose3.pose.orientation.x = 0.0
        self.goal_pose3.pose.orientation.y = 0.0
        self.goal_pose3.pose.orientation.z = 0.0
        self.goal_pose3.pose.orientation.w = 1.0
        
        self.goal_pose4 = PoseStamped()
        self.goal_pose4.header.frame_id = 'map'
        self.goal_pose4.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose4.pose.position.x = 10.0
        self.goal_pose4.pose.position.y = 8.0
        self.goal_pose4.pose.position.z = 0.0
        self.goal_pose4.pose.orientation.x = 0.0
        self.goal_pose4.pose.orientation.y = 0.0
        self.goal_pose4.pose.orientation.z = 0.0
        self.goal_pose4.pose.orientation.w = 1.0
        
        self.goal_pose5 = PoseStamped()
        self.goal_pose5.header.frame_id = 'map'
        self.goal_pose5.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose5.pose.position.x = 0.0
        self.goal_pose5.pose.position.y = 8.0
        self.goal_pose5.pose.position.z = 0.0
        self.goal_pose5.pose.orientation.x = 0.0
        self.goal_pose5.pose.orientation.y = 0.0
        self.goal_pose5.pose.orientation.z = 0.0
        self.goal_pose5.pose.orientation.w = 1.0
        
        self.goal_pose6 = PoseStamped()
        self.goal_pose6.header.frame_id = 'map'
        self.goal_pose6.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose6.pose.position.x = -10.0
        self.goal_pose6.pose.position.y = 8.0
        self.goal_pose6.pose.position.z = 0.0
        self.goal_pose6.pose.orientation.x = 0.0
        self.goal_pose6.pose.orientation.y = 0.0
        self.goal_pose6.pose.orientation.z = 0.0
        self.goal_pose6.pose.orientation.w = 1.0
        
        self.goal_pose7 = PoseStamped()
        self.goal_pose7.header.frame_id = 'map'
        self.goal_pose7.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose7.pose.position.x = -10.0
        self.goal_pose7.pose.position.y = 6.0
        self.goal_pose7.pose.position.z = 0.0
        self.goal_pose7.pose.orientation.x = 0.0
        self.goal_pose7.pose.orientation.y = 0.0
        self.goal_pose7.pose.orientation.z = 0.0
        self.goal_pose7.pose.orientation.w = 1.0
        
        self.goal_pose8 = PoseStamped()
        self.goal_pose8.header.frame_id = 'map'
        self.goal_pose8.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose8.pose.position.x = 0.0
        self.goal_pose8.pose.position.y = 6.0
        self.goal_pose8.pose.position.z = 0.0
        self.goal_pose8.pose.orientation.x = 0.0
        self.goal_pose8.pose.orientation.y = 0.0
        self.goal_pose8.pose.orientation.z = 0.0
        self.goal_pose8.pose.orientation.w = 1.0
       
        self.goal_pose9 = PoseStamped()
        self.goal_pose9.header.frame_id = 'map'
        self.goal_pose9.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose9.pose.position.x = 10.0
        self.goal_pose9.pose.position.y = 6.0
        self.goal_pose9.pose.position.z = 0.0
        self.goal_pose9.pose.orientation.x = 0.0
        self.goal_pose9.pose.orientation.y = 0.0
        self.goal_pose9.pose.orientation.z = 0.0
        self.goal_pose9.pose.orientation.w = 1.0
        
        self.goal_pose10 = PoseStamped()
        self.goal_pose10header.frame_id = 'map'
        self.goal_pose10header.stamp = self.get_clock().now().to_msg()
        self.goal_pose10.pose.position.x = 10.0
        self.goal_pose10.pose.position.y = 4.0
        self.goal_pose10.pose.position.z = 0.0
        self.goal_pose10.pose.orientation.x = 0.0
        self.goal_pose10.pose.orientation.y = 0.0
        self.goal_pose10.pose.orientation.z = 0.0
        self.goal_pose10.pose.orientation.w = 1.0
        
        self.goal_pose11 = PoseStamped()
        self.goal_pose11.header.frame_id = 'map'
        self.goal_pose11.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose11.pose.position.x = 0.0
        self.goal_pose11.pose.position.y = 4.0
        self.goal_pose11.pose.position.z = 0.0
        self.goal_pose11.pose.orientation.x = 0.0
        self.goal_pose11.pose.orientation.y = 0.0
        self.goal_pose11.pose.orientation.z = 0.0
        self.goal_pose11.pose.orientation.w = 1.0
        
        self.goal_pose12 = PoseStamped()
        self.goal_pose12.header.frame_id = 'map'
        self.goal_pose12.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose12.pose.position.x = -10.0
        self.goal_pose12.pose.position.y = 4.0
        self.goal_pose12.pose.position.z = 0.0
        self.goal_pose12.pose.orientation.x = 0.0
        self.goal_pose12.pose.orientation.y = 0.0
        self.goal_pose12.pose.orientation.z = 0.0
        self.goal_pose12.pose.orientation.w = 1.0
        
        self.goal_pose13 = PoseStamped()
        self.goal_pose13.header.frame_id = 'map'
        self.goal_pose13.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose13.pose.position.x = -10.0
        self.goal_pose13.pose.position.y = 2.0
        self.goal_pose13.pose.position.z = 0.0
        self.goal_pose13.pose.orientation.x = 0.0
        self.goal_pose13.pose.orientation.y = 0.0
        self.goal_pose13.pose.orientation.z = 0.0
        self.goal_pose13.pose.orientation.w = 1.0
        
        self.goal_pose14 = PoseStamped()
        self.goal_pose14.header.frame_id = 'map'
        self.goal_pose14.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose14.pose.position.x = 0.0
        self.goal_pose14.pose.position.y = 2.0
        self.goal_pose14.pose.position.z = 0.0
        self.goal_pose14.pose.orientation.x = 0.0
        self.goal_pose14.pose.orientation.y = 0.0
        self.goal_pose14.pose.orientation.z = 0.0
        self.goal_pose14.pose.orientation.w = 1.0
        
        self.goal_pose15 = PoseStamped()
        self.goal_pose15.header.frame_id = 'map'
        self.goal_pose15.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose15.pose.position.x = 10.0
        self.goal_pose15.pose.position.y = 2.0
        self.goal_pose15.pose.position.z = 0.0
        self.goal_pose15.pose.orientation.x = 0.0
        self.goal_pose15.pose.orientation.y = 0.0
        self.goal_pose15.pose.orientation.z = 0.0
        self.goal_pose15.pose.orientation.w = 1.0
        
        self.goal_pose16 = PoseStamped()
        self.goal_pose16.header.frame_id = 'map'
        self.goal_pose16.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose16.pose.position.x = 10.0
        self.goal_pose16.pose.position.y = 0.0
        self.goal_pose16.pose.position.z = 0.0
        self.goal_pose16.pose.orientation.x = 0.0
        self.goal_pose16.pose.orientation.y = 0.0
        self.goal_pose16.pose.orientation.z = 0.0
        self.goal_pose16.pose.orientation.w = 1.0
        
        self.goal_pose17 = PoseStamped()
        self.goal_pose17.header.frame_id = 'map'
        self.goal_pose17.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose17.pose.position.x = 0.0
        self.goal_pose17.pose.position.y = 0.0
        self.goal_pose17.pose.position.z = 0.0
        self.goal_pose17.pose.orientation.x = 0.0
        self.goal_pose17.pose.orientation.y = 0.0
        self.goal_pose17.pose.orientation.z = 0.0
        self.goal_pose17.pose.orientation.w = 1.0
        
        self.path=[self.goal_pose1,self.goal_pose2, self.goal_pose3,self.goal_pose4, self.goal_pose5,self.goal_pose6, self.goal_pose7,self.goal_pose8, self.goal_pose9, self.goal_pose10, self.goal_pose11, self.goal_pose12, self.goal_pose13, self.goal_pose14, self.goal_pose15, self.goal_pose16, self.goal_pose17, self.goal_pose18, self.goal_pose19, self.goal_pose20]
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
