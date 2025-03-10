# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.desired_distance = 1.0  # Desired distance to the person (meters)
        self.linear_speed = 0.5       # Maximum linear speed (m/s)
        self.angular_speed = 1.0      # Maximum angular speed (rad/s)

    def listener_callback(self, input_msg):
        angle_min = input_msg.angle_min
        angle_max = input_msg.angle_max
        angle_increment = input_msg.angle_increment
        ranges = input_msg.ranges

        # Find the closest point (assuming it's the person)
        min_range = float('inf')
        min_index = -1

        for i, r in enumerate(ranges):
            if r < min_range and r > 0.1: #Ignore very small ranges and infinite ranges
                min_range = r
                min_index = i

        vx = 0.0
        wz = 0.0

        if min_index != -1:
            # Calculate the angle to the closest point
            angle = angle_min + min_index * angle_increment

            # Calculate the error in distance
            distance_error = min_range - self.desired_distance

            # Calculate linear velocity (move towards or away from the person)
            vx = self.linear_speed * (-distance_error)
            vx = max(min(vx, self.linear_speed), -self.linear_speed) #Limit the linear velocity

            # Calculate angular velocity (turn towards the person)
            wz = self.angular_speed * (-angle)
            wz = max(min(wz, self.angular_speed), -self.angular_speed) #Limit the angular velocity

        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = wz
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
