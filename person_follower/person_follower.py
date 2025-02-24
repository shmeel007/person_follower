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
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class PersonFollower(Node):

    def __init__(self):
        super().__init__('person_follower')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        
        # Parameters
        self.safe_distance = 0.5  # Stop if closer than this
        self.follow_distance = 1.0  # Try to maintain this distance
        self.max_linear_speed = 0.3
        self.max_angular_speed = 1.0

    def listener_callback(self, input_msg):
        print("LaserScan received!")  # Debugging step 1

        ranges = np.array(input_msg.ranges)  # Convert to NumPy array
        angles = np.linspace(input_msg.angle_min, input_msg.angle_max, len(ranges))  # Get corresponding angles

        # Ignore invalid readings
        valid_ranges = np.where(np.isfinite(ranges), ranges, np.inf)

        # Find closest object
        min_index = np.argmin(valid_ranges)
        min_distance = valid_ranges[min_index]
        min_angle = angles[min_index]

        print(f"Closest object: Distance = {min_distance:.2f}, Angle = {min_angle:.2f}")  # Debugging step 2

        # Default velocities
        vx = 0.0  # Linear velocity
        wz = 0.0  # Angular velocity

        if min_distance < np.inf:  # Ensure valid detection
            if min_distance > self.follow_distance:
                vx = min(self.max_linear_speed, 0.5 * (min_distance - self.follow_distance))
            elif min_distance < self.safe_distance:
                vx = -0.1  # Move slightly backward if too close
            
            # Turn toward the detected object
            wz = min(self.max_angular_speed, max(-self.max_angular_speed, -2.0 * min_angle))

        print(f"Publishing cmd: vx = {vx:.2f}, wz = {wz:.2f}")  # Debugging step 3

        # Publish movement command
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
