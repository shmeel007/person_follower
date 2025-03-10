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
        self.follow_distance = 1.0  # Ideal following distance
        self.max_linear_speed = 0.3
        self.max_angular_speed = 1.0
        self.is_moving_backward = False  

    def listener_callback(self, input_msg):
        print("LaserScan received!")  

        ranges = np.array(input_msg.ranges)  
        angles = np.linspace(input_msg.angle_min, input_msg.angle_max, len(ranges))  

        # Ignore invalid readings
        valid_ranges = np.where(np.isfinite(ranges), ranges, np.inf)

        # Find closest object
        min_index = np.argmin(valid_ranges)
        min_distance = valid_ranges[min_index]
        min_angle = angles[min_index]

        print(f"Closest object: Distance = {min_distance:.2f}, Angle = {min_angle:.2f}")

        # Default velocities
        vx = 0.0  
        wz = 0.0  

        if min_distance < np.inf:  
            if min_distance > self.follow_distance:
                vx = min(self.max_linear_speed, 0.5 * (min_distance - self.follow_distance))
                self.is_moving_backward = False  
            elif min_distance < self.safe_distance:
                if not self.is_moving_backward:  
                    vx = -0.1  
                    self.is_moving_backward = True
                    print("Too close! Moving backward...")
            else:
                vx = 0.0  
                self.is_moving_backward = False  

            # Smooth turning toward the detected object
            wz = min(self.max_angular_speed, max(-self.max_angular_speed, -2.0 * min_angle))

        # New Fix: Stop after moving backward, then reassess before moving forward
        if self.is_moving_backward:
            print("Moving backward, stopping after 2 seconds...")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2))  
            vx = 0.0  
            wz = 0.0  
            self.is_moving_backward = False  
            print("Stopped. Checking if it's safe to move forward...")

        # After stopping, resume moving forward if the person is at a followable distance
        if min_distance > self.safe_distance and min_distance < self.follow_distance:
            print("Person moved to followable distance, resuming movement!")
            vx = min(self.max_linear_speed, 0.5 * (min_distance - self.follow_distance))

        print(f"Publishing cmd: vx = {vx:.2f}, wz = {wz:.2f}")  

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
