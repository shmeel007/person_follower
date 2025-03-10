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
        self.safe_distance = 0.6  # Stop if closer than this
        self.follow_distance = 1.2  # Ideal following distance
        self.max_linear_speed = 0.2
        self.max_angular_speed = 0.8
        self.prev_wz = 0.0  # Store previous angular speed for smoothing
        self.prev_vx = 0.0  # Store previous linear speed for gradual acceleration

    def listener_callback(self, input_msg):
        print("LaserScan received!")  

        ranges = np.array(input_msg.ranges)  
        angles = np.linspace(input_msg.angle_min, input_msg.angle_max, len(ranges))  

        # Ignore invalid readings and apply basic filtering
        valid_ranges = np.where(np.isfinite(ranges), ranges, np.inf)
        valid_ranges = self.moving_average_filter(valid_ranges, window_size=5)

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
                vx = min(self.max_linear_speed, 0.4 * (min_distance - self.follow_distance))  # Smoother acceleration
            elif min_distance < self.safe_distance:
                vx = -0.05  # Slow backward motion to avoid overshooting
            else:
                vx = 0.0  # Stop at the correct distance

            # Smooth turning using proportional control
            wz = -1.5 * min_angle  # Turn speed proportional to angle
            wz = max(-self.max_angular_speed, min(self.max_angular_speed, wz))  # Limit max turning speed

        # Apply smoothing to prevent oscillations
        vx = self.smooth_velocity(vx, self.prev_vx, alpha=0.3)
        wz = self.smooth_velocity(wz, self.prev_wz, alpha=0.4)

        self.prev_vx = vx  # Store previous values for next iteration
        self.prev_wz = wz

        print(f"Publishing cmd: vx = {vx:.2f}, wz = {wz:.2f}")  

        # Publish movement command
        output_msg = Twist()
        output_msg.linear.x = vx
        output_msg.angular.z = wz
        self.publisher_.publish(output_msg)

    def moving_average_filter(self, data, window_size=3):
        """Smooths noisy LiDAR data using a moving average filter."""
        return np.convolve(data, np.ones(window_size)/window_size, mode='same')

    def smooth_velocity(self, new_value, prev_value, alpha=0.3):
        """Applies an exponential moving average filter to smooth motion."""
        return alpha * new_value + (1 - alpha) * prev_value

def main(args=None):
    rclpy.init(args=args)
    person_follower = PersonFollower()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
