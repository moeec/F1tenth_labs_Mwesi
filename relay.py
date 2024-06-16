import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):
    def __init__(self):
        super().__init__('relay')

        # Create a subscriber to the 'drive' topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.drive_callback,
            10
        )

        # Create a publisher for the 'drive_relay' topic
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def drive_callback(self, msg):
        # Extract speed and steering angle, multiply them by 3
        modified_speed = msg.drive.speed * 3
        modified_steering_angle = msg.drive.steering_angle * 3

        # Create a new AckermannDriveStamped message
        relay_msg = AckermannDriveStamped()
        relay_msg.drive.speed = modified_speed
        relay_msg.drive.steering_angle = modified_steering_angle

        # Publish the modified message to the 'drive_relay' topic
        self.publisher_.publish(relay_msg)
        self.get_logger().info(f'Relaying: speed={modified_speed}, steering_angle={modified_steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = Relay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
