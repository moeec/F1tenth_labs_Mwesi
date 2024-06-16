import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.parameter import Parameter

class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # Declare and initialize parameters
        self.declare_parameter('v', 0.0)  # speed parameter
        self.declare_parameter('d', 0.0)  # steering angle parameter

        # Create a publisher for AckermannDriveStamped
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        # Create a timer to publish messages as fast as possible
        self.timer = self.create_timer(0.01, self.publish_drive)  # 100 Hz

    def publish_drive(self):
        # Get parameters
        v = self.get_parameter('v').get_parameter_value().double_value
        d = self.get_parameter('d').get_parameter_value().double_value

        # Create and populate AckermannDriveStamped message
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = v
        drive_msg.drive.steering_angle = d

        # Publish the message
        self.publisher_.publish(drive_msg)
        self.get_logger().info(f'Publishing: speed={v}, steering_angle={d}')

def main(args=None):
    rclpy.init(args=args)
    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
