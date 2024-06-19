import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.parameter import Parameter



class talker(Node):

    def __init__(self):
        super().__init__("talker")
        
        # Declare and initialize parameters
        self.declare_parameter('v', 0.0)  # speed parameter
        self.declare_parameter('d', 0.0)  # steering angle parameter
        
        # Create a publisher for AckermannDriveStamped
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        
        self.counter_ = 0
        self.frequency_ = 100
        self.get_logger().info("Publishing Drive at %d Hz" % self.frequency_)
        
        self.timer_ = self.create_timer(self.frequency_, self.publish_drive)

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
    node = talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
