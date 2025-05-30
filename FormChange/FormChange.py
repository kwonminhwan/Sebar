import rclpy
from rclpy.node import Node
from yolo_msgs.msg import Yolodetect
import serial

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(
            Yolodetect,
            'yolo_info',
            self.yolo_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Set up serial communication with Arduino
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust port and baud rate as necessary
        
        self.get_logger().info('Motor Control Node Initialized')

    def yolo_callback(self, msg):
        if msg.name == "Scissor":
            self.extend_motor()
            self.get_logger().info('Scissor detected: Extending motor')
        elif msg.name == "Paper":
            self.retract_motor()
            self.get_logger().info('Paper detected: Retracting motor')
        else:
            self.stop_motor()
            self.get_logger().info(f'Unknown object ({msg.name}) detected: Stopping motor')

    def extend_motor(self):
        self.ser.write(b'E')  # Send 'E' command to Arduino

    def retract_motor(self):
        self.ser.write(b'R')  # Send 'R' command to Arduino

    def stop_motor(self):
        self.ser.write(b'S')  # Send 'S' command to Arduino

def main(args=None):
    rclpy.init(args=args)
    motor_control = MotorControlNode()
    rclpy.spin(motor_control)
    motor_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

