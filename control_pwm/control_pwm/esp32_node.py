import rclpy
from rclpy.node import Node
import serial
from joystick_msgs.msg import JoystickData

class ESP32SerialReader(Node):
    def __init__(self):
        super().__init__('esp32_serial_reader')

        # --- Parameters ---
        # self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # --- Publisher ---
        self.publisher_ = self.create_publisher(JoystickData, 'joystick_data', 10)

        # --- Serial Connection (Simple and Direct) ---
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"✅ Successfully connected to {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed to connect to serial port: {e}")
            rclpy.shutdown() # Exit if connection fails
            return

        # --- Timer for reading data ---
        self.timer = self.create_timer(0.01, self.read_and_publish)

    def read_and_publish(self):
        try:
            # Read one line from the serial port
            line = self.serial_connection.readline().decode('utf-8').strip()

            # If a line was received, process it
            if line:
                # This is the most important line for debugging
                self.get_logger().info(f"RAW DATA IN: '{line}'") 
                
                # Attempt to parse the data
                values = [float(val) for val in line.split(',')]

                if len(values) == 6:
                    msg = JoystickData()
                    msg.x, msg.y, msg.z, msg.yaw, msg.pitch, msg.roll = values
                    self.publisher_.publish(msg)
                else:
                    self.get_logger().warn(f"⚠️ Malformed (expected 6 values): {line}")

        except ValueError:
            self.get_logger().warn(f"⚠️ Non-numeric data received: {line}")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

    def destroy_node(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESP32SerialReader()
    if rclpy.ok():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()