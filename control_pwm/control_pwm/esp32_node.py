# Filename: serial_reader_node.py
# Location: ~/ros2_ws/src/esp32_serial_bridge/esp32_serial_bridge/serial_reader_node.py

import rclpy
from rclpy.node import Node
import serial
import time
import sys

# custom message type
from joystick_msgs.msg import JoystickData

class ESP32SerialReader(Node):
    """
    ROS2 Node to read serial data from ESP32 and publish it as JoystickData messages.
    This node acts as the bridge between the ESP32 and other ROS2 nodes (like your control_node).
    """
    def __init__(self):
        super().__init__('esp32_serial_reader') # Initialize the ROS2 node with a name

        self.declare_parameter('serial_port', '/dev/ttyS0') 
        self.declare_parameter('baud_rate', 115200)         # Baud rate matching ESP32's Serial2

        # Retrieve the parameter values
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Create a publisher for the custom JoystickData message.
        self.publisher_ = self.create_publisher(JoystickData, 'joystick_data', 10)

        self.serial_connection = None
        self.connect_serial() # Attempt to establish the serial connection upon node startup

        # Create a timer that will call 'read_serial_callback' periodically (every 10ms).
        # This ensures continuous reading of the serial port.
        self.timer = self.create_timer(0.01, self.read_serial_callback)

        self.get_logger().info(f'ESP32 Serial Reader Node initialized. Connecting to {self.serial_port} @ {self.baud_rate} baud.')

    def connect_serial(self):
        """
        Establishes or re-establishes the serial connection to the ESP32.
        Includes basic error handling and retry mechanism.
        """
        # Close any existing connection before attempting a new one
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            self.get_logger().info("Closed existing serial connection.")

        try:
            # Open the serial port with a timeout
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            # Clear any old data in the serial buffers
            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()
            self.get_logger().info(f"Successfully connected to serial port {self.serial_port}")
        except serial.SerialException as e:
            # Log an error if connection fails and schedule a retry
            self.get_logger().error(f"Failed to connect to serial port {self.serial_port}: {e}")
            self.serial_connection = None
            self.get_logger().warn("Retrying serial connection in 5 seconds...")
            self.create_timer(5.0, self.connect_serial) # Schedule a retry
        except Exception as e:
            # Catch any other unexpected errors during connection attempt
            self.get_logger().error(f"An unexpected error occurred during serial connection: {e}")
            self.serial_connection = None
            self.get_logger().warn("Retrying serial connection in 5 seconds...")
            self.create_timer(5.0, self.connect_serial)

    def read_serial_callback(self):
        """
        Callback function executed by the timer.
        Reads a line from the serial port, parses the data,
        and publishes it as a JoystickData message.
        """
        # Only proceed if the serial connection is active
        if self.serial_connection and self.serial_connection.is_open:
            try:
                # Check if there's data available to read in the input buffer
                if self.serial_connection.in_waiting > 0:
                    # Read a line, decode it from bytes to UTF-8 string, and remove whitespace
                    line = self.serial_connection.readline().decode('utf-8').strip()

                    if line: # Process only if the line is not empty
                        try:
                            # Split the string by commas and convert each part to a float
                            values = [float(val) for val in line.split(',')]

                            # Ensure we received exactly 6 values (x, y, z, yaw, pitch, roll)
                            if len(values) == 6:
                                # Create a new instance of the custom JoystickData message
                                msg = JoystickData()
                                # Populate the message fields with the parsed float values
                                msg.x = values[0]
                                msg.y = values[1]
                                msg.z = values[2]
                                msg.yaw = values[3]
                                msg.pitch = values[4]
                                msg.roll = values[5]

                                # Publish the populated message to the 'joystick_data' topic
                                self.publisher_.publish(msg)
                                # self.get_logger().info(f'Published: X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}') # Uncomment for verbose logging
                            else:
                                self.get_logger().warn(f"Received malformed data (expected 6 values): {line}")
                        except ValueError:
                            # Handle cases where string parts cannot be converted to float
                            self.get_logger().warn(f"Could not convert data to float, skipping: {line}")
                        except IndexError:
                            # Handle cases where the split string doesn't have enough elements
                            self.get_logger().warn(f"Insufficient data fields received, skipping: {line}")
                        except Exception as parse_e:
                            # Catch any other unexpected errors during data parsing
                            self.get_logger().error(f"Error during data parsing: {parse_e} for line: {line}")

            except serial.SerialException as e:
                # Handle serial communication errors (e.g., device unplugged)
                self.get_logger().error(f"Serial communication error: {e}. Attempting to reconnect...")
                if self.serial_connection.is_open:
                    self.serial_connection.close()
                self.connect_serial() # Trigger a reconnection attempt
            except Exception as e:
                # Catch any other general exceptions during the callback execution
                self.get_logger().error(f"An unexpected error occurred in serial callback: {e}")

        # If serial_connection is not established, the connect_serial timer will handle retries.
        elif not self.serial_connection:
            pass # No action needed here, connection logic is in connect_serial()

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args) # Initialize the ROS2 client library
    node = ESP32SerialReader() # Create an instance of the node

    try:
        rclpy.spin(node) # Keep the node alive and process callbacks
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user (KeyboardInterrupt).")
    except Exception as e:
        node.get_logger().fatal(f"Unhandled exception in main: {e}")
        sys.exit(1) # Exit with an error code on unhandled exception
    finally:
        # Clean up resources before exiting
        if node.serial_connection and node.serial_connection.is_open:
            node.serial_connection.close() # Close the serial port
            node.get_logger().info("Serial port closed.")
        node.destroy_node() # Destroy the ROS2 node
        rclpy.shutdown() # Shut down the ROS2 client library

if __name__ == '__main__':
    main()
