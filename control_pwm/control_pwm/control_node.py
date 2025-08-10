import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Twist
from joystick_msgs.msg import JoystickData
import serial
import numpy as np

class ROVPWMController(Node):
    def __init__(self):
        super().__init__('rov_pwm_controller')

        # --- Parameters ---
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        port = self.get_parameter('arduino_port').get_parameter_value().string_value
        self.arduino = serial.Serial(port, baudrate=115200, timeout=0.1)


        # --- Thruster Mixing Matrix (6x6) ---
        self.mixing_matrix = np.array([
            [1,  0,  1,  1, 0,  0],  # T1
            [1,  0, -1, -1, 0,  0],  # T2
            [1,  0, -1,  1, 0,  0],  # T3
            [1,  0,  1, -1, 0,  0],  # T4
            [0,  1,  0,  0, 0,  1],  # T5
            [0,  1,  0,  0, 0, -1],  # T6
        ])

        # --- Init ---
        self.thrust_input =  np.zeros(6)

        # --- Subscribers ---
        # self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)
        self.create_subscription(
            JoystickData,       # The message type it expects
            'joystick_data',    # The topic name to subscribe to
            self.joystick_data_callback, # The callback function
            10                  # QoS profile (matching publisher's)
        )

        # --- Timer to send to Arduino ---
        self.create_timer(0.05, self.send_pwm)  # 20 Hz

        self.get_logger().info("ROV PWM Controller Initialized")

    def joystick_data_callback(self, msg):
        self.thrust_input = np.array([
            msg.x,
            msg.y,
            msg.z,
            msg.yaw,  # yaw
            msg.pitch,  # pitch
            msg.roll   # roll
        ])


    def send_pwm(self):

        # --- Thruster Mixing ---
        thrust_output = self.mixing_matrix @ self.thrust_input
        thrust_output = np.clip(thrust_output, -1.0, 1.0)
        thrust_pwm = [int(1500 + 400 * t) for t in thrust_output]


        # --- Combine and Send to Arduino ---
        packet = ','.join(str(p) for p in thrust_pwm) + '\n'
        try:
            self.arduino.write(packet.encode('utf-8'))
            self.get_logger().info(f"Sent PWM: {packet.encode('utf-8')}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")
            return

        # Optional debug log

def main(args=None):
    rclpy.init(args=args)
    node = ROVPWMController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()