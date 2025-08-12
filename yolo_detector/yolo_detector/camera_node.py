import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.get_logger().info('YOLO Detector Node has been started.')

        # IMPORTANT: Change this to the actual path of your model
        model_path = './model/best.pt'
        
        # Load the YOLOv8 model
        self.model = YOLO(model_path)
        self.get_logger().info('Successfully loaded YOLOv8 model.')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create a subscriber to the camera image topic
        # IMPORTANT: Change '/camera/image_raw' to your actual camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.image_callback,
            10)
        
        # Create a publisher for the annotated image
        self.publisher = self.create_publisher(Image, '/yolo/image_annotated', 10)

    def image_callback(self, msg):
        """
        This function is called every time a new image is received.
        """
        try:
            # Convert the ROS Image message to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Run YOLOv8 inference on the frame
        results = self.model(cv_image)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Convert the annotated OpenCV image back to a ROS Image message
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8')
        
        # Publish the annotated image
        self.publisher.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    yolo_detector_node = YoloDetectorNode()
    rclpy.spin(yolo_detector_node)
    
    # Destroy the node explicitly
    yolo_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()