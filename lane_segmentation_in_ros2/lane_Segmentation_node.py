import sys
import os

# Add conda environment site-packages to Python path
conda_prefix = os.environ.get('CONDA_PREFIX')
if conda_prefix:
    site_packages = os.path.join(conda_prefix, 'lib', 'python3.10', 'site-packages')
    sys.path.insert(0, site_packages)

import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
from ultralytics import YOLO

class LaneSegmenatationNode(Node):
    def __init__(self):
        super().__init__('lane_segmentation_node')
        self.bridge = CvBridge()
        self.model = YOLO('/home/biswash/Documents/yolo_biswash/lane_segmentation/best.pt')
        # Updated topic names
        self.input_topic = 'camera_raw'
        self.output_topic = 'lane_segmented'
        
        self.subscription = self.create_subscription(
            Image, self.input_topic, self.image_callback, 10)
        self.publisher = self.create_publisher(Image, self.output_topic, 10)
        self.get_logger().info('Lane Segmenatation Node initialized')

    def image_callback(self, msg):
        try:
            # Convert ROS2 image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Perform face detection
            results = self.model.predict(frame, imgsz=640, conf=0.1)
            
            # Get annotated frame
            annotated_frame = results[0].plot()
            
            # Convert back to ROS2 message and publish
            ros2_image_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.publisher.publish(ros2_image_msg)
            
            self.get_logger().debug('Lane segmenatation processed successfully')
            
        except Exception as e:
            self.get_logger().error(f'Error in image processing: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneSegmenatationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
