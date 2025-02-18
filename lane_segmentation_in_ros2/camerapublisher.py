import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class PublisherNodeClass(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.cameraDeviceNumber = 0
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'camera_raw'
        self.queueSize = 20
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        self.periodComunication = 0.02
        self.timer = self.create_timer(self.periodComunication, self.timer_callbackFunction)
        self.i = 0

    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)
        
        if success:
            ros2_image_message = self.bridgeObject.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(ros2_image_message)
            self.get_logger().info('Publishing image number %d' % self.i)
            self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisherObject = PublisherNodeClass()
    rclpy.spin(publisherObject)
    publisherObject.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# import cv2
# import rclpy
# from sensor_msgs.msg import Image
# from rclpy.node import Node
# from cv_bridge import CvBridge

# class PublisherNodeClass(Node):
#     def __init__(self, video_path=None):
#         super().__init__('publisher_node')
        
#         # Use video file if provided, else use webcam
#         if video_path:
#             self.get_logger().info(f"Using video file: {video_path}")
#             self.camera = cv2.VideoCapture(video_path)
#         else:
#             self.get_logger().info("Using webcam")
#             self.camera = cv2.VideoCapture(0)

#         self.bridgeObject = CvBridge()
#         self.topicNameFrames = 'camera_raw'
#         self.queueSize = 20
#         self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
#         self.periodComunication = 0.02  # 50 FPS
#         self.timer = self.create_timer(self.periodComunication, self.timer_callbackFunction)
#         self.i = 0

#     def timer_callbackFunction(self):
#         success, frame = self.camera.read()

#         if not success:
#             self.get_logger().warn("Frame capture failed, restarting video stream...")
#             self.camera.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart video if it ends
#             return

#         frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_CUBIC)
#         ros2_image_message = self.bridgeObject.cv2_to_imgmsg(frame, encoding="bgr8")
#         self.publisher.publish(ros2_image_message)
#         self.get_logger().info(f'Publishing image number {self.i}')
#         self.i += 1

# def main(args=None):
#     rclpy.init(args=args)
    
#     # Set the path to the video file, or leave as None for the webcam
#     video_path = 'lane_basketball.mp4'  # Change to None for webcam
#     publisherObject = PublisherNodeClass(video_path)

#     rclpy.spin(publisherObject)
#     publisherObject.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
