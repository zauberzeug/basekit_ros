import cv2
import numpy as np
import requests
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager
from image_transport import ImageTransport


class MjpegCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('mjpeg_camera')
        
        # Declare and get parameters
        self.declare_parameter('camera_url', 'http://localhost:8080/video')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('camera_info_url', '')
        
        self.camera_url = self.get_parameter('camera_url').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        camera_name = self.get_parameter('camera_name').value
        camera_info_url = self.get_parameter('camera_info_url').value
        
        # Initialize camera info manager
        self.camera_info_manager = CameraInfoManager(
            self, 
            camera_name,
            camera_info_url
        )
        self.camera_info_manager.loadCameraInfo()
        
        # Create publishers
        self.image_transport = ImageTransport(self)
        self.image_publisher = self.image_transport.advertiseCamera('image_raw', 10)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create timer for publishing images
        self.timer = self.create_timer(1.0/publish_rate, self.publish_image)
        
        self.get_logger().info(f'MjpegCameraNode initialized with URL: {self.camera_url}')

    def publish_image(self) -> None:
        try:
            # Get the MJPEG stream
            response = requests.get(self.camera_url, stream=True)
            if response.status_code == 200:
                # Convert the stream to numpy array
                image_array = np.asarray(bytearray(response.content), dtype=np.uint8)
                frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                
                # Convert to ROS message
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.frame_id = self.frame_id
                msg.header.stamp = self.get_clock().now().to_msg()
                
                # Get camera info
                info = self.camera_info_manager.getCameraInfo()
                info.header = msg.header
                
                # Publish both image and camera info
                self.image_publisher.publish(msg, info)
            else:
                self.get_logger().warn(f'Failed to get image from camera. Status code: {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MjpegCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 