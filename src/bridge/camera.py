import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from detection.detection import Detect as detect 

class RosCameraSubscriber(Node):
    def __init__(self):
        super().__init__('vtalon_camera_processor')
        
        self.subscription_vtalon = self.create_subscription(
            Image,
            '/vtalon/camera/image_raw',
            self._image_callback,
            qos_profile_sensor_data
        )
        self.subscription_iris = self.create_subscription(
            Image,
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image',
            self._image_callback,
            qos_profile_sensor_data
        )
        
        self.bridge = CvBridge()
        self.get_logger().info('node active ...')

    def _image_callback(self, msg):
        self.get_logger().info('Frame received...')
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            cv2.imshow("Camera", frame_bgr)
            
            detectFrame = detect(frame_bgr, False)  

            redMask = detectFrame.maskRed(frame_bgr)
            blueMask = detectFrame.maskBlue(frame_bgr) 
            redFrame = detectFrame.detect(redMask, frame_bgr)
            blueFrame = detectFrame.detect(blueMask, frame_bgr)

            cv2.imshow("red detection", redFrame) 
            cv2.imshow("blue detection", blueFrame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                rclpy.shutdown()
                exit(0)
                
        except Exception as e:
            self.get_logger().error(f"Failed to decode image frame array: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RosCameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n Closing window stream context.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
