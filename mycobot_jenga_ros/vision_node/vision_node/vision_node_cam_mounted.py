import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Quaternion
from mycobot_interfaces.msg import DetectedBlock, DetectedBlockArray
from cv_bridge import CvBridge
import numpy as np
import ultralytics
import cv2
import tf2_ros
from tf_transformations import quaternion_from_euler

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Load YOLOv8 model (trained on Jenga blocks)
        self.model = ultralytics.YOLO('/home/jesse/Documents/CPTS483-Robotics/Project/Yolo-Block_Detection/runs/detect/train/weights/best.pt')

        self.bridge = CvBridge()

        # Subscribers to RealSense topics
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.info_callback, 10)

        self.color_image = None
        self.depth_image = None
        self.camera_intrinsics = None

        # Publisher for detected blocks
        self.pub = self.create_publisher(DetectedBlockArray, '/detected_blocks', 10)

        # TF buffer for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1/30, self.publish_detections)
        
        # DEBUG: Log that the node is fully up
        self.get_logger().info("VisionNode running")

    def color_callback(self, msg):
        self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')

    def info_callback(self, msg):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = msg

    def publish_detections(self):
        if self.color_image is None or self.depth_image is None or self.camera_intrinsics is None:
            return

        results = self.model.predict(self.color_image, imgsz=640, conf=0.5)[0]
        detected_blocks = []

        for idx, box in enumerate(results.boxes.xyxy.cpu().numpy()):
            x1, y1, x2, y2 = map(int, box)
            confidence = results.boxes.conf[idx].item()

            # Center of bounding box
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            # Get average depth in bbox region
            depth_region = self.depth_image[y1:y2, x1:x2].astype(float)
            valid_depth = depth_region[depth_region > 0]

            if valid_depth.size == 0:
                continue  # skip if no valid depth

            z = np.median(valid_depth) / 1000.0  # mm to meters

            # Camera intrinsics
            fx = self.camera_intrinsics.k[0]
            fy = self.camera_intrinsics.k[4]
            cx = self.camera_intrinsics.k[2]
            cy = self.camera_intrinsics.k[5]

            # 3D coordinates in camera frame
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            point_camera = np.array([x, y, z, 1.0])

            # Transform from camera to base_link frame
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    self.camera_intrinsics.header.frame_id,
                    rclpy.time.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                quat = [rotation.x, rotation.y, rotation.z, rotation.w]

                transform_mat = tf2_ros.transformations.quaternion_matrix(quat)
                transform_mat[0:3, 3] = [translation.x, translation.y, translation.z]

                point_base = transform_mat @ point_camera
            except Exception as e:
                self.get_logger().warn(f'TF error: {e}')
                continue

            # Assume upright orientation (no rotation), refine later if needed
            orientation = quaternion_from_euler(0, 0, 0)

            block = DetectedBlock()
            block.id = idx
            block.confidence = confidence
            block.position = Point(x=point_base[0], y=point_base[1], z=point_base[2])
            block.orientation = Quaternion(
                x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

            detected_blocks.append(block)
            
            # DEBUG: print per detection
            print(f"[VisionNode] Detection ID {block.id}: conf={block.confidence:.2f}, "
      f"pos=({block.position.x:.2f}, {block.position.y:.2f},             {block.position.z:.2f})")


        # Publish detected blocks
        msg = DetectedBlockArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.blocks = detected_blocks

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

