import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthImageSubscriber(Node):
    def __init__(self):
        super().__init__('depth_image_subscriber')
        self.bridge = CvBridge()
        
        # Subscribe to depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # Subscribe to camera info for intrinsics
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/depth/camera_info',
            self.info_callback,
            10
        )
        
        self.get_logger().info('Subscribed to /depth/image_raw and /depth/camera_info')
        
        # Store camera intrinsics
        self.fx = None  # Focal length x
        self.fy = None  # Focal length y
        self.cx = None  # Principal point x
        self.cy = None  # Principal point y
        
        # Occupancy grid (simple 2D map)
        self.grid_size = 100  # 10m x 10m at 10cm resolution
        self.resolution = 0.1  # 10cm per cell
        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.uint8)

    def info_callback(self, msg):
        """Store camera intrinsics from /depth/camera_info"""
        if self.fx is None:  # Only set once
            self.fx = msg.k[0]  # fx
            self.fy = msg.k[4]  # fy
            self.cx = msg.k[2]  # cx
            self.cy = msg.k[5]  # cy
            self.get_logger().info('Camera intrinsics received')

    def depth_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV (uint16 or float32, typically in mm)
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Normalize for visualization (optional)
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            cv2.imshow('Depth Image', depth_normalized)
            cv2.waitKey(1)
            
            # Process depth data for mapping
            if self.fx is not None:  # Wait for intrinsics
                self.process_depth_for_mapping(depth_image)
            else:
                self.get_logger().warn('Waiting for camera intrinsics...')
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

    def process_depth_for_mapping(self, depth_image):
        """Convert depth image to 2D occupancy grid"""
        # Get image dimensions
        height, width = depth_image.shape
        
        # Downsample for performance (optional)
        step = 4  # Process every 4th pixel
        u, v = np.meshgrid(np.arange(0, width, step), np.arange(0, height, step))
        depth_values = depth_image[v, u] / 1000.0  # Convert mm to meters
        
        # Convert to 3D points using intrinsics
        z = depth_values  # Depth in meters
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        # Filter valid points (remove zeros or very far points)
        valid = (z > 0) & (z < 10.0)  # Max 10m range
        x, y, z = x[valid], y[valid], z[valid]
        
        # Project to 2D grid (ignore z for simplicity)
        grid_x = (x / self.resolution + self.grid_size / 2).astype(int)
        grid_y = (y / self.resolution + self.grid_size / 2).astype(int)
        
        # Update occupancy grid
        for gx, gy in zip(grid_x, grid_y):
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                self.occupancy_grid[gx, gy] = 255  # Mark as occupied
        
        # Visualize the map
        cv2.imshow('Occupancy Grid', self.occupancy_grid)
        cv2.waitKey(1)
        
        self.get_logger().info(f'Map updated with {len(x)} points')

def main(args=None):
    rclpy.init(args=args)
    node = DepthImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()