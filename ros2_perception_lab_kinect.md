# ROS2 Humble Perception Lab with Kinect Sensor

## Laboratory Guide: Vision and Depth Perception

**Author:** Robotics Perception Lab  
**ROS2 Distribution:** Humble Hawksbill  
**Sensor:** Microsoft Kinect (Xbox 360/One/Azure)  
**Duration:** 4-6 hours

---

## Table of Contents

1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Part 1: Installation and Setup](#part-1-installation-and-setup)
4. [Part 2: Image Processing with Subscribers](#part-2-image-processing-with-subscribers)
5. [Part 3: Depth Analysis](#part-3-depth-analysis)
6. [Part 4: PointCloud Fundamentals](#part-4-pointcloud-fundamentals)
7. [Exercises and Challenges](#exercises-and-challenges)
8. [Troubleshooting](#troubleshooting)
9. [References](#references)

---

## Introduction

### Learning Objectives

By the end of this laboratory, you will be able to:

- Install and configure perception packages for ROS2 Humble
- Process RGB images using OpenCV and cv_bridge
- Analyze depth information from Kinect sensor
- Work with 3D point clouds
- Implement real-time perception algorithms for robotics applications

### Hardware Requirements

- Microsoft Kinect sensor (Xbox 360, Xbox One, or Azure Kinect)
- USB 3.0 port (for Kinect One/Azure) or proprietary adapter (for Kinect 360)
- Computer running Ubuntu 22.04 with ROS2 Humble

---

## Prerequisites

Before starting this lab, ensure you have:

- ROS2 Humble installed on Ubuntu 22.04
- Basic knowledge of Python or C++
- Familiarity with ROS2 concepts (nodes, topics, messages)
- Workspace setup (`~/ros2_ws`)

### Verify ROS2 Installation

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

---

## Part 1: Installation and Setup

### 1.1 Install Point Cloud Library (PCL) and Dependencies

```bash
sudo apt update
sudo apt install -y \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    libpcl-dev \
    pcl-tools
```

### 1.2 Install cv_bridge and Image Processing Tools

```bash
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    python3-opencv
```

### 1.3 Install Kinect Drivers

#### For Kinect Xbox 360 or Xbox One:

```bash
sudo apt install -y \
    ros-humble-freenect-camera \
    libfreenect-dev
```

#### For Azure Kinect:

```bash
# Add Microsoft package repository
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo apt-add-repository https://packages.microsoft.com/ubuntu/22.04/prod

# Install Azure Kinect SDK
sudo apt install -y k4a-tools libk4a1.4-dev

# Install ROS2 driver
sudo apt install -y ros-humble-azure-kinect-ros-driver
```

### 1.4 Additional Useful Packages

```bash
sudo apt install -y \
    ros-humble-image-pipeline \
    ros-humble-depth-image-proc \
    ros-humble-rviz2 \
    ros-humble-rqt-image-view
```

### 1.5 Create Workspace and Package

```bash
cd ~/ros2_ws/src
ros2 pkg create perception_lab \
    --build-type ament_python \
    --dependencies rclpy sensor_msgs std_msgs cv_bridge
cd ~/ros2_ws
colcon build --packages-select perception_lab
source install/setup.bash
```

### 1.6 Launch Kinect Driver

#### For Freenect (Kinect 360/One):

```bash
ros2 launch freenect_camera freenect_camera.launch.py
```

#### For Azure Kinect:

```bash
ros2 launch azure_kinect_ros_driver driver.launch.py
```

### 1.7 Verify Topics

```bash
ros2 topic list
```

Expected topics:
- `/camera/rgb/image_raw` - RGB image
- `/camera/depth/image_raw` - Depth image
- `/camera/depth/points` - Point cloud

---

## Part 2: Image Processing with Subscribers

### 2.1 Basic Image Subscriber

Create `image_subscriber.py` in `perception_lab/perception_lab/`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # Initialize cv_bridge
        self.bridge = CvBridge()
        
        # Subscribe to RGB image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Image subscriber node started')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display image
            cv2.imshow('RGB Camera', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

### 2.2 Image Processing with Edge Detection

Create `edge_detection.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('edge_detection_node')
        
        self.bridge = CvBridge()
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for processed image
        self.publisher = self.create_publisher(
            Image,
            '/camera/edges',
            10
        )
        
        self.get_logger().info('Edge detection node started')
    
    def image_callback(self, msg):
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Canny edge detection
            edges = cv2.Canny(blurred, 50, 150)
            
            # Convert edges back to BGR for visualization
            edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            
            # Publish processed image
            edges_msg = self.bridge.cv2_to_imgmsg(edges_bgr, encoding='bgr8')
            self.publisher.publish(edges_msg)
            
            # Display
            cv2.imshow('Original', cv_image)
            cv2.imshow('Edges', edges)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

### 2.3 Color-based Object Detection

Create `color_detection.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        # HSV color ranges (example: red object)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
        self.get_logger().info('Color detection node started')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create mask for red color
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw contours on original image
            result = cv_image.copy()
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Filter small contours
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(result, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    
                    # Calculate centroid
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(result, (cx, cy), 5, (255, 0, 0), -1)
                        
                        self.get_logger().info(f'Object at ({cx}, {cy}), area: {area:.0f}')
            
            # Display
            cv2.imshow('Original', cv_image)
            cv2.imshow('Mask', mask)
            cv2.imshow('Detection', result)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

---

## Part 3: Depth Analysis

### 3.1 Basic Depth Subscriber

Create `depth_subscriber.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.get_logger().info('Depth subscriber node started')
    
    def depth_callback(self, msg):
        try:
            # Convert depth image (16-bit or 32-bit)
            if msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            elif msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Normalize for visualization
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, 
                                            cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Apply colormap
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # Calculate statistics
            valid_depths = depth_image[depth_image > 0]
            if len(valid_depths) > 0:
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)
                mean_depth = np.mean(valid_depths)
                
                self.get_logger().info(
                    f'Depth - Min: {min_depth:.3f}m, Max: {max_depth:.3f}m, Mean: {mean_depth:.3f}m',
                    throttle_duration_sec=1.0
                )
            
            # Display
            cv2.imshow('Depth Image', depth_colormap)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

### 3.2 Distance Measurement at Specific Point

Create `distance_measurement.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DistanceMeasurement(Node):
    def __init__(self):
        super().__init__('distance_measurement')
        
        self.bridge = CvBridge()
        self.current_depth = None
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        # Create window and set mouse callback
        cv2.namedWindow('Depth Measurement')
        cv2.setMouseCallback('Depth Measurement', self.mouse_callback)
        
        self.get_logger().info('Distance measurement node started')
        self.get_logger().info('Click on image to measure distance')
    
    def depth_callback(self, msg):
        try:
            if msg.encoding == '16UC1':
                self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            elif msg.encoding == '32FC1':
                self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Normalize for visualization
            depth_normalized = cv2.normalize(self.current_depth, None, 0, 255, 
                                            cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            cv2.imshow('Depth Measurement', depth_colormap)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.current_depth is not None:
            # Get depth at clicked point
            depth_value = self.current_depth[y, x]
            
            # Convert to meters (depends on sensor units)
            if self.current_depth.dtype == np.uint16:
                distance_m = depth_value / 1000.0  # millimeters to meters
            else:
                distance_m = depth_value
            
            self.get_logger().info(
                f'Distance at pixel ({x}, {y}): {distance_m:.3f} meters'
            )

def main(args=None):
    rclpy.init(args=args)
    node = DistanceMeasurement()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

### 3.3 Obstacle Detection using Depth

Create `obstacle_detection.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detection')
        
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('distance_threshold', 1.0)  # meters
        self.declare_parameter('min_area', 1000)  # pixels
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            10
        )
        
        self.get_logger().info('Obstacle detection node started')
    
    def depth_callback(self, msg):
        try:
            # Get depth image
            if msg.encoding == '16UC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth_image = depth_image.astype(np.float32) / 1000.0  # to meters
            elif msg.encoding == '32FC1':
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Get parameters
            distance_threshold = self.get_parameter('distance_threshold').value
            min_area = self.get_parameter('min_area').value
            
            # Create binary mask for close objects
            mask = np.zeros(depth_image.shape, dtype=np.uint8)
            mask[(depth_image > 0) & (depth_image < distance_threshold)] = 255
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            
            # Visualize
            depth_colormap = cv2.applyColorMap(
                cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U),
                cv2.COLORMAP_JET
            )
            
            obstacle_count = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > min_area:
                    obstacle_count += 1
                    
                    # Draw bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(depth_colormap, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    
                    # Calculate average distance in region
                    roi = depth_image[y:y+h, x:x+w]
                    avg_distance = np.mean(roi[roi > 0])
                    
                    cv2.putText(depth_colormap, f'{avg_distance:.2f}m', 
                              (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 
                              0.5, (0, 0, 255), 2)
            
            if obstacle_count > 0:
                self.get_logger().info(
                    f'Detected {obstacle_count} obstacles within {distance_threshold}m',
                    throttle_duration_sec=1.0
                )
            
            cv2.imshow('Obstacle Detection', depth_colormap)
            cv2.imshow('Mask', mask)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

---

## Part 4: PointCloud Fundamentals

### 4.1 Basic PointCloud Subscriber

Create `pointcloud_subscriber.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )
        
        self.get_logger().info('PointCloud subscriber node started')
    
    def pointcloud_callback(self, msg):
        try:
            # Extract points from PointCloud2 message
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])
            
            points = np.array(points)
            
            if len(points) > 0:
                # Calculate statistics
                self.get_logger().info(
                    f'PointCloud stats - Points: {len(points)}, '
                    f'X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}], '
                    f'Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}], '
                    f'Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]',
                    throttle_duration_sec=2.0
                )
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.2 PointCloud Filtering

Create `pointcloud_filter.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np

class PointCloudFilter(Node):
    def __init__(self):
        super().__init__('pointcloud_filter')
        
        # Parameters
        self.declare_parameter('x_min', -1.0)
        self.declare_parameter('x_max', 1.0)
        self.declare_parameter('y_min', -1.0)
        self.declare_parameter('y_max', 1.0)
        self.declare_parameter('z_min', 0.0)
        self.declare_parameter('z_max', 3.0)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            PointCloud2,
            '/filtered_points',
            10
        )
        
        self.get_logger().info('PointCloud filter node started')
    
    def pointcloud_callback(self, msg):
        try:
            # Get filter parameters
            x_min = self.get_parameter('x_min').value
            x_max = self.get_parameter('x_max').value
            y_min = self.get_parameter('y_min').value
            y_max = self.get_parameter('y_max').value
            z_min = self.get_parameter('z_min').value
            z_max = self.get_parameter('z_max').value
            
            # Extract and filter points
            filtered_points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
                x, y, z = point[0], point[1], point[2]
                
                # Apply filters
                if (x_min <= x <= x_max and 
                    y_min <= y <= y_max and 
                    z_min <= z <= z_max):
                    filtered_points.append(point)
            
            # Create new PointCloud2 message
            if len(filtered_points) > 0:
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = msg.header.frame_id
                
                filtered_msg = pc2.create_cloud(header, msg.fields, filtered_points)
                self.publisher.publish(filtered_msg)
                
                self.get_logger().info(
                    f'Filtered: {len(filtered_points)} / {msg.width * msg.height} points',
                    throttle_duration_sec=1.0
                )
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.3 Plane Segmentation (RANSAC)

Create `plane_segmentation.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np

class PlaneSegmentation(Node):
    def __init__(self):
        super().__init__('plane_segmentation')
        
        # Parameters
        self.declare_parameter('ransac_iterations', 100)
        self.declare_parameter('distance_threshold', 0.01)
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )
        
        self.plane_publisher = self.create_publisher(
            PointCloud2,
            '/plane_points',
            10
        )
        
        self.objects_publisher = self.create_publisher(
            PointCloud2,
            '/object_points',
            10
        )
        
        self.get_logger().info('Plane segmentation node started')
    
    def fit_plane_ransac(self, points, iterations, threshold):
        """
        Fit a plane using RANSAC algorithm
        Returns: plane coefficients [a, b, c, d] for ax + by + cz + d = 0
        """
        best_inliers = []
        best_plane = None
        
        for _ in range(iterations):
            # Randomly select 3 points
            idx = np.random.choice(len(points), 3, replace=False)
            sample_points = points[idx]
            
            # Calculate plane from 3 points
            v1 = sample_points[1] - sample_points[0]
            v2 = sample_points[2] - sample_points[0]
            normal = np.cross(v1, v2)
            
            if np.linalg.norm(normal) < 1e-6:
                continue
                
            normal = normal / np.linalg.norm(normal)
            d = -np.dot(normal, sample_points[0])
            
            # Calculate distances to plane
            distances = np.abs(np.dot(points, normal) + d)
            
            # Count inliers
            inliers = np.where(distances < threshold)[0]
            
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_plane = np.append(normal, d)
        
        return best_plane, best_inliers
    
    def pointcloud_callback(self, msg):
        try:
            # Extract points
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
                points.append(point)
            
            if len(points) < 3:
                return
            
            points_array = np.array(points)
            xyz = points_array[:, :3]
            
            # Get parameters
            iterations = self.get_parameter('ransac_iterations').value
            threshold = self.get_parameter('distance_threshold').value
            
            # Fit plane
            plane_coeffs, inlier_indices = self.fit_plane_ransac(xyz, iterations, threshold)
            
            if plane_coeffs is not None and len(inlier_indices) > 0:
                # Separate plane and object points
                plane_points = points_array[inlier_indices]
                outlier_mask = np.ones(len(points_array), dtype=bool)
                outlier_mask[inlier_indices] = False
                object_points = points_array[outlier_mask]
                
                # Publish plane points
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = msg.header.frame_id
                
                plane_msg = pc2.create_cloud(header, msg.fields, plane_points.tolist())
                self.plane_publisher.publish(plane_msg)
                
                # Publish object points
                if len(object_points) > 0:
                    objects_msg = pc2.create_cloud(header, msg.fields, object_points.tolist())
                    self.objects_publisher.publish(objects_msg)
                
                self.get_logger().info(
                    f'Plane: {len(inlier_indices)} points, Objects: {len(object_points)} points',
                    throttle_duration_sec=1.0
                )
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PlaneSegmentation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.4 Visualizing PointCloud in RViz2

Launch file `pointcloud_visualization.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Kinect driver
        Node(
            package='freenect_camera',
            executable='freenect_camera_node',
            name='freenect_camera',
            output='screen'
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/path/to/your/config.rviz']
        ),
    ])
```

**RViz2 Configuration:**
1. Launch RViz2: `rviz2`
2. Add > By topic > `/camera/depth/points` > PointCloud2
3. Change Fixed Frame to `camera_depth_optical_frame` or appropriate frame
4. Adjust point size and color scheme

---

## Exercises and Challenges

### Exercise 1: Face Detection with Depth
**Difficulty:** Intermediate

**Objective:** Detect faces using RGB camera and measure distance to each face using depth data.

**Tasks:**
1. Use Haar Cascade or DNN face detector from OpenCV
2. For each detected face, calculate the average depth in that region
3. Display bounding box with distance annotation
4. Publish face positions (x, y, distance) to a custom message topic

**Hints:**
- Use `cv2.CascadeClassifier` with `haarcascade_frontalface_default.xml`
- Synchronize RGB and depth topics
- Handle cases where depth data is invalid

---

### Exercise 2: Object Counting
**Difficulty:** Intermediate

**Objective:** Count objects on a table using depth discontinuities.

**Tasks:**
1. Detect the table plane using RANSAC
2. Remove table points from pointcloud
3. Cluster remaining points (objects on table)
4. Count and visualize each cluster with different colors
5. Publish object count to a topic

**Hints:**
- Use DBSCAN or Euclidean Cluster Extraction for clustering
- Set appropriate distance thresholds for object separation

---

### Exercise 3: 3D Object Localization
**Difficulty:** Advanced

**Objective:** Detect and localize a colored object in 3D space.

**Tasks:**
1. Detect object using color segmentation (HSV)
2. Extract corresponding 3D points from pointcloud
3. Calculate object's centroid in 3D space
4. Transform coordinates to robot base frame (if applicable)
5. Publish object pose (position + orientation)

**Hints:**
- Use `tf2_ros` for coordinate transformations
- Calculate principal axes for orientation estimation

---

### Exercise 4: Real-time Floor Removal
**Difficulty:** Intermediate

**Objective:** Remove floor plane in real-time to focus on objects above ground.

**Tasks:**
1. Implement efficient RANSAC for horizontal plane detection
2. Filter points below the plane
3. Publish filtered pointcloud at >10 Hz
4. Add dynamic reconfigure for threshold adjustment

**Requirements:**
- Processing time < 100ms per frame
- Robust to sensor noise

---

### Exercise 5: 3D Obstacle Map
**Difficulty:** Advanced

**Objective:** Create a 2D occupancy grid from 3D pointcloud for navigation.

**Tasks:**
1. Project pointcloud to 2D plane
2. Create occupancy grid (resolution: 0.05m)
3. Mark cells as free, occupied, or unknown
4. Publish as `nav_msgs/OccupancyGrid`
5. Visualize in RViz2

**Hints:**
- Filter points by height (e.g., 0.1m to 2.0m)
- Use voxel grid for downsampling

---

### Exercise 6: Distance-based Alert System
**Difficulty:** Beginner

**Objective:** Create an alert system that warns when objects are too close.

**Tasks:**
1. Subscribe to depth image
2. Divide image into regions (left, center, right)
3. Calculate minimum distance in each region
4. Publish warning messages when distance < threshold
5. Add audio/visual feedback

**Bonus:**
- Add histogram showing distance distribution

---

### Exercise 7: 3D Object Measurement
**Difficulty:** Advanced

**Objective:** Measure dimensions of objects using pointcloud data.

**Tasks:**
1. Segment individual objects from scene
2. Calculate bounding box (oriented)
3. Measure length, width, height
4. Display measurements on image overlay
5. Save measurements to file (CSV)

**Requirements:**
- Accuracy within 5% for objects >10cm

---

### Exercise 8: Point Cloud Registration
**Difficulty:** Advanced

**Objective:** Align two pointclouds from different viewpoints.

**Tasks:**
1. Capture pointcloud from two positions
2. Implement ICP (Iterative Closest Point) algorithm
3. Merge aligned pointclouds
4. Visualize merged result

**Hints:**
- Use PCL library functions for ICP
- Downsample clouds before registration

---

### Exercise 9: Hand Gesture Recognition
**Difficulty:** Advanced

**Objective:** Recognize simple hand gestures using depth data.

**Tasks:**
1. Detect hand region (closest object to camera)
2. Extract hand pointcloud
3. Calculate features (number of fingers, palm center)
4. Recognize gestures: open palm, fist, pointing
5. Publish recognized gesture

**Bonus:**
- Add support for dynamic gestures (wave, swipe)

---

### Exercise 10: Multi-sensor Fusion
**Difficulty:** Expert

**Objective:** Combine RGB, depth, and pointcloud for robust object detection.

**Tasks:**
1. Create synchronized callback for all three topics
2. Use RGB for color-based detection
3. Use depth for distance filtering
4. Use pointcloud for 3D localization
5. Implement Kalman filter for tracking
6. Handle occlusions and sensor failures

**Requirements:**
- Track multiple objects simultaneously
- Maintain ID across frames

---

## Troubleshooting

### Common Issues

#### 1. Kinect Not Detected

```bash
# Check USB connection
lsusb | grep Xbox

# Test with freenect directly
freenect-glview
```

#### 2. cv_bridge Encoding Errors

```python
# Use try-except with multiple encodings
try:
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
except:
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
```

#### 3. PointCloud2 Empty or NaN values

- Check sensor positioning (not too close to objects)
- Verify IR emitter is working
- Increase lighting in room
- Filter NaN values using `skip_nans=True`

#### 4. High CPU Usage

- Reduce image resolution
- Downsample pointcloud
- Process every N-th frame
- Use multi-threading

#### 5. TF Frame Errors

```bash
# Check available frames
ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame

# View TF tree
ros2 run tf2_tools view_frames
```

---

## References

### Documentation
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [cv_bridge Tutorial](http://wiki.ros.org/cv_bridge/Tutorials)
- [OpenCV Python Documentation](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [Point Cloud Library (PCL)](https://pointclouds.org/)

### Packages
- [freenect_camera](https://github.com/ros-drivers/freenect_camera)
- [Azure Kinect ROS Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver)
- [image_pipeline](https://github.com/ros-perception/image_pipeline)

### Academic Resources
- Zhang, Z. (2012). Microsoft Kinect Sensor and Its Effect. IEEE MultiMedia.
- Rusu, R. B., & Cousins, S. (2011). 3D is here: Point Cloud Library (PCL). ICRA.
- Fischler, M. A., & Bolles, R. C. (1981). Random sample consensus: a paradigm for model fitting. Communications of the ACM.

---

## Appendix: Useful Commands

### Launch Kinect and View Topics
```bash
# Terminal 1: Launch driver
ros2 launch freenect_camera freenect_camera.launch.py

# Terminal 2: View RGB image
ros2 run rqt_image_view rqt_image_view

# Terminal 3: Echo pointcloud stats
ros2 topic echo /camera/depth/points --no-arr
```

### Record and Playback Data
```bash
# Record
ros2 bag record /camera/rgb/image_raw /camera/depth/image_raw /camera/depth/points

# Playback
ros2 bag play <bag_file>
```

### Convert ROS Bag to Images
```bash
ros2 run image_view extract_images_sync image:=/camera/rgb/image_raw
```

---

**End of Laboratory Guide**

*For questions or issues, consult your lab instructor or refer to the ROS2 community forums.*
