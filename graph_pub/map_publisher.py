#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from PIL import Image
import numpy as np

class MapPublisher(Node):
    """
    ROS2 Node to publish a static map from a map file.
    """
    def __init__(self):
        super().__init__('map_publisher')
        
        # Declare parameters
        self.declare_parameter('map_file', 'mapaTransformSoloLab')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_frequency', 1.0)  # Hz
        
        # Get parameters
        self.map_name = self.get_parameter('map_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        
        # Create QoS profile for map
        map_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publisher with proper QoS
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', map_qos)
        
        # Setup timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_map)
        
        # Setup TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Load map
        self.map_msg = self.load_map()
        
        self.get_logger().info(f'Map publisher initialized with map: {self.map_name}')
    
    def load_map(self):
        """
        Load map from file and create OccupancyGrid message.
        """
        package_share_dir = get_package_share_directory('graph_pub')
        map_dir = os.path.join(package_share_dir, 'maps')
        yaml_file = os.path.join(map_dir, f'{self.map_name}.yaml')
        
        self.get_logger().info(f'Loading map from: {yaml_file}')
        
        # Read YAML file
        with open(yaml_file, 'r') as f:
            map_data = yaml.safe_load(f)
        
        # Get map properties
        image_path = os.path.join(map_dir, map_data['image'])
        resolution = float(map_data['resolution'])
        origin = map_data['origin']
        
        self.get_logger().info(f'Map image path: {image_path}')
        self.get_logger().info(f'Map resolution: {resolution}')
        self.get_logger().info(f'Map origin: {origin}')
        
      
            
        try:
            # Use PIL/Pillow for more robust image reading
            img = Image.open(image_path)
            img_data = np.array(img)
            
            # Create OccupancyGrid message
            map_msg = OccupancyGrid()
            map_msg.header.frame_id = self.frame_id
            map_msg.info.resolution = resolution
            map_msg.info.width = img.width
            map_msg.info.height = img.height
            map_msg.info.origin.position.x = float(origin[0])
            map_msg.info.origin.position.y = float(origin[1])
            map_msg.info.origin.position.z = float(origin[2])
            
            map_msg.info.origin.orientation.x = 0.0 #float(origin[0])
            map_msg.info.origin.orientation.y = 0.0 #float(origin[1])
            map_msg.info.origin.orientation.z =  0.0 #float(origin[2])
            map_msg.info.origin.orientation.w = 1.0
            
            # Convert image data to occupancy grid data
            map_msg.data = []
            for y in range(img.height - 1, -1, -1):
                for x in range(img.width):
                    pixel_val = img_data[y, x]
                    
                    # Convert from 0-255 to 0-100 (occupancy probability)
                    if pixel_val < 20:  # Black (occupied)
                        map_msg.data.append(100)
                    elif pixel_val > 235:  # White (free)
                        map_msg.data.append(0)
                    else:  # Gray (unknown)
                        map_msg.data.append(-1)
        except Exception as e:
            self.get_logger().error(f'Error loading map: {e}')
            # Return an empty map
            map_msg = OccupancyGrid()
            map_msg.header.frame_id = self.frame_id
            
        return map_msg
    
    def publish_map(self):
        """
        Publish the map and the transform.
        """
        # Update timestamp
        now = self.get_clock().now().to_msg()
        self.map_msg.header.stamp = now
        
        # Publish map
        self.map_publisher.publish(self.map_msg)
        
        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = 'world'
        transform.child_frame_id = self.frame_id
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
