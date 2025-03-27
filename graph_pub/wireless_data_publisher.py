#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import xml.etree.ElementTree as ET
import numpy as np
from ament_index_python.packages import get_package_share_directory
import struct
import yaml
from graph_pub.xml_manager import read_xml
from scipy.spatial.transform import Rotation as R

class WirelessDataPublisher(Node):
    """
    ROS2 Node to read wireless data positions from XML and publish as PointCloud2.
    """
    def __init__(self):
        super().__init__('wireless_data_publisher')
        
        # Declare parameters
        self.declare_parameter('xml_file', 'wireless_data_transform_lab.xml')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_frequency', 1.0)  # Hz
        self.declare_parameter('map_file', 'mapaTransformSoloLab')  # Added to get map origin
        
        # Get parameters
        self.xml_file = self.get_parameter('xml_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.map_name = self.get_parameter('map_file').value
        
        # Get map origin and orientation for coordinate transformation
        self.map_origin, self.map_orientation = self.get_map_origin_and_orientation()
        
        # Create publisher
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2, 'wireless_positions', 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        
        # Load wireless data and apply the static transform
        self.positions = self.load_and_transform_xml_data()
        
        # Create the point cloud once
        self.static_point_cloud = self.create_point_cloud(self.positions)
        
        # Setup timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_wireless_data)
        
        self.get_logger().info(f'Wireless data publisher initialized with {len(self.positions)} positions')
    
    def get_map_origin_and_orientation(self):
        """
        Get the map origin and orientation from the map YAML file for coordinate transformation.
        """
        package_share_dir = get_package_share_directory('graph_pub')
        map_dir = os.path.join(package_share_dir, 'maps')
        yaml_file = os.path.join(map_dir, f'{self.map_name}.yaml')
        
        default_origin = [0.0, 0.0, 0.0]
        
        try:
            with open(yaml_file, 'r') as f:
                map_data = yaml.safe_load(f)
                origin =  map_data.get('origin', [0.0, 0.0, 0.0])
                orientation = map_data.get('orientation', [0.0, 0.0, 0.0, 1.0])
                self.get_logger().info(f'Map origin for transformation: {origin}')
                self.get_logger().info(f'Map orientation for transformation: {orientation}')
                return origin, orientation
        except Exception as e:
            self.get_logger().error(f'Error loading map origin and orientation: {e}')
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]  # Default if error
    
    def apply_map_transform(self, point, origin, orientation):
        """
        Apply the map origin and orientation transformation to a point.
        """
        
        # Create the rotation matrix from the quaternion
        rotation = R.from_quat(orientation)
        
        rotated_point = rotation.apply([point[0], point[1], point[2]])
        
        # Apply the translation based on the map origin
        translated_point = np.array([
            rotated_point[0] + origin[0],
            rotated_point[1] + origin[1],
            rotated_point[2] + origin[2],
            point[3]
        ])
        
        return translated_point
    
    def quaternion_to_euler(x, y, z, w):
        """Convert quaternion to Euler angles (yaw only)."""
        import math
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw

    def load_and_transform_xml_data(self):
        positions = []
        try:
            data = read_xml(".", self.xml_file)
            
            for entry in data:
                position = entry['Position']
                x = float(position['X'])
                y = float(position['Y'])
                z = float(position['Z'])
                
                orientation = entry['Orientation']
                qx = float(orientation['X'])
                qy = float(orientation['Y'])
                qz = float(orientation['Z'])
                qw = float(orientation['W'])
                
                # Apply the map transform
                positions.append([x, y, z, 0.5])
            
            return positions
            
        except FileNotFoundError:
            self.get_logger().error(f'File not found: {self.xml_file}')
            return []
        except ET.ParseError as e:
            self.get_logger().error(f'Error parsing XML: {e}')
            return []

    def create_point_cloud(self, points):
        """
        Create a PointCloud2 message from a list of points.
        Each point is (x, y, z, intensity) where intensity is the WiFi signal strength.
        """
        # Define the fields for the point cloud
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create header with fixed timestamp
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # Use current time
        header.frame_id = self.frame_id
        
        # Create point cloud
        point_cloud = PointCloud2(
            header=header,
            height=1,
            width=len(points),
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=16,  # 4 floats * 4 bytes
            row_step=16 * len(points),
        )
        
        # Pack data into bytes
        buffer = bytearray(point_cloud.row_step)
        for i, point in enumerate(points):
            offset = i * 16
            struct.pack_into('ffff', buffer, offset, *point)
        
        point_cloud.data = buffer
        
        return point_cloud
    
    def create_marker(self, points):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "wireless_data"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.a = 1.0  # Alpha
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        for i in range(len(points) - 1):
            p1 = Point()
            p1.x = points[i][0]
            p1.y = points[i][1]
            p1.z = points[i][2]
            marker.points.append(p1)

            p2 = Point()
            p2.x = points[i + 1][0]
            p2.y = points[i + 1][1]
            p2.z = points[i + 1][2]
            marker.points.append(p2)

        return marker

    def publish_wireless_data(self):
        """
        Publish wireless data positions as PointCloud2 and Marker.
        """
        if not self.positions:
            self.get_logger().warn('No wireless data positions to publish')
            return
        
        # Update timestamp
        self.static_point_cloud.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the point cloud
        self.pointcloud_publisher.publish(self.static_point_cloud)

        # Create and publish the marker
        marker = self.create_marker(self.positions)
        self.marker_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = WirelessDataPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
