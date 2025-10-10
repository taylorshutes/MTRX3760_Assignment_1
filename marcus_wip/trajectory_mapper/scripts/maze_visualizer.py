#!/usr/bin/env python3
"""
Maze Wall Visualizer for RViz

Publishes visualization markers representing maze walls by reading
directly from Gazebo's model state.

Usage:
    ros2 run trajectory_mapper maze_visualizer.py
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math


class MazeVisualizer(Node):
    def __init__(self):
        super().__init__('maze_visualizer')
        
        # Publisher for wall markers
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/maze_walls', 
            10
        )
        
        # Timer to publish walls periodically
        self.timer = self.create_timer(2.0, self.publish_maze_walls)
        
        # Known wall models from the world files
        self.wall_models = [
            # Outer walls
            'outer_north', 'outer_south', 'outer_east', 'outer_west',
            # Inner walls
            'inner_wall1', 'inner_wall2', 'inner_wall3', 'inner_wall4',
            'inner_wall5', 'inner_wall6', 'inner_wall7', 'inner_wall8',
            'inner_wall9', 'inner_wall10', 'inner_wall11', 'inner_wall12',
            'inner_wall13', 'inner_wall14', 'inner_wall15', 'inner_wall16',
            'inner_wall17', 'inner_wall18', 'inner_wall19', 'inner_wall20',
            'inner_wall21', 'inner_wall22',
            # Coloured walls
            'start_wall', 'finish_wall', 'end_wall'
        ]
        
        self.get_logger().info('Maze Visualizer started - publishing wall markers')
        
    def publish_maze_walls(self):
        """Publish visualization markers for all maze walls"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # Create markers for known walls from world definition
        # These are hardcoded positions from tay_maze.world
        walls = self.get_tay_maze_walls()
        
        for wall in walls:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "maze_walls"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = wall['x']
            marker.pose.position.y = wall['y']
            marker.pose.position.z = 0.5  # Half height of wall
            
            # Orientation
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(wall['yaw'] / 2.0)
            marker.pose.orientation.w = math.cos(wall['yaw'] / 2.0)
            
            # Scale (size of wall)
            marker.scale.x = wall['length']
            marker.scale.y = wall['width']
            marker.scale.z = wall['height']
            
            # Color
            if 'start' in wall.get('name', ''):
                # Red for start
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            elif 'finish' in wall.get('name', '') or 'end' in wall.get('name', ''):
                # Green for finish
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
            else:
                # Grey for regular walls
                marker.color.r = 0.6
                marker.color.g = 0.6
                marker.color.b = 0.6
                marker.color.a = 0.6
            
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # Persistent
            
            marker_array.markers.append(marker)
            marker_id += 1
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} wall markers', 
                              once=True)
    
    def get_tay_maze_walls(self):
        """
        Return wall definitions from tay_maze.world
        Parse the SDF positions into marker-friendly format
        """
        walls = [
            # Outer walls
            {'name': 'outer_north', 'x': 0.0, 'y': 5.0, 'yaw': 0.0, 
             'length': 10.0, 'width': 0.2, 'height': 1.0},
            {'name': 'outer_south', 'x': -0.1, 'y': -5.0, 'yaw': 0.0, 
             'length': 8.0, 'width': 0.2, 'height': 1.0},
            {'name': 'outer_east', 'x': 5.1, 'y': 0.0, 'yaw': 1.5708, 
             'length': 10.2, 'width': 0.2, 'height': 1.0},
            {'name': 'outer_west', 'x': -5.1, 'y': 0.0, 'yaw': 1.5708, 
             'length': 10.2, 'width': 0.2, 'height': 1.0},
            
            # Inner walls (from tay_maze.world)
            {'name': 'inner_wall1', 'x': -4.0, 'y': -3.0, 'yaw': 1.5708, 
             'length': 3.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall2', 'x': -2.7, 'y': -1.0, 'yaw': 0.0, 
             'length': 2.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall3', 'x': -3.0, 'y': -3.0, 'yaw': 1.5708, 
             'length': 1.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall4', 'x': -1.7, 'y': -2.0, 'yaw': 0.0, 
             'length': 2.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall5', 'x': -2.2, 'y': -4.0, 'yaw': 0.0, 
             'length': 1.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall6', 'x': -0.4, 'y': -0.7, 'yaw': 1.5708, 
             'length': 2.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall7', 'x': -0.4, 'y': -3.9, 'yaw': 1.5708, 
             'length': 2.0, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall8', 'x': -1.2, 'y': -3.0, 'yaw': 0.0, 
             'length': 1.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall9', 'x': 0.6, 'y': -0.7, 'yaw': 1.5708, 
             'length': 6.2, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall10', 'x': 1.6, 'y': -3.6, 'yaw': 1.5708, 
             'length': 2.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall11', 'x': -1.7, 'y': 2.4, 'yaw': 0.0, 
             'length': 6.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall12', 'x': -1.7, 'y': 0.8, 'yaw': 0.0, 
             'length': 2.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall13', 'x': -1.4, 'y': 0.8, 'yaw': 1.5708, 
             'length': 1.2, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall14', 'x': -2.2, 'y': -0.7, 'yaw': 1.5708, 
             'length': 0.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall15', 'x': -3.1, 'y': 0.5, 'yaw': 1.5708, 
             'length': 0.8, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall16', 'x': -4.0, 'y': 0.0, 'yaw': 0.0, 
             'length': 2.0, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall17', 'x': 2.1, 'y': -2.3, 'yaw': 0.0, 
             'length': 1.2, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall18', 'x': 3.8, 'y': -2.2, 'yaw': 1.5708, 
             'length': 5.6, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall19', 'x': 3.0, 'y': -3.8, 'yaw': 0.0, 
             'length': 1.4, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall20', 'x': 2.6, 'y': -0.6, 'yaw': 0.0, 
             'length': 2.2, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall21', 'x': 2.7, 'y': 1.6, 'yaw': 1.5708, 
             'length': 4.2, 'width': 0.2, 'height': 1.0},
            {'name': 'inner_wall22', 'x': 1.2, 'y': 3.6, 'yaw': 0.0, 
             'length': 5.0, 'width': 0.2, 'height': 1.0},
            
            # Start/Finish walls
            {'name': 'start_wall', 'x': -4.55, 'y': -5.0, 'yaw': 0.0, 
             'length': 0.9, 'width': 0.2, 'height': 1.0},
            {'name': 'finish_wall', 'x': 4.5, 'y': -5.7, 'yaw': 0.0, 
             'length': 2.2, 'width': 0.2, 'height': 1.0},
        ]
        
        return walls


def main(args=None):
    rclpy.init(args=args)
    node = MazeVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
