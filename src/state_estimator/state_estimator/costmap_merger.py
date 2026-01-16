#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformException
import numpy as np
from rclpy.duration import Duration


class CostmapMerger(Node):
    def __init__(self):
        super().__init__('costmap_merger')
        
        # Declare parameters
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('heatmap_topic', '/predicted_obstacles_heatmap')
        self.declare_parameter('output_topic', '/merged_costmap')
        self.declare_parameter('output_nav2_topic', '/merged_costmap_nav2')
        self.declare_parameter('merge_method', 'max')
        self.declare_parameter('heatmap_weight', 0.5)
        self.declare_parameter('costmap_weight', 0.5)
        
        # Get parameters
        costmap_topic = self.get_parameter('costmap_topic').value
        heatmap_topic = self.get_parameter('heatmap_topic').value
        output_topic = self.get_parameter('output_topic').value
        output_nav2_topic = self.get_parameter('output_nav2_topic').value
        self.merge_method = self.get_parameter('merge_method').value
        self.heatmap_weight = self.get_parameter('heatmap_weight').value
        self.costmap_weight = self.get_parameter('costmap_weight').value
        
        # Initialize TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Storage for maps
        self.heatmap = None
        self.costmap = None
        
        # Subscribers
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            costmap_topic,
            self.costmap_callback,
            10
        )
        
        self.heatmap_sub = self.create_subscription(
            OccupancyGrid,
            heatmap_topic,
            self.heatmap_callback,
            10
        )
        
        # Publishers
        self.merged_pub = self.create_publisher(OccupancyGrid, output_topic, 10)
        self.merged_nav2_pub = self.create_publisher(Costmap, output_nav2_topic, 10)
        self.merged_update_pub = self.create_publisher(
            OccupancyGridUpdate, 
            output_topic + '_updates', 
            10
        )
        
        self.get_logger().info(f'Costmap Merger Node initialized')
        self.get_logger().info(f'Publishing updates to: {output_topic}_updates')
        
    def heatmap_callback(self, msg):
        self.heatmap = msg
        self.merge_and_publish()
        
    def costmap_callback(self, msg):
        self.costmap = msg
        self.merge_and_publish()
        
    def occupancy_grid_to_nav2_costmap(self, occupancy_grid, merged_data):
        costmap_msg = Costmap()
        costmap_msg.header = occupancy_grid.header
        costmap_msg.metadata.size_x = occupancy_grid.info.width
        costmap_msg.metadata.size_y = occupancy_grid.info.height
        costmap_msg.metadata.resolution = occupancy_grid.info.resolution
        costmap_msg.metadata.origin = occupancy_grid.info.origin
        costmap_msg.metadata.map_load_time = occupancy_grid.header.stamp
        
        nav2_data = np.zeros_like(merged_data, dtype=np.uint8)
        for i in range(merged_data.shape[0]):
            for j in range(merged_data.shape[1]):
                val = merged_data[i, j]
                if val == -1:
                    nav2_data[i, j] = 255
                elif val == 0:
                    nav2_data[i, j] = 0
                elif val >= 100:
                    nav2_data[i, j] = 254
                else:
                    nav2_data[i, j] = int((val / 100.0) * 252) + 1
        
        costmap_msg.data = nav2_data.flatten().tolist()
        return costmap_msg
        
    def merge_and_publish(self):
        if self.costmap is None:
            return
        elif self.heatmap is None:
            self.merged_pub.publish(self.costmap)
            costmap_data = np.array(self.costmap.data, dtype=np.int8).reshape(
                (self.costmap.info.height, self.costmap.info.width)
            )
            nav2_msg = self.occupancy_grid_to_nav2_costmap(self.costmap, costmap_data)
            self.merged_nav2_pub.publish(nav2_msg)
            return
            
        try:
            transform = self.tf_buffer.lookup_transform(
                self.costmap.header.frame_id,
                self.heatmap.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            )
            
            merged_map = OccupancyGrid()
            merged_map.header = self.costmap.header
            merged_map.info = self.costmap.info
            
            costmap_data = np.array(self.costmap.data, dtype=np.int8).reshape(
                (self.costmap.info.height, self.costmap.info.width)
            )
            
            heatmap_data = np.array(self.heatmap.data, dtype=np.int8).reshape(
                (self.heatmap.info.height, self.heatmap.info.width)
            )
            
            merged_data = costmap_data.copy()
            
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            
            for i in range(self.costmap.info.height):
                for j in range(self.costmap.info.width):
                    world_x = self.costmap.info.origin.position.x + (j + 0.5) * self.costmap.info.resolution
                    world_y = self.costmap.info.origin.position.y + (i + 0.5) * self.costmap.info.resolution
                    
                    map_x = world_x + tx
                    map_y = world_y + ty
                    
                    heatmap_j = int((map_x - self.heatmap.info.origin.position.x) / self.heatmap.info.resolution)
                    heatmap_i = int((map_y - self.heatmap.info.origin.position.y) / self.heatmap.info.resolution)
                    
                    if (0 <= heatmap_i < self.heatmap.info.height and 
                        0 <= heatmap_j < self.heatmap.info.width):
                        
                        costmap_val = costmap_data[i, j]
                        heatmap_val = heatmap_data[heatmap_i, heatmap_j]
                        
                        if costmap_val == -1:
                            costmap_val = 0
                        if heatmap_val == -1:
                            heatmap_val = 0
                            
                        if self.merge_method == 'max':
                            merged_data[i, j] = max(costmap_val, heatmap_val)
                        elif self.merge_method == 'average':
                            merged_data[i, j] = int((costmap_val + heatmap_val) / 2)
                        elif self.merge_method == 'weighted':
                            merged_val = (self.costmap_weight * costmap_val + 
                                        self.heatmap_weight * heatmap_val)
                            merged_data[i, j] = int(min(100, merged_val))
            
            merged_map.data = merged_data.flatten().tolist()
            
            # Publish full map
            self.merged_pub.publish(merged_map)
            
            # Publish Nav2 costmap
            nav2_costmap = self.occupancy_grid_to_nav2_costmap(merged_map, merged_data)
            self.merged_nav2_pub.publish(nav2_costmap)
            
            # Publish update using costmap dimensions and origin
            update_msg = OccupancyGridUpdate()
            update_msg.header = merged_map.header
            update_msg.x = int(self.costmap.info.origin.position.x / self.costmap.info.resolution)
            update_msg.y = int(self.costmap.info.origin.position.y / self.costmap.info.resolution)
            update_msg.width = self.costmap.info.width
            update_msg.height = self.costmap.info.height
            update_msg.data = merged_data.flatten().tolist()
            self.merged_update_pub.publish(update_msg)
            
        except TransformException as e:
            self.get_logger().warn(f'Could not transform: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error merging maps: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = CostmapMerger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()