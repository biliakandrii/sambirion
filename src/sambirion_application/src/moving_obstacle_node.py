#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist, TransformStamped, PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from tf2_ros import TransformBroadcaster

class MovingObstacleNode(Node):
    def __init__(self):
        super().__init__('moving_obstacle_node')
        
        # Declare and get parameters
        self.declare_parameter('model_name', 'moving_obstacle')
        self.declare_parameter('trajectory', 'circular')
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('obstacle_radius', 0.05)
        self.declare_parameter('obstacle_height', 1.5)
        self.declare_parameter('color_r', 1.0)
        self.declare_parameter('color_g', 0.0)
        self.declare_parameter('color_b', 0.0)
        self.declare_parameter('visualize_path', True)
        self.declare_parameter('linear_axis', 'x')  # 'x' or 'y'
        self.declare_parameter('linear_direction', 'increment')  # 'increment' or 'decrement'
        
        self.model_name = self.get_parameter('model_name').value
        self.trajectory_type = self.get_parameter('trajectory').value
        self.speed = self.get_parameter('speed').value
        self.radius = self.get_parameter('radius').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.obstacle_height = self.get_parameter('obstacle_height').value
        self.color_r = self.get_parameter('color_r').value
        self.color_g = self.get_parameter('color_g').value
        self.color_b = self.get_parameter('color_b').value
        self.visualize_path = self.get_parameter('visualize_path').value
        self.linear_axis = self.get_parameter('linear_axis').value
        self.linear_direction = self.get_parameter('linear_direction').value
        
        # Generate unique marker ID based on model name hash
        self.marker_id = hash(self.model_name) % 10000
        
        # Movement control flags
        self.movement_started = False
        self.start_time = None
        
        # Print configuration
        self.get_logger().info("=" * 50)
        self.get_logger().info("Moving Obstacle Configuration:")
        self.get_logger().info(f"  Model: {self.model_name}")
        self.get_logger().info(f"  Trajectory: {self.trajectory_type}")
        self.get_logger().info(f"  Speed: {self.speed} rad/s")
        self.get_logger().info(f"  Radius: {self.radius} m")
        self.get_logger().info(f"  Start Position: ({self.start_x}, {self.start_y})")
        if self.trajectory_type == 'linear':
            self.get_logger().info(f"  Linear Axis: {self.linear_axis}")
            self.get_logger().info(f"  Linear Direction: {self.linear_direction}")
        self.get_logger().info(f"  Size: radius={self.obstacle_radius}m, height={self.obstacle_height}m")
        self.get_logger().info(f"  Color: RGB({self.color_r}, {self.color_g}, {self.color_b})")
        self.get_logger().info(f"  Marker ID: {self.marker_id}")
        self.get_logger().info(f"  Status: WAITING for /goal_pose message")
        self.get_logger().info("=" * 50)
        
        # Create service clients (shared across instances)
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        # Publishers for visualization - unique per instance
        if self.visualize_path:
            self.marker_pub = self.create_publisher(
                Marker, 
                f'/moving_obstacle_marker/{self.model_name}', 
                10
            )
            self.path_marker_pub = self.create_publisher(
                Marker, 
                f'/moving_obstacle_path/{self.model_name}', 
                10
            )
        
        # TF broadcaster for the obstacle
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to /goal_pose to trigger movement
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        
        self.spawned = False
        
        # Wait for services
        self.get_logger().info("Waiting for Gazebo spawn service...")
        if not self.spawn_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Spawn service not available!")
            self.get_logger().error("Make sure Gazebo is running with: ros2 launch gazebo_ros gazebo.launch.py")
            return
        
        # Spawn the obstacle
        self.spawn_obstacle()
        
        # Create timer for updating position (30 Hz)
        self.timer = self.create_timer(1.0/30.0, self.update_position)
        
    def goal_pose_callback(self, msg):
        """Callback when goal_pose is received - starts the movement"""
        if not self.movement_started:
            self.movement_started = True
            self.start_time = self.get_clock().now()
            self.get_logger().info(f"🚀 {self.model_name}: Movement STARTED! (Goal pose received)")
            self.get_logger().info(f"   Goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        
    def generate_sdf(self):
        """Generate SDF model string with physics plugin"""
        sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{self.model_name}">
    <pose>0 0 {self.obstacle_height / 2.0} 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.166</ixx>
          <iyy>0.166</iyy>
          <izz>0.166</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{self.obstacle_radius}</radius>
            <length>{self.obstacle_height}</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.0</mu>
              <mu2>0.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{self.obstacle_radius}</radius>
            <length>{self.obstacle_height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{self.color_r} {self.color_g} {self.color_b} 1</ambient>
          <diffuse>{self.color_r} {self.color_g} {self.color_b} 1</diffuse>
        </material>
      </visual>
    </link>
    
    <plugin name="gazebo_ros_planar_move" filename="libgazebo_ros_planar_move.so">
      <ros>
        <namespace>/{self.model_name}</namespace>
      </ros>
      <update_rate>100</update_rate>
      <publish_rate>10</publish_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>{self.model_name}</robot_base_frame>
    </plugin>
  </model>
</sdf>"""
        return sdf
    
    def spawn_obstacle(self):
        """Spawn the obstacle in Gazebo"""
        try:
            # Try to delete existing model first
            delete_req = DeleteEntity.Request()
            delete_req.name = self.model_name
            try:
                future = self.delete_client.call_async(delete_req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            except:
                pass
            
            # Wait a bit
            import time
            time.sleep(0.5)
            
            # Spawn new model
            spawn_req = SpawnEntity.Request()
            spawn_req.name = self.model_name
            spawn_req.xml = self.generate_sdf()
            spawn_req.robot_namespace = ""
            
            initial_pose = Pose()
            initial_pose.position.x = self.start_x
            initial_pose.position.y = self.start_y
            initial_pose.position.z = 0.0
            initial_pose.orientation.w = 1.0
            spawn_req.initial_pose = initial_pose
            
            future = self.spawn_client.call_async(spawn_req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                self.get_logger().info(f"Successfully spawned {self.model_name}")
                self.spawned = True
                
                # Create cmd_vel publisher for the spawned model - unique per instance
                self.cmd_vel_pub = self.create_publisher(
                    Twist, 
                    f'/{self.model_name}/cmd_vel', 
                    10
                )
                self.get_logger().info(f"Publishing velocity commands to /{self.model_name}/cmd_vel")
                
                return True
            else:
                self.get_logger().error("Failed to spawn model")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Failed to spawn model: {e}")
            return False
    
    def circular_trajectory(self, t):
        """Generate circular trajectory"""
        x = self.start_x + self.radius * math.cos(self.speed * t)
        y = self.start_y + self.radius * math.sin(self.speed * t)
        yaw = self.speed * t + math.pi/2
        
        # Calculate velocities
        vx = -self.radius * self.speed * math.sin(self.speed * t)
        vy = self.radius * self.speed * math.cos(self.speed * t)
        vyaw = self.speed
        
        return x, y, yaw, vx, vy, vyaw
    
    def linear_trajectory(self, t):
        """Generate back-and-forth linear trajectory"""
        # Determine direction multiplier
        direction = 1.0 if self.linear_direction == 'increment' else -1.0
        
        if self.linear_axis == 'x':
            # Movement along X axis
            x = self.start_x + direction * self.radius * math.sin(self.speed * t)
            y = self.start_y
            yaw = 0 if direction > 0 else math.pi
            
            vx = direction * self.radius * self.speed * math.cos(self.speed * t)
            vy = 0.0
        else:  # linear_axis == 'y'
            # Movement along Y axis
            x = self.start_x
            y = self.start_y + direction * self.radius * math.sin(self.speed * t)
            yaw = math.pi/2 if direction > 0 else -math.pi/2
            
            vx = 0.0
            vy = direction * self.radius * self.speed * math.cos(self.speed * t)
        
        vyaw = 0.0
        
        return x, y, yaw, vx, vy, vyaw
    
    def figure8_trajectory(self, t):
        """Generate figure-8 trajectory"""
        x = self.start_x + self.radius * math.sin(self.speed * t)
        y = self.start_y + self.radius * math.sin(2 * self.speed * t) / 2
        
        dx = self.radius * self.speed * math.cos(self.speed * t)
        dy = self.radius * self.speed * math.cos(2 * self.speed * t)
        yaw = math.atan2(dy, dx)
        
        vx = dx
        vy = dy
        vyaw = 0.0
        
        return x, y, yaw, vx, vy, vyaw
    
    def square_trajectory(self, t):
        """Generate square trajectory"""
        period = 2 * math.pi / self.speed
        side_time = period / 4
        current_time = t % period
        
        if current_time < side_time:
            progress = current_time / side_time
            x = self.start_x + self.radius * progress
            y = self.start_y
            yaw = 0
            vx = self.radius / side_time
            vy = 0.0
        elif current_time < 2 * side_time:
            progress = (current_time - side_time) / side_time
            x = self.start_x + self.radius
            y = self.start_y + self.radius * progress
            yaw = math.pi/2
            vx = 0.0
            vy = self.radius / side_time
        elif current_time < 3 * side_time:
            progress = (current_time - 2*side_time) / side_time
            x = self.start_x + self.radius * (1 - progress)
            y = self.start_y + self.radius
            yaw = math.pi
            vx = -self.radius / side_time
            vy = 0.0
        else:
            progress = (current_time - 3*side_time) / side_time
            x = self.start_x
            y = self.start_y + self.radius * (1 - progress)
            yaw = -math.pi/2
            vx = 0.0
            vy = -self.radius / side_time
        
        vyaw = 0.0
        return x, y, yaw, vx, vy, vyaw
    
    def update_position(self):
        """Update obstacle position based on trajectory"""
        if not self.spawned:
            return
        
        # Don't move until goal_pose is received
        if not self.movement_started:
            # Keep obstacle stationary at start position
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            # Still visualize the static position and planned path
            if self.visualize_path:
                self.publish_marker(self.start_x, self.start_y, 0.0)
            return
            
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9
        
        # Get trajectory based on type
        if self.trajectory_type == 'circular':
            x, y, yaw, vx, vy, vyaw = self.circular_trajectory(t)
        elif self.trajectory_type == 'linear':
            x, y, yaw, vx, vy, vyaw = self.linear_trajectory(t)
        elif self.trajectory_type == 'figure8':
            x, y, yaw, vx, vy, vyaw = self.figure8_trajectory(t)
        elif self.trajectory_type == 'square':
            x, y, yaw, vx, vy, vyaw = self.square_trajectory(t)
        else:
            self.get_logger().warn(f"Unknown trajectory type: {self.trajectory_type}")
            return
        
        # Publish velocity command to the planar move plugin
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = vyaw
        self.cmd_vel_pub.publish(twist)
        
        # Publish visualization markers
        if self.visualize_path:
            self.publish_marker(x, y, yaw)
    
    def publish_marker(self, x, y, yaw):
        """Publish visualization marker for RViz"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"moving_obstacle_{self.model_name}"
        marker.id = self.marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = self.obstacle_height / 2.0
        marker.pose.orientation.z = math.sin(yaw/2)
        marker.pose.orientation.w = math.cos(yaw/2)
        
        marker.scale.x = self.obstacle_radius * 2
        marker.scale.y = self.obstacle_radius * 2
        marker.scale.z = self.obstacle_height
        
        marker.color.r = self.color_r
        marker.color.g = self.color_g
        marker.color.b = self.color_b
        marker.color.a = 0.8
        
        self.marker_pub.publish(marker)
        
        # Publish path trajectory line
        self.publish_path_marker()
    
    def publish_path_marker(self):
        """Publish the trajectory path as a line strip"""
        path_marker = Marker()
        path_marker.header.frame_id = "odom"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = f"obstacle_path_{self.model_name}"
        path_marker.id = self.marker_id + 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        
        # Generate path points
        for i in range(100):
            t = i * 0.1
            if self.trajectory_type == 'circular':
                x, y, _, _, _, _ = self.circular_trajectory(t)
            elif self.trajectory_type == 'linear':
                x, y, _, _, _, _ = self.linear_trajectory(t)
            elif self.trajectory_type == 'figure8':
                x, y, _, _, _, _ = self.figure8_trajectory(t)
            elif self.trajectory_type == 'square':
                x, y, _, _, _, _ = self.square_trajectory(t)
            else:
                continue
            
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.01
            path_marker.points.append(point)
        
        path_marker.scale.x = 0.05
        path_marker.color.r = self.color_r
        path_marker.color.g = self.color_g
        path_marker.color.b = self.color_b
        path_marker.color.a = 0.5
        
        self.path_marker_pub.publish(path_marker)
    
    def destroy_node(self):
        """Clean up on shutdown"""
        self.get_logger().info(f"Shutting down and removing {self.model_name}...")
        try:
            delete_req = DeleteEntity.Request()
            delete_req.name = self.model_name
            self.delete_client.call_async(delete_req)
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MovingObstacleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()