from gs_ros import GsRosBridge
import rclpy
from rclpy.node import Node
import genesis as gs
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan 
from tf2_ros import TransformBroadcaster
import numpy as np
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration

def gs_quat_to_ros(quat):
    quat = np.array(quat).flatten()
    if len(quat) != 4:
        return [0.0, 0.0, 0.0, 1.0]
    return [quat[1], quat[2], quat[3], quat[0]]

def main(args=None):
    gs.init(logging_level="debug", performance_mode=True)
    rclpy.init(args=args)
    
    system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    default_ros_node = Node('gs_ros_bridge_node')
    
    tf_broadcaster = TransformBroadcaster(default_ros_node)
    odom_pub = default_ros_node.create_publisher(Odometry, '/odom', 10)
    scan_pub = default_ros_node.create_publisher(LaserScan, '/scan', 10)

    def scan_callback(msg):
        msg.header.stamp = system_clock.now().to_msg()
        scan_pub.publish(msg)

    default_ros_node.create_subscription(LaserScan, '/go2/laser_scan_topic_raw', scan_callback, 10)

    gs_ros_bridge = GsRosBridge(default_ros_node, "/home/datvu/quadruped_hust_ws/src/go2.yaml", add_debug_objects=True)
    gs_ros_bridge.build()
    
    robot_entity = None
    try:
        if len(gs_ros_bridge.robots) > 0:
            robot_entity = gs_ros_bridge.robots[0][2]
            gs.logger.info("Đã tìm thấy Robot Entity.")
    except Exception as e:
        gs.logger.error(f"Không thể lấy robot entity: {e}")

    while rclpy.ok():
        rclpy.spin_once(default_ros_node, timeout_sec=0)
        gs_ros_bridge.step()
        
        if robot_entity is not None:
            now = system_clock.now()
            tf_future_time = now + Duration(seconds=0.05)
            try:
                pos = robot_entity.get_pos().detach().cpu().numpy().flatten()
                quat_gs = robot_entity.get_quat().detach().cpu().numpy().flatten()
                vel = robot_entity.get_vel().detach().cpu().numpy().flatten()
                ang_vel = robot_entity.get_ang().detach().cpu().numpy().flatten()
            except Exception:
                continue 

            quat_ros = gs_quat_to_ros(quat_gs)

            t = TransformStamped()
            t.header.stamp = tf_future_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base' 
            
            if len(pos) >= 3:
                t.transform.translation.x = float(pos[0])
                t.transform.translation.y = float(pos[1])
                t.transform.translation.z = float(pos[2])
            
            t.transform.rotation.x = float(quat_ros[0])
            t.transform.rotation.y = float(quat_ros[1])
            t.transform.rotation.z = float(quat_ros[2])
            t.transform.rotation.w = float(quat_ros[3])
            
            tf_broadcaster.sendTransform(t)
            odom_msg = Odometry()
            odom_msg.header.stamp = now.to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base"
            
            if len(pos) >= 3:
                odom_msg.pose.pose.position.x = float(pos[0])
                odom_msg.pose.pose.position.y = float(pos[1])
                odom_msg.pose.pose.position.z = float(pos[2])
            
            odom_msg.pose.pose.orientation.x = float(quat_ros[0])
            odom_msg.pose.pose.orientation.y = float(quat_ros[1])
            odom_msg.pose.pose.orientation.z = float(quat_ros[2])
            odom_msg.pose.pose.orientation.w = float(quat_ros[3])
            
            if len(vel) >= 3:
                odom_msg.twist.twist.linear.x = float(vel[0])
                odom_msg.twist.twist.linear.y = float(vel[1])
                odom_msg.twist.twist.linear.z = float(vel[2])
            
            if len(ang_vel) >= 3:
                odom_msg.twist.twist.angular.x = float(ang_vel[0])
                odom_msg.twist.twist.angular.y = float(ang_vel[1])
                odom_msg.twist.twist.angular.z = float(ang_vel[2])
            
            odom_pub.publish(odom_msg)

    rclpy.shutdown()

if __name__=='__main__':
    main()