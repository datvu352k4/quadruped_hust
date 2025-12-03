from gs_ros import GsRosBridge
import rclpy
from rclpy.node import Node
import genesis as gs
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
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
    print("--- STARTING GO2 IMPORT ---")
    gs.init(logging_level="warning", performance_mode=True)
    rclpy.init(args=args)
    
    system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
    default_ros_node = Node('gs_ros_bridge_node')
    
    tf_broadcaster = TransformBroadcaster(default_ros_node)
    odom_pub = default_ros_node.create_publisher(Odometry, '/odom', 10)
    scan_pub = default_ros_node.create_publisher(LaserScan, '/scan', 10)
    joint_pub = default_ros_node.create_publisher(JointState, '/go2/joint_states_custom', 10)
    joint_names = [
        "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
        "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
        "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint"
    ]

    def scan_callback(msg):
        msg.header.stamp = system_clock.now().to_msg()
        scan_pub.publish(msg)

    default_ros_node.create_subscription(LaserScan, '/go2/laser_scan_topic_raw', scan_callback, 10)

    gs_ros_bridge = GsRosBridge(default_ros_node, "/home/datvu/quadruped_hust_ws/src/go2.yaml", add_debug_objects=True)
    gs_ros_bridge.build()
    
    robot_entity = None
    if len(gs_ros_bridge.robots) > 0:
        robot_entity = gs_ros_bridge.robots[0][2]
        print(f"--- ROBOT ENTITY FOUND: {robot_entity} ---")

    step_count = 0
    
    while rclpy.ok():
        rclpy.spin_once(default_ros_node, timeout_sec=0)
        gs_ros_bridge.step()
        step_count += 1
        
        if robot_entity is None:
            continue

        now = system_clock.now()
        now_msg = now.to_msg()

        future_now = now + Duration(seconds=0.1) 
        future_now_msg = future_now.to_msg()

        try:
            pos = robot_entity.get_pos().detach().cpu().numpy().flatten()
            quat_gs = robot_entity.get_quat().detach().cpu().numpy().flatten()
            vel = robot_entity.get_vel().detach().cpu().numpy().flatten()
            ang_vel = robot_entity.get_ang().detach().cpu().numpy().flatten()
            
            dofs_pos = robot_entity.get_dofs_position().detach().cpu().numpy().flatten()
            dofs_vel = robot_entity.get_dofs_velocity().detach().cpu().numpy().flatten() 
            dofs_force = robot_entity.get_dofs_force().detach().cpu().numpy().flatten() 
            
        except Exception:
            continue

        quat_ros = gs_quat_to_ros(quat_gs)

        js_msg = JointState()
        js_msg.header.stamp = now_msg
        js_msg.name = joint_names
        if len(dofs_pos) >= 12:
            # Position
            raw_pos = dofs_pos[-12:]
            js_msg.position = [float(p) for p in raw_pos]
            if len(dofs_vel) >= 12:
                raw_vel = dofs_vel[-12:]
                js_msg.velocity = [float(v) for v in raw_vel]
            if len(dofs_force) >= 12:
                raw_eff = dofs_force[-12:]
                js_msg.effort = [float(e) for e in raw_eff]
            
            joint_pub.publish(js_msg)
        
        t = TransformStamped()
        t.header.stamp = future_now_msg
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
        odom_msg.header.stamp = now_msg
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base"
        
        if len(pos) >= 3:
            odom_msg.pose.pose.position.x = float(pos[0])
            odom_msg.pose.pose.position.y = float(pos[1])
            odom_msg.pose.pose.position.z = float(pos[2])
        
        odom_msg.pose.pose.orientation = t.transform.rotation
        
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