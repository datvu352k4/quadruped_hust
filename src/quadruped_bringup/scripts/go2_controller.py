import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
import torch
import numpy as np
import pickle
import os

from rsl_rl.runners import OnPolicyRunner

class DummyGo2Env:
    def __init__(self, env_cfg, obs_cfg, device="cpu"):
        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.device = device
        self.num_envs = 1
        self.num_obs = obs_cfg["num_obs"]
        self.num_privileged_obs = None
        self.num_actions = env_cfg["num_actions"]
        
    def get_observations(self):
        obs = torch.zeros((self.num_envs, self.num_obs), device=self.device)
        extras = {}
        extras["observations"] = {}
        extras["observations"]["critic"] = obs 
        return obs, extras
    
    def get_privileged_observations(self):
        return None

    def reset(self):
        return self.get_observations()

class Go2RLController(Node):
    def __init__(self):
        super().__init__('go2_rl_controller')
        
        self.exp_name = "go2-walking"
        self.ckpt = 500
        self.log_dir = f"/home/datvu/quadruped_hust_ws/src/logs/{self.exp_name}" 
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        
        try:
            with open(f"{self.log_dir}/cfgs.pkl", "rb") as f:
                self.env_cfg, self.obs_cfg, self.reward_cfg, self.command_cfg, self.train_cfg = pickle.load(f)
        except FileNotFoundError:
            self.get_logger().error(f"Không tìm thấy file config tại {self.log_dir}/cfgs.pkl")
            raise

        self.obs_scales = self.obs_cfg["obs_scales"]
        self.cmd_scale = torch.tensor(
            [self.obs_scales["lin_vel"], self.obs_scales["lin_vel"], self.obs_scales["ang_vel"]],
            device=self.device, dtype=torch.float32
        )
        self.action_scale = self.env_cfg["action_scale"]
        self.joint_names_rl = self.env_cfg["joint_names"] 
        self.joint_names_bridge = sorted(self.joint_names_rl)

        manual_default_angles = {
            "FL_hip_joint": 0.0, "FR_hip_joint": 0.0, "RL_hip_joint": 0.0, "RR_hip_joint": 0.0,
            "FL_thigh_joint": 0.8, "FR_thigh_joint": 0.8, "RL_thigh_joint": 1.0, "RR_thigh_joint": 1.0,
            "FL_calf_joint": -1.5, "FR_calf_joint": -1.5, "RL_calf_joint": -1.5, "RR_calf_joint": -1.5,
        }
        
        default_pos_list_rl = []
        for name in self.joint_names_rl:
            default_pos_list_rl.append(manual_default_angles[name])
        self.default_dof_pos = torch.tensor(default_pos_list_rl, device=self.device, dtype=torch.float32)

        self.get_logger().info(f"Loading model checkpoint {self.ckpt}...")
        dummy_env = DummyGo2Env(self.env_cfg, self.obs_cfg, device=self.device)
        self.runner = OnPolicyRunner(dummy_env, self.train_cfg, self.log_dir, device=self.device)
        resume_path = os.path.join(self.log_dir, f"model_{self.ckpt}.pt")
        self.runner.load(resume_path)
        self.policy = self.runner.get_inference_policy(device=self.device)
        self.get_logger().info("Model loaded successfully!")

        self.latest_imu = None
        self.latest_joint_state = None
        self.cmd_vel = torch.tensor([0.0, 0.0, 0.0], device=self.device, dtype=torch.float32) 
        
        self.last_actions = torch.zeros(12, device=self.device, dtype=torch.float32)
        self.dof_pos = torch.zeros(12, device=self.device, dtype=torch.float32)
        self.dof_vel = torch.zeros(12, device=self.device, dtype=torch.float32)

        self.reset_steps = 200  
        self.current_step = 0
        self.is_resetting = True 
        self.initial_dof_pos = None 
        
        self.sub_joint = self.create_subscription(JointState, '/go2/joint_states', self.joint_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/go2/imu_topic', self.imu_callback, 10)
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_control = self.create_publisher(JointState, '/go2/joint_state_target', 10)

        self.create_timer(0.02, self.control_loop)

    def joint_callback(self, msg):
        self.latest_joint_state = msg

    def imu_callback(self, msg):
        self.latest_imu = msg

    def cmd_vel_callback(self, msg):
        vx = np.clip(msg.linear.x, -1.0, 1.0)
        vy = np.clip(msg.linear.y, -1.0, 1.0)
        dyaw = np.clip(msg.angular.z, -1.0, 1.0)
        self.cmd_vel = torch.tensor([vx, vy, dyaw], device=self.device, dtype=torch.float32)

    def get_gravity_vector(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        gx = 2 * (x * z - w * y)
        gy = 2 * (y * z + w * x)
        gz = 1 - 2 * (x * x + y * y)
        return torch.tensor([-gx, -gy, -gz], device=self.device, dtype=torch.float32)

    def sort_and_publish(self, target_pos_dict):
        sorted_positions = []
        for name in self.joint_names_bridge: 
            sorted_positions.append(target_pos_dict[name])
        self.publish_cmd(self.joint_names_bridge, sorted_positions)

    def control_loop(self):
        if self.latest_joint_state is None or self.latest_imu is None:
            return

        msg = self.latest_joint_state
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        current_pos_list = []
        current_vel_list = []
        try:
            for joint_name in self.joint_names_rl:
                idx = name_to_idx[joint_name]
                current_pos_list.append(msg.position[idx])
                current_vel_list.append(msg.velocity[idx])
        except KeyError:
            return

        self.dof_pos = torch.tensor(current_pos_list, device=self.device, dtype=torch.float32)
        self.dof_vel = torch.tensor(current_vel_list, device=self.device, dtype=torch.float32)

        if self.is_resetting:
            if self.initial_dof_pos is None:
                self.initial_dof_pos = self.dof_pos.clone()

            self.current_step += 1
            alpha = min(self.current_step / float(self.reset_steps), 1.0)
            
            interpolated_pos = self.initial_dof_pos * (1 - alpha) + self.default_dof_pos * alpha
            
            target_dict = {}
            interpolated_np = interpolated_pos.cpu().numpy()
            for i, name in enumerate(self.joint_names_rl):
                target_dict[name] = interpolated_np[i]
            
            self.sort_and_publish(target_dict)
            
            if self.current_step >= self.reset_steps:
                self.is_resetting = False
                self.get_logger().info("Reset pose done!")
                self.last_actions = torch.zeros(12, device=self.device, dtype=torch.float32)
            return
        
        ang_vel = torch.tensor([
            self.latest_imu.angular_velocity.x,
            self.latest_imu.angular_velocity.y,
            self.latest_imu.angular_velocity.z
        ], device=self.device, dtype=torch.float32)
        
        projected_gravity = self.get_gravity_vector(self.latest_imu.orientation)

        obs_list = [
            ang_vel * self.obs_scales["ang_vel"],
            projected_gravity,
            self.cmd_vel * self.cmd_scale,
            (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],
            self.dof_vel * self.obs_scales["dof_vel"],
            self.last_actions
        ]
        obs = torch.cat(obs_list, dim=-1).unsqueeze(0)

        with torch.no_grad():
            actions = self.policy(obs)
        
        self.last_actions = actions[0]

        target_dof_pos_rl = (actions[0] * self.action_scale + self.default_dof_pos).cpu().numpy()

        target_dict = {}
        for i, name in enumerate(self.joint_names_rl):
            target_dict[name] = target_dof_pos_rl[i]
            
        self.sort_and_publish(target_dict)

    def publish_cmd(self, names, positions):
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = names
        
        if isinstance(positions, torch.Tensor):
            positions = positions.cpu().numpy()
        
        if isinstance(positions, np.ndarray):
            positions = positions.tolist()
            
        positions = [float(p) for p in positions]
            
        cmd_msg.position = positions
        cmd_msg.velocity = []
        cmd_msg.effort = []
        self.pub_control.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Go2RLController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()