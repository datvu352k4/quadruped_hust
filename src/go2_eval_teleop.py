import argparse
import os
import pickle
import torch
from go2_env import Go2Env
from rsl_rl.runners import OnPolicyRunner
import numpy as np
import genesis as gs
from pynput import keyboard

# Global variables to store command velocities
lin_x = 0.0
lin_y = 0.0
ang_z = 0.0
base_height = 0.3
stop = False

def on_press(key):
    global lin_x, lin_y, ang_z, stop
    try:
        if key.char == 'w':
            lin_x += 0.1
        elif key.char == 's':
            lin_x -= 0.1
        elif key.char == 'a':
            lin_y += 0.1
        elif key.char == 'd':
            lin_y -= 0.1
        elif key.char == 'q':
            ang_z += 0.1
        elif key.char == 'e':
            ang_z -= 0.1
        elif key.char == '8':
            stop = True
            
        lin_x = np.clip(lin_x, -1.0, 1.0)
        lin_y = np.clip(lin_y, -0.5, 0.5)
        ang_z = np.clip(ang_z, -0.6, 0.6)
            
        # Clear the console
        os.system('clear')
        
        print(f"lin_x: {lin_x:.2f}, lin_y: {lin_y:.2f}, ang_z: {ang_z:.2f}")
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def main():
    global lin_x, lin_y, ang_z, stop
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--exp_name", type=str, default="go2-walking2")
    parser.add_argument("--ckpt", type=int, default=1999)
    parser.add_argument("--save-data", type=bool, default=False)
    args = parser.parse_args()

    gs.init(
        logger_verbose_time = False,
        logging_level="warning",
    )

    log_dir = f"logs/{args.exp_name}"
    env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(f"logs/{args.exp_name}/cfgs.pkl", "rb"))
    # env_cfg, obs_cfg, reward_cfg, command_cfg, train_cfg = pickle.load(open(f"genesis/logs/{args.exp_name}/cfgs.pkl", "rb"))
    reward_cfg["reward_scales"] = {}

    env_cfg["termination_if_roll_greater_than"] =  50  # degree
    env_cfg["termination_if_pitch_greater_than"] = 50  # degree
    num_envs = 1
    env = Go2Env(
        num_envs=num_envs,
        env_cfg=env_cfg,
        obs_cfg=obs_cfg,
        reward_cfg=reward_cfg,
        command_cfg=command_cfg,
        show_viewer=True,
    )
    
    
    runner = OnPolicyRunner(env, train_cfg, log_dir, device="cuda:0")
    resume_path = os.path.join(log_dir, f"model_{args.ckpt}.pt")
    runner.load(resume_path)
    policy = runner.get_inference_policy(device="cuda:0")

    obs, _ = env.reset()
    
    env.commands = torch.tensor([[lin_x, lin_y, ang_z]]).to("cuda:0").repeat(num_envs, 1)
    iter = 0

    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    images_buffer = []
    commands_buffer = []
    with torch.no_grad():
        while not stop:
            
            # env.cam_0.set_pose(lookat=env.base_pos.cpu().numpy()[0],)
            # env.cam_0.set_pose(pos=env.base_pos.cpu().numpy()[0] + np.array([0.5, 0.0, 0.5]) * iter / 50, lookat=env.base_pos.cpu().numpy()[0],)
                
            actions = policy(obs)
            # print(f"toggle_jump: {toggle_jump}, jump_height: {jump_height}")
            env.commands = torch.tensor([[lin_x, lin_y, ang_z]], dtype=torch.float).to("cuda:0").repeat(num_envs, 1)
            obs, _, rews, dones= env.step(actions)
            # print(env.base_pos, env.base_lin_vel)
                    
            iter += 1
          
    if args.save_data:
        # save the images and commands
        images_buffer = np.array(images_buffer)
        commands_buffer = np.array(commands_buffer)
        pickle.dump(images_buffer, open("images_buffer.pkl", "wb"))
        pickle.dump(commands_buffer, open("commands_buffer.pkl", "wb"))

if __name__ == "__main__":
    main()

"""
# evaluation
python examples/locomotion/go2_eval.py -e go2-walking -v --ckpt 100
"""