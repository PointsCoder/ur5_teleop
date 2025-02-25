import datetime
import glob
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple
from multiprocessing import Process

import numpy as np
import tyro

from gello.data_utils.format_obs import save_frame
from gello.env import RobotEnv
from gello.robots.robot import PrintRobot
from gello.zmq_core.robot_node import ZMQClientRobot
# from gello.zmq_core.camera_node import ZMQServerCamera
from gello.zmq_core.camera_node import ZMQClientCamera

from gello.cameras.realsense_camera import RealSenseCamera, get_device_ids
from gello.zmq_core.camera_node import ZMQServerCamera

# diffusion policy inference
from diffusion_policy.wrap_policy import get_workspace, preprocess

def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


@dataclass
class Args:
    agent: str = "none"
    robot_port: int = 6002
    wrist_camera_port: int = 5001
    base_camera_port: int = 5002
    hostname: str = "127.0.0.1"
    robot_type: str = None  # only needed for quest agent or spacemouse agent
    hz: int = 100
    start_joints: Optional[Tuple[float, ...]] = None

    gello_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False
    camera_ids = get_device_ids()

def main(args):
    if args.mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {
            # you can optionally add camera nodes here for imitation learning purposes
            # "wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
            "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
            # "base": ZMQServerCamera(camera=RealSenseCamera(args.camera_ids[0]), port=args.base_camera_port, host=args.hostname)
        }
        robot_client = ZMQClientRobot(port=args.robot_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=args.hz, camera_dict=camera_clients)

    # going to start position
    print("Going to start position")
    obs = env.get_obs()
    joints = obs["joint_positions"]

    start_pos = np.deg2rad(
        [0, -90, 90, -90, -90, 0, 0]
    )
    # start_pos = np.array([0.06927531, -1.6276184 ,  1.52793462, -1.56947624, -1.70781208, 0.07917938, 0])
    abs_deltas = np.abs(start_pos - joints)
    id_max_joint_delta = np.argmax(abs_deltas)

    max_joint_delta = 0.8
    if abs_deltas[id_max_joint_delta] > max_joint_delta:
        id_mask = abs_deltas > max_joint_delta
        print()
        ids = np.arange(len(id_mask))[id_mask]
        for i, delta, joint, current_j in zip(
            ids,
            abs_deltas[id_mask],
            start_pos[id_mask],
            joints[id_mask],
        ):
            print(
                f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
            )
        return

    print(f"Start pos: {len(start_pos)}", f"Joints: {len(joints)}")
    assert len(start_pos) == len(
        joints
    ), f"agent output dim = {len(start_pos)}, but env dim = {len(joints)}"
    print("Move to the start position")
    max_delta = 0.05
    for _ in range(20):
        obs = env.get_obs()
        current_joints = obs["joint_positions"]
        delta = start_pos - current_joints
        max_joint_delta = np.abs(delta).max()
        if max_joint_delta > max_delta:
            delta = delta / max_joint_delta * max_delta
            env.step(current_joints + delta)
        else:
            break
    print("reached start position")
    
    print("import policy network")

    workspace = get_workspace()
    obs = env.get_obs()
    joint_positions = obs["joint_positions"]
    base_rgb = obs["base_rgb"]

    HISTORY_LENGTH = 6

    data_sample = [{'joint_positions': joint_positions, 'base_rgb': base_rgb} for _ in range(HISTORY_LENGTH)]
    print("Data: ", data_sample)

    # print("Start policy inference")
    # while True:
    #     data = preprocess(data_sample)
    #     pred_action = workspace.predict_action(data)

    #     pred_action = pred_action.cpu().numpy()
    #     pred_action = np.array(pred_action)
    #     print(f"Action: {pred_action}")
    #     # assert action is np.ndarray, "action is not numpy array"

    #     for i in range(HISTORY_LENGTH):
    #         action = pred_action[i]
    #         env.step(action)
    #         if (action[:6] - joint_positions[:6] > 2.0).any():
    #             print("Action is too big")

    #             # print which joints are too big
    #             joint_index = np.where(action[:6] - joint_positions[:6] > 0.5)
    #             for j in joint_index:
    #                 print(
    #                     f"Joint [{j}], leader: {action[j]}, follower: {joint_positions[j]}, diff: {action[j] - joint_positions[j]}"
    #                 )
    #             exit()
  
    #         obs = env.get_obs()
    #         joint_positions = obs["joint_positions"]
    #         base_rgb = obs["base_rgb"]

    #         data_sample = data_sample[1:]
    #         # import pdb; pdb.set_trace()
    #         new_data = {'joint_positions': joint_positions, 'base_rgb': base_rgb}
    #         data_sample.append(new_data)
    #         assert len(data_sample) == HISTORY_LENGTH
    #         print("Data: ", data_sample[-1])

    print("Start policy inference")
    while True:
        data = preprocess(data_sample)
        pred_action = workspace.predict_action(data)

        pred_action = pred_action.cpu().numpy()
        pred_action = np.array(pred_action)
        print(f"Action: {pred_action}")
        # assert action is np.ndarray, "action is not numpy array"

        action = pred_action[0]
        env.step(action)
        if (action[:6] - joint_positions[:6] > 0.5).any():
            print("Action is too big")
            # print which joints are too big
            joint_index = np.where(action[:6] - joint_positions[:6] > 0.5)
            for j in joint_index:
                print(
                    f"Joint [{j}], leader: {action[j]}, follower: {joint_positions[j]}, diff: {action[j] - joint_positions[j]}"
                )
            exit()

        obs = env.get_obs()
        joint_positions = obs["joint_positions"]
        base_rgb = obs["base_rgb"]
        data_sample = data_sample[1:]
        # import pdb; pdb.set_trace()
        new_data = {'joint_positions': joint_positions, 'base_rgb': base_rgb}
        data_sample.append(new_data)
        assert len(data_sample) == HISTORY_LENGTH
        print("Data: ", data_sample[-1])

if __name__ == "__main__":
    main(tyro.cli(Args))
