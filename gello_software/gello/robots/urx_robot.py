from typing import Dict
import time
import numpy as np

from gello.robots.robot import Robot
import urx as urx


class URXRobot(Robot):
    """A class representing a UR robot."""

    def __init__(self, robot_ip: str = "192.10.0.11", no_gripper: bool = False):
        # import rtde_control
        # import rtde_receive

        [print("in ur robot") for _ in range(4)]
        try:
            # self.robot = rtde_control.RTDEControlInterface(robot_ip)
            self.robot = urx.Robot(robot_ip)
            # print('done')
        except Exception as e:
            print(e)
            print(robot_ip)

        # self.r_inter = rtde_receive.RTDEReceiveInterface(robot_ip)
        if not no_gripper:
            # from gello.robots.robotiq_gripper import RobotiqGripper

            # self.gripper = RobotiqGripper()
            # self.gripper.connect(hostname="192.168.1.11", port=63352)
            # self.gripper.connect(hostname="192.10.0.11", port=63352)

            # from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper
            # self.gripper = Robotiq_Two_Finger_Gripper(self.robot, socket_host="192.168.1.11")
            # self.gripper.get_current_position()

            # print("close gripper")
            # self.gripper.close_gripper()
            # print("open gripper")
            # self.gripper.open_gripper()
            print("gripper test")
            # self.gripper.gripper_action(128)
            print("close gripper")
            self.robot.set_digital_out(8, True)
            self.robot.set_digital_out(9, False)
            dout0 = self.robot.get_digital_out(8)
            dout1 = self.robot.get_digital_out(9)
            print(f'Dout0 {dout0} Dout1 {dout1}')
            time.sleep(0.05)
            print("open gripper")
            self.robot.set_digital_out(8, False)
            self.robot.set_digital_out(9, False)
            dout0 = self.robot.get_digital_out(8)
            dout1 = self.robot.get_digital_out(9)
            print(f'Dout0 {dout0} Dout1 {dout1}')
            time.sleep(0.05)
            # self.gripper.open_gripper()
            # left_v = self.robot.get_analog_in(nb=8)
            # right_v = self.robot.get_analog_in(nb=9)
            # print(f"left gripper: {left_v}, right gripper: {right_v}")
            print("gripper connected")
            # gripper.activate()
            # pass


        [print("connect") for _ in range(4)]

        self._free_drive = False
        # self.robot.set_freedrive(True)
        # self.robot.endFreedriveMode()
        self._use_gripper = not no_gripper

    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            return 7
        return 6

    def _get_gripper_pos(self) -> float:
        # time.sleep(0.01)
        # gripper_pos = self.gripper.get_current_position()
        # gripper_pos = 0
        # assert 0 <= gripper_pos <= 255, "Gripper position must be between 0 and 255"
        dout0 = self.robot.get_digital_out(8)
        dout1 = self.robot.get_digital_out(9)
        if dout0 > 0.5 or dout1 > 0.5:
            gripper_pos = 1.0 # close
        else:
            gripper_pos = 0.0 # open
        assert 0 <= gripper_pos <= 1, "Gripper position must be between 0 and 1"
        return gripper_pos

    def get_joint_state(self) -> np.ndarray:
        """Get the current state of the leader robot.

        Returns:
            T: The current state of the leader robot.
        """
        # robot_joints = self.r_inter.getActualQ()
        robot_joints = np.array(self.robot.getj()) 
        if self._use_gripper:
            gripper_pos = self._get_gripper_pos()
            pos = np.append(robot_joints, gripper_pos)
        else:
            pos = robot_joints
        return pos

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the leader robot to a given state.

        Args:
            joint_state (np.ndarray): The state to command the leader robot to.
        """
        velocity = 0.5 
        acceleration = 0.5
        dt = 1.0 / 500  # 2ms
        lookahead_time = 0.2
        gain = 100
        t=0.5

        # velocity = 0.1
        # acceleration = 0.1
        # dt = 1.0 / 500  # 2ms
        # lookahead_time = 0.2
        # gain = 100
        # t=0.5

        robot_joints = joint_state[:6]
        # t_start = self.robot.initPeriod()
        # t_start = time.time()

        # gripper control
        if self._use_gripper:
            gripper_pos = joint_state[-1] # range [0, 1]
            if gripper_pos > 0.5: # close
                self.robot.set_digital_out(8, True)
                self.robot.set_digital_out(9, False)
            else: # open
                self.robot.set_digital_out(8, False)
                self.robot.set_digital_out(9, False)
        
        # arm control
        self.robot.servoj_robot(
            # robot_joints, velocity, acceleration, t, lookahead_time, gain, wait=False,
            robot_joints, velocity, acceleration, t, lookahead_time, gain, wait=True,
        )
        # self.robot.servoj_robot(
        #     tjoints=robot_joints, acc=0.01, vel=0.01, t=0.1, lookahead_time=0.2, gain=100, wait=False, relative=False, threshold=None
        # )
        # self.robot.set_pos(
        #     robot_joints,  acceleration, velocity
        # )
        print("commanded joint state")
        # self.robot.waitPeriod(t_start)
        # elapsed_time = time.time() - t_start
        # while elapsed_time < dt:
        #     time.sleep(0.001)  # Sleep for a short time (1 ms) to avoid busy-waiting
        #     elapsed_time = time.time() - t_start


    def freedrive_enabled(self) -> bool:
        """Check if the robot is in freedrive mode.

        Returns:
            bool: True if the robot is in freedrive mode, False otherwise.
        """
        return self._free_drive

    def set_freedrive_mode(self, enable: bool) -> None:
        """Set the freedrive mode of the robot.

        Args:
            enable (bool): True to enable freedrive mode, False to disable it.
        """
        if enable and not self._free_drive:
            self._free_drive = True
            self.robot.set_freedrive(True)
        elif not enable and self._free_drive:
            self._free_drive = False
            self.robot.set_freedrive(False)

    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.get_joint_state()
        pos_quat = np.zeros(7)
        if self._use_gripper:
            gripper_pos = np.array([joints[-1]])
        else:
            gripper_pos = np.zeros(1)
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }


def main():
    robot_ip = "192.10.0.11"
    ur = URXRobot(robot_ip, no_gripper=False)
    print(ur)
    ur.set_freedrive_mode(True)
    print(ur.get_observations())
    print('done')

if __name__ == "__main__":
    main()
