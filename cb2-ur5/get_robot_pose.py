import urx_local
import time

if __name__ == '__main__':
    robot_left = urx_local.Robot("192.10.0.11")
    trans = robot_left.get_pose()
    print('Left robot pose: ', trans)
    time.sleep(1)

    # robot_right = urx_local.Robot("192.10.0.12")
    # trans = robot_right.get_pose()
    # print('Right robot pose: ', trans)
    print('Done')