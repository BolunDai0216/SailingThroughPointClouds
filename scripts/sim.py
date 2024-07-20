import argparse
import pickle
import signal
import sys
import time
from copy import deepcopy

import numpy as np
from Go2Py.sim.mujoco import Go2Sim
from pcd_cbf.compass import Compass
from PIL import Image
from quadruped_controller import QuadrupedController
from utils.exp_utils import clip_velocity, get_performance_controller, world2body


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=str, default="00")
    parser.add_argument(
        "--mapdir", type=str, default="/home/pcd-cbf/assets/height_map_img/"
    )
    args = parser.parse_args()

    map_name = f"map0{args.id}"
    robot = Go2Sim(mode="highlevel", xml_path="../assets/mujoco/Go2/go2.xml")
    image = Image.open(f"{args.mapdir}{map_name}.png").convert("L")
    image_np = np.array(image).reshape(-1) / 255
    robot.model.hfield_data = 2.0 * image_np
    robot.viewer.update_hfield(0)
    robot.pos0[0:2] = np.array([-4.0, -4.0])
    robot.reset()

    p_target_world = np.array([6.0, 6.0, 0.0])

    controller = QuadrupedController(a=0.5, b=0.3, c=0.2)  # with vessel
    preview_controller = Compass(a=0.8, b=0.1, c=0.2, order=4)

    # Define the signal handler function
    def signal_handler(sig, frame):
        robot.close()
        sys.exit(0)

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # JIT run preview_target_body
    lidar_points = robot.getLaserScan(max_range=10.0)["pcd"]
    controller.jit_run(lidar_points)
    preview_controller.jit_run(lidar_points, p_target_world)

    preview_arm_counter = 0
    done = False
    robot_positions = []
    success = False

    for i in range(15000):
        if done:
            break
        p_robot_world, _quat_robot_world = robot.getPose()
        robot_positions.append(deepcopy(p_robot_world))
        quat_robot_world = np.array(
            [
                _quat_robot_world[1],
                _quat_robot_world[2],
                _quat_robot_world[3],
                _quat_robot_world[0],
            ]
        )

        p_target_body = world2body(
            p_target_world, p_robot_world, quat_robot_world, SLAM=True
        )
        robot_positions.append(deepcopy(p_robot_world))

        if i % 100 == 0:
            lidar_points = robot.getLaserScan(max_range=10.0)["pcd"]
            cbf, dcbf = controller.get_cbf(lidar_points)
            if preview_arm_counter == 0:
                (
                    arm_q,
                    preview_target_body,
                    distance,
                    chosen_scale,
                ) = preview_controller.controller(lidar_points, p_target_body)

            preview_arm_counter = (preview_arm_counter + 1) % 4

            if np.isnan(cbf):
                print("NaNs in CBF")
                break

            v_des, done = get_performance_controller(
                np.zeros(2), preview_target_body[:2], 0.0, kv=0.5, kw=1.0, v_max=0.4
            )
            v_cmd = clip_velocity(v_des[:2], 0.4)
            w_cmd = np.clip(v_des[2], -1.0, 1.0)
            vel = np.array([v_cmd[0], v_cmd[1], w_cmd])
            safe_vel = controller.get_control(vel, cbf, dcbf)
            local_planner_overall_comp_times.append(
                (time.time() - cbf_tic) / lidar_points.shape[0]
            )

        if i <= 20:
            robot.step(0.0, 0.0, 0.0)
        else:
            robot.step(
                safe_vel[0],
                safe_vel[1],
                safe_vel[2],
                kp=[0, 0.0, 0.0],
                ki=[0.0, 0.0, 0.0],
            )

        error = np.linalg.norm(
            np.array([robot.data.qpos[0], robot.data.qpos[1]]) - p_target_world[:2]
        )
        if error < 0.5:
            success = True
            break

    robot.viewer.close()


if __name__ == "__main__":
    main()
