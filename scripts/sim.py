import pickle
import signal
import sys
import time

import hydra
import numpy as np
from Go2Py.sim.mujoco import Go2Sim
from omegaconf import DictConfig, OmegaConf
from PIL import Image
from sailing_through_pcds.mariner import Mariner
from sailing_through_pcds.quadruped_controller import QuadrupedController
from sailing_through_pcds.utils import (
    clip_velocity,
    get_performance_controller,
    world2body,
)


@hydra.main(version_base=None, config_path="conf", config_name="config")
def main(cfg: DictConfig):
    map_dir = cfg["map_dir"]
    map_id = cfg["map_id"]

    map_name = f"map0{map_id}"
    robot = Go2Sim(mode="highlevel", xml_path="../assets/go2.xml")
    image = Image.open(f"{map_dir}{map_name}.png").convert("L")
    image_np = np.array(image).reshape(-1) / 255
    robot.model.hfield_data = 2.0 * image_np
    robot.viewer.update_hfield(0)
    robot.pos0[0:2] = np.array(cfg["robot_start_pos"][:2])
    robot.reset()

    p_target_world = np.array(cfg["robot_goal_pos"])
    controller = QuadrupedController(
        a=cfg["vessel_a"], b=cfg["vessel_b"], c=cfg["vessel_c"]
    )
    preview_controller = Mariner(
        a=cfg["mariner_a"],
        b=cfg["mariner_b"],
        c=cfg["mariner_c"],
        order=cfg["mariner_d"],
    )

    # Define the signal handler function
    def signal_handler(sig, frame):
        robot.close()
        image.close()
        sys.exit(0)

    # Register the signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # JIT run preview_target_body
    lidar_points = robot.getLaserScan(max_range=cfg["lidar_max_range"])["pcd"]
    controller.jit_run(lidar_points)
    preview_controller.jit_run(lidar_points, p_target_world)

    preview_arm_counter = 0
    done = False
    success = False

    for i in range(cfg["num_iters"]):
        if done:
            break
        p_robot_world, _quat_robot_world = robot.getPose()
        quat_robot_world = np.array(
            [
                _quat_robot_world[1],
                _quat_robot_world[2],
                _quat_robot_world[3],
                _quat_robot_world[0],
            ]
        )

        p_target_body = world2body(p_target_world, p_robot_world, quat_robot_world)

        if i % cfg["safe_vel_update_freq"] == 0:
            lidar_points = robot.getLaserScan(max_range=cfg["lidar_max_range"])["pcd"]
            cbf, dcbf = controller.get_cbf(lidar_points)
            if preview_arm_counter == 0:
                (
                    arm_q,
                    preview_target_body,
                    distance,
                    chosen_scale,
                ) = preview_controller.controller(lidar_points, p_target_body)

            preview_arm_counter = (preview_arm_counter + 1) % cfg["mariner_update_freq"]

            if np.isnan(cbf):
                print("NaNs in CBF")
                break

            yaw_angle_in_body_frame = 0.0
            v_des, done = get_performance_controller(
                np.zeros(2),
                preview_target_body[:2],
                yaw_angle_in_body_frame,
                kv=cfg["performance_controller_kv"],
                kw=cfg["performance_controller_kw"],
                v_max=cfg["performance_controller_v_max"],
            )
            v_cmd = clip_velocity(v_des[:2], cfg["performance_controller_v_max"])
            w_cmd = np.clip(
                v_des[2],
                -cfg["performance_controller_w_max"],
                cfg["performance_controller_w_max"],
            )
            vel = np.array([v_cmd[0], v_cmd[1], w_cmd])
            safe_vel = controller.get_control(vel, cbf, dcbf)

        if i <= cfg["start_delay"]:
            robot.step(0.0, 0.0, 0.0)
        else:
            robot.step(
                safe_vel[0],
                safe_vel[1],
                safe_vel[2],
                kp=[0.0, 0.0, 0.0],
                ki=[0.0, 0.0, 0.0],
            )

        error = np.linalg.norm(
            np.array([robot.data.qpos[0], robot.data.qpos[1]]) - p_target_world[:2]
        )
        if error < cfg["success_distance"]:
            success = True
            break


if __name__ == "__main__":
    main()
