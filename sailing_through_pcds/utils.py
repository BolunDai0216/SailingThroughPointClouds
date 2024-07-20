import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from matplotlib.patches import Ellipse
from scipy.spatial.transform import Rotation


def clip_velocity(v, v_max):
    norm_v = np.linalg.norm(v)

    if norm_v > v_max:
        v_clipped = v_max * v / norm_v
    else:
        v_clipped = v

    return v_clipped


def get_performance_controller(
    pos, pos_des, yaw_angle, kv=0.5, kw=1.0, v_max=0.4, stop_at_goal=True
):
    R = np.array(
        [
            [np.cos(yaw_angle), -np.sin(yaw_angle)],
            [np.sin(yaw_angle), np.cos(yaw_angle)],
        ]
    )

    p_error = pos_des - pos
    _v = kv * R.T @ p_error[:, np.newaxis]
    v = clip_velocity(_v, v_max)

    θ_des = np.arctan2(p_error[1], p_error[0])

    ω = kw * get_yaw_error(yaw_angle, θ_des)
    v_ref = np.append(v, ω)
    done = False

    if stop_at_goal and np.linalg.norm(p_error) <= 0.01:
        v_ref = np.zeros(3)
        done = True

    return v_ref, done


def get_yaw_error(yaw, yaw_des):
    # R1 = Rotation.from_euler("z", yaw).as_matrix()
    # R2 = Rotation.from_euler("z", yaw_des).as_matrix()
    # R_error = Rotation.from_matrix(R1.T @ R2)
    # yaw_error = R_error.as_euler("zyx")[0]
    # breakpoint()
    R1 = pin.rpy.rpyToMatrix(0, 0, yaw)
    R2 = pin.rpy.rpyToMatrix(0, 0, yaw_des)
    R_error = R1.T @ R2
    yaw_error = pin.rpy.matrixToRpy(R_error)[2]

    return yaw_error


def plot_lidar_data(lidar_data, pose):
    fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    ax.scatter(lidar_data[:, 0], lidar_data[:, 1], color="darkorange")
    ax.set_aspect("equal")

    rpy = pin.rpy.matrixToRpy(pose[:3, :3])
    _ellipse = Ellipse(
        (pose[0, -1], pose[1, -1]),
        1.6,
        0.8,
        angle=np.rad2deg(rpy[-1]),
        facecolor="lightsteelblue",
        edgecolor="cornflowerblue",
        linewidth=2,
        zorder=10,
    )
    ax.add_patch(_ellipse)

    plt.show()


def world2body(p_target_world, p_robot_world, quat_robot_world):
    """
    Transform a point in world frame to robot body frame

    Args:
        p_target_world (np.ndarray): Target point in world frame
        p_robot_world (np.ndarray): Robot position in world frame
        quat_robot_world (np.ndarray): Robot orientation in world frame

    Returns:
        p_target_body (np.ndarray): Target point in robot body frame
    """
    T_WB = pin.SE3(pin.Quaternion(quat_robot_world), p_robot_world)
    T_BW = T_WB.inverse()
    p_target_body = T_BW.act(p_target_world)

    return p_target_body
