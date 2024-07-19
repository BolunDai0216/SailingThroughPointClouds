import numpy as np
import warp as wp
from rich.console import Console


@wp.kernel
def computeArmAlphasHighOrderEllipsoid(
    points: wp.array(dtype=wp.vec3),
    Rs: wp.array(dtype=wp.mat33),
    a: wp.float32,
    b: wp.float32,
    c: wp.float32,
    alphas: wp.array2d(dtype=wp.float32),
    order: wp.float32,
):
    # get the thread id
    arm_index, tid = wp.tid()

    # compute the scaling functions
    point = points[tid]
    RT = Rs[arm_index]
    point_rotated = RT * point
    x, y, z = point_rotated[0], point_rotated[1], point_rotated[2]

    if wp.abs(y) < b and wp.abs(z) < c and x > 0.0:
        d = wp.pow(1.0 - wp.pow(y / b, order) - wp.pow(z / c, order), 1.0 / order)
        if d >= 0.0:
            s = x / ((1.0 + d) * a)
            alphas[arm_index, tid] = s


class Mariner:
    def __init__(
        self,
        a=1.0,
        b=0.2,
        c=0.2,
        n_ellipse=100,
        max_scale=2.0,
        order=8,
        angle_lim=7 / 8,
    ):
        self.n_ellipse = n_ellipse
        self.setup_ellipsoid(a, b, c)
        self.console = Console()
        self.thetas = np.linspace(-angle_lim * np.pi, angle_lim * np.pi, self.n_ellipse)
        self.max_scale = max_scale
        self.order = order

        Rs = np.stack(
            [
                np.array(
                    [
                        [np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta), 0],
                        [0, 0, 1],
                    ]
                ).T
                for theta in self.thetas.tolist()
            ]
        )
        self.Rs_wp = wp.from_numpy(Rs, dtype=wp.mat33)
        self.ts = np.stack(
            [
                2 * np.array([a * np.cos(theta), a * np.sin(theta), 0])
                for theta in self.thetas.tolist()
            ]
        )

    def setup_ellipsoid(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def jit_run(self, points, target):
        with self.console.status(
            "[bold green]Working PreviewController JIT Run..."
        ) as status:
            self.controller(points, target)
            self.console.log("Finished PreviewController JIT run")

    def controller(self, points, target, return_info=False):
        target[2] = 0.0
        self.n_points = points.shape[0]

        points_wp = wp.array(points, dtype=wp.vec3)
        alphas = wp.from_numpy(
            self.max_scale * np.ones((self.n_ellipse, self.n_points)), dtype=float
        )

        wp.launch(
            kernel=computeArmAlphasHighOrderEllipsoid,
            dim=(self.n_ellipse, self.n_points),
            inputs=[points_wp, self.Rs_wp, self.a, self.b, self.c, alphas, self.order],
        )

        scales = np.min(alphas.numpy(), axis=1)[:, np.newaxis]
        scaled_tips = self.ts * scales
        invalid_scales = scales < 0.5
        distances = (
            np.linalg.norm(scaled_tips - target.reshape(1, 3), axis=1)
            + invalid_scales[:, 0] * 1e6
        )
        min_idx = np.argmin(distances)
        min_distance = distances[min_idx]
        min_theta = self.thetas[min_idx]
        preview_target = scaled_tips[min_idx, :]
        chosen_scale = scales[min_idx]

        if return_info:
            info = {
                "min_theta": min_theta,
                "preview_target": preview_target,
                "min_distance": min_distance,
                "scales": scales,
                "scaled_tips": scaled_tips,
                "distances": distances,
            }

            return info

        return min_theta, preview_target, min_distance, chosen_scale
