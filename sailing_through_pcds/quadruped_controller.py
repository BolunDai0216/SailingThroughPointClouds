import numpy as np
from rich.console import Console

from pcd_cbf.cbf_solver import CBFQPSolver
from pcd_cbf.vessel import Vessel


class QuadrupedController:
    def __init__(self, a=0.8, b=0.4, c=0.4):
        self.solver = CBFQPSolver(3, n_ieq=1, warm_start=False)
        self.console = Console()

        # F from \dot{x} = Fu
        F_mat = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )

        # compute Q matrix
        Q_mat = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        self.QF = Q_mat @ F_mat
        self.H_unicycle = np.array([[0.0, 0, 0], [0, 1.0, 0], [0, 0, 2.0]])

        self.cbf_warp = Vessel(a, b, c)

    def jit_run(self, points):
        """
        Run CBF-QP solver once to JIT compile.

        Args:
            points (np.ndarray): point cloud in body frame
        """
        with self.console.status("[bold green]Working JIT Run...") as status:
            _, _ = self.get_cbf(points)
            self.console.log("Finished JIT run")

    def get_cbf(self, points):
        αs, dαdr, dαdq = self.cbf_warp.compute_alphas_ellipsoid_body(points)
        cbf, dcbf = self.cbf_warp.get_cbfs(αs, dαdr, dαdq, β=1.5, δ=0.00001)

        return cbf, dcbf

    def get_control(self, ref_cmd, cbf, dcbf, gamma=2.0):
        """
        Get safe control action from CBF-QP.

        Args:
            ref_cmd (np.ndarray): reference command (vx, vy, wz)
            cbf (float): CBF value
            dcbf (np.ndarray): CBF gradient

        Returns:
            safe_cmd (np.ndarray): safe command (vx, vy, wz)
        """
        H = 2 * (np.eye(3) + 0.1 * self.H_unicycle)
        g = -2 * ref_cmd
        C = dcbf[np.newaxis, :] @ self.QF
        lb = -gamma * np.array([[cbf]])
        params = {"H": H, "g": g, "C": C, "lb": lb}

        # solve CBFQP
        self.solver.solve(params)
        safe_cmd = self.solver.qp.results.x

        return safe_cmd