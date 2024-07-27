import torch
import warp as wp


@wp.kernel
def computeAlphasBody(
    points: wp.array(dtype=wp.vec3),
    a: wp.float32,
    b: wp.float32,
    c: wp.float32,
    d: wp.float32,
    alphas: wp.array(dtype=float),
    d_alpha_d_r: wp.array(dtype=wp.vec3),
    d_alpha_d_q: wp.array(dtype=wp.vec4),
):
    tid = wp.tid()

    p = points[tid]
    x, y, z = p[0], p[1], p[2]

    xa_dm1 = wp.pow(x / a, 2.0 * d - 1.0)
    yb_dm1 = wp.pow(y / b, 2.0 * d - 1.0)
    zc_dm1 = wp.pow(z / c, 2.0 * d - 1.0)

    xa_d = xa_dm1 * x / a
    yb_d = yb_dm1 * y / b
    zc_d = zc_dm1 * z / c

    _alpha = xa_d + yb_d + zc_d
    alphas[tid] = _alpha

    d_alpha_d_x = -2.0 * d * xa_dm1 / a
    d_alpha_d_y = -2.0 * d * yb_dm1 / b
    d_alpha_d_z = -2.0 * d * zc_dm1 / c
    d_alpha_d_r[tid] = wp.vec3(d_alpha_d_x, d_alpha_d_y, d_alpha_d_z)

    d_alpha_d_qw = 8.0 * d * _alpha
    d_alpha_d_qx = -2.0 * z * d_alpha_d_y + 2.0 * y * d_alpha_d_z
    d_alpha_d_qy = 2.0 * z * d_alpha_d_x - 2.0 * x * d_alpha_d_z
    d_alpha_d_qz = -2.0 * y * d_alpha_d_x + 2.0 * x * d_alpha_d_y
    d_alpha_d_q[tid] = wp.vec4(d_alpha_d_qw, d_alpha_d_qx, d_alpha_d_qy, d_alpha_d_qz)


def computeCBF(αs, dαdr, dαdq, β=1.5, δ=0.001):
    _αs = wp.to_torch(αs)
    _cbfs = _αs - β
    min_cbf = torch.min(_cbfs)
    cbfs = _cbfs - min_cbf

    exps = torch.exp(-cbfs / δ)
    expSum = exps.sum()
    h = min_cbf - δ * torch.log(expSum / _αs.shape[0])

    dhdr = (exps.unsqueeze(1) * wp.to_torch(dαdr)).sum(axis=0) / expSum
    dhdq = (exps.unsqueeze(1) * wp.to_torch(dαdq)).sum(axis=0) / expSum
    dhdx = torch.cat([dhdr, dhdq])

    return h, dhdx


class Vessel:
    def __init__(self, a, b, c, device="cpu"):
        self.init_warp(device=device)
        self.setup_ellipsoid(a, b, c)

        self.device = device
        self.n_pcd = None

    def init_warp(self, device="cpu"):
        wp.init()
        wp.set_device(device)

    def setup_ellipsoid(self, a, b, c):
        """
        Set up the ellipsoid

        Args:
            a (float): length of the semi-axis along x
            b (float): length of the semi-axis along y
            c (float): length of the semi-axis along z
        """
        self.P = wp.mat33(
            1 / a**2, 0.0, 0.0, 0.0, 1 / b**2, 0.0, 0.0, 0.0, 1 / c**2
        )
        self.a = a
        self.b = b
        self.c = c

    def compute_alphas_ellipsoid_body(self, pcds, d=1.0):
        """
        Compute the scaling functions for the ellipsoid

        Args:
            pcds (np.array): point cloud data

        Returns:
            np.array: scaling functions
        """
        n_pcd = pcds.shape[0]
        αs = wp.zeros(n_pcd, dtype=float)
        dαdq = wp.zeros(n_pcd, dtype=wp.vec4)
        dαdr = wp.zeros(n_pcd, dtype=wp.vec3)
        points = wp.from_numpy(pcds, wp.vec3, device=self.device)

        wp.launch(
            kernel=computeAlphasBody,
            dim=n_pcd,
            inputs=[points, self.a, self.b, self.c, d, αs, dαdr, dαdq],
        )

        return αs, dαdr, dαdq

    def get_cbfs(self, αs, dαdr, dαdq, β=1.5, δ=0.001):
        cbf, dcbf = computeCBF(αs, dαdr, dαdq, β=β, δ=δ)

        return cbf.cpu().numpy().item(), dcbf.cpu().numpy()
