import proxsuite


class CBFQPSolver:
    def __init__(self, n_var, n_ieq=1, warm_start=False):
        self.n = n_var  # number of decision variables
        self.n_eq = 0  # number of equality constraints
        self.n_ieq = n_ieq  # number of inequality constraints
        self.qp = proxsuite.proxqp.dense.QP(self.n, self.n_eq, self.n_ieq)
        self.initialized = False
        self.warm_start = warm_start

    def solve(self, params):
        self.H, self.g, self.C, self.lb = self.compute_params(params)

        if not self.initialized:
            # self.qp.init(H=self.H, g=self.g)
            self.qp.init(H=self.H, g=self.g, C=self.C, l=self.lb)

            self.qp.settings.eps_abs = 1.0e-6
            self.qp.settings.max_iter = 20
            self.initialized = True
        else:
            if self.warm_start:
                self.qp.settings.initial_guess = (
                    proxsuite.proxqp.InitialGuess.WARM_START_WITH_PREVIOUS_RESULT
                )

            # self.qp.update(H=self.H, g=self.g)
            self.qp.update(H=self.H, g=self.g, C=self.C, l=self.lb)

        self.qp.solve()

    def compute_params(self, params):
        H = params["H"]
        g = params["g"]
        C = params["C"]
        lb = params["lb"]

        return H, g, C, lb
