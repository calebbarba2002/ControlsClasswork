# 3rd-party
import numpy as np
from numpy.typing import NDArray
import control as cnt

# local (controlbook)
from ..common.numeric_integration import rk4_step

class Observer:
    def __init__(
        self,
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        C: NDArray[np.float64],
        des_poles: NDArray,
        ts: float,
        print_gain: bool = False,
    ):
        self.A = A
        self.B = B
        self.C = C
        self.ts = ts

        dim_state, dim_input = B.shape

        observability_mat = cnt.ctrb(A.T, C.T)
        if np.linalg.matrix_rank(observability_mat) != dim_state:
            raise ValueError("System is not observable")
        self.L = cnt.place(A.T, C.T, des_poles).T
        if print_gain:
            print(f"L^T =", self.L.T)
        self.xhat = np.zeros(dim_state)
        self.u_prev = np.zeros(dim_input)

    # TODO: decide whether to reimplement RK4 or import it like this
    def update(
        self, y: NDArray[np.float64], u_prev: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        self.u_prev = u_prev
        self.xhat = rk4_step(self.observer_f, self.xhat, y, self.ts)
        return self.xhat
    
    def observer_f(
        self, xhat: NDArray[np.float64], y: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        y_error = y - self.C @ xhat
        xhat_dot = self.A @ xhat + self.B @ self.u_prev + self.L @ y_error
        return xhat_dot

    # TODO: decide whether to include linearization info in observer
    # (the code below)

    # def update(self, y: NDArray[np.float64]) -> NDArray[np.float64]:
    #   self.xhat = rk4_step(self.observer_f, self.xhat, y, self.ts)
    #   return self.xhat.copy()
    #
    # def observer_f(
    #   self, xhat: NDArray[np.float64], y: NDArray[np.float64]
    #  ) -> NDArray[np.float64]:
    #   xhat_tilde = xhat - self.x_eq
    #   u_tilde = self.u_prev - self.u_eq
    #   yhat_tilde = y - self.C @ xhat
    #   xhat_dot = self.A @ xhat_tilde + self.B @ u_tilde + self.L @ yhat_tilde
    #   return xhat_dot
    #
    # def set_previous_input(self, u: NDArray[np.float64]) -> None:
    #   self.u_prev = u.copy()