# 3rd-party
import numpy as np
import control as ctrl
from numpy.typing import NDArray

class StateSpaceController:
    def __init__(
        self,
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        Cr: NDArray[np.float64],
        des_poles: NDArray,
        use_kr: bool = False,
        print_gains: bool = False,
    ):
        # TODO: decide whether or not to do dimension checks
        dim_state, dim_input = B.shape
        dim_ref = Cr.shape[0]

        # Ensure the dimensions match
        assert len(des_poles) == dim_state

        # verify controllability
        ctrl_mat = ctrl.ctrb(A, B)
        if np.linalg.matrix_rank(ctrl_mat) != dim_state:
            raise ValueError("System is not controllable.")
        
        # compute control gain from desired poles
        self.K = ctrl.place(A, B, des_poles)
        if print_gains:
            print(f"K =", self.K)
        if use_kr:
            A_cl = A - B @ self.K
            self.kr = -np.linalg.inv(Cr @ np.linalg.inv(A_cl) @ B)
            if print_gains:
                print(f"kr = {self.kr}")
        else:
            self.kr = np.zeros((dim_input, dim_ref))

    def update(self, r: NDArray[np.float64], x: NDArray[np.float64]):
        u = -self.K @ x + self.kr @ r
        return u