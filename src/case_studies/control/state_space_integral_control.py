# 3rd-party
import numpy as np
import control as ctrl
from numpy.typing import NDArray

# local (controlbook)
from .integration_filter import IntegrationArrayFilter
from .state_space_control import StateSpaceController

def augment_system(
    A: NDArray[np.float64],
    B: NDArray[np.float64],
    Cr: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """
    Augment the system matrices for integral control.
    """
    dim_state, dim_input = B.shape
    dim_ref = Cr.shape[0]

    A1 = np.block([[A, np.zeros((dim_state, dim_ref))], # fmt: skip
        [-Cr, np.zeros((dim_ref, dim_ref))]]) # fmt: skip
    B1 = np.block([[B], [np.zeros((dim_ref, dim_input))]])

    return A1, B1

class StateSpaceIntegralController:
    def __init__(
        self,
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        Cr: NDArray[np.float64],
        des_poles: NDArray,
        ts: float,
        antiwindup_vel_tol: float = float("inf"),
        print_gains: bool = False,
    ):
        # get dimensions
        dim_state, dim_input = B.shape
        dim_ref = Cr.shape[0]

        # Ensure the dimensions match
        assert len(des_poles) == dim_state + dim_ref

        # augment system for integral control
        A1, B1 = augment_system(A, B, Cr)

        # verify controllability
        ctrl_mat = ctrl.ctrb(A1, B1)
        if np.linalg.matrix_rank(ctrl_mat) != len(A1):
            raise ValueError("System is not controllable.")
        
        # compute control gain from desired poles
        self.K1 = ctrl.place(A1, B1, des_poles)
        self.K = self.K1[:, :-dim_ref]
        self.ki = self.K1[:, -dim_ref:]

        if print_gains:
            print(f"K1 =", self.K1)

        # save the reference matrix
        self.Cr = Cr

        # create the integrator
        self.integrator = IntegrationArrayFilter(ts, dim_ref)

        # TODO: add anti-windup?
        self.antiwindup_vel_tol = antiwindup_vel_tol

    # TODO: decide whether to name variables with tilde or not
    def update(self, r: NDArray[np.float64], x: NDArray[np.float64]):
        error = r - self.Cr @ x

        # anti-windup based on velocity
        n2 = len(x) // 2
        vel = np.abs(self.Cr[:, :n2] @ x[n2:]) # velocity of reference states
        mask = np.abs(vel) < self.antiwindup_vel_tol # where velocity is small enough
        error_integral = self.integrator.update(error * mask)

        # TODO: decide which version to use
        separate_integrator = True
        if separate_integrator:
            u = -self.K @ x - self.ki @ error_integral
        else:
            x1 = np.hstack([x, error_integral])
            u = -self.K1 @ x1
        return u

    def add_anti_windup_saturation(
        self, u_sat: NDArray[np.float64], u_unsat: NDArray[np.float64]
    ):
        """
        Add anti-windup saturation to the integral controller.
        """
        if np.all(self.ki == 0.0):
            return
        self.integrator.add_anti_windup_saturation(self.ki, u_sat, u_unsat)

class StateSpaceIntegralController_v2:
    def __init__(
        self,
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        Cr: NDArray[np.float64],
        des_poles: NDArray,
        ts: float,
        print_gains: bool = False,
    ):
        # get dimensions
        dim_state, dim_input = B.shape
        dim_ref = Cr.shape[0]
        # TODO: decide whether or not to do dimension checks
        assert len(des_poles) == dim_state + dim_ref # assumes 1 integrator per ref

        # augment system for integral control
        A1 = np.block([[A, np.zeros((dim_state, dim_ref))], # fmt: skip
            [-Cr, np.zeros((dim_ref, dim_ref))]]) # fmt: skip
        B1 = np.block([[B], [np.zeros((dim_ref, dim_input))]])
        self.Cr = Cr

        self.integrator = IntegrationArrayFilter(ts, dim_ref)

        # check controllability
        ctrl_mat = ctrl.ctrb(A1, B1)
        if np.linalg.matrix_rank(ctrl_mat) != len(A1):
            raise ValueError("System is not controllable.")
        
        # create controller gains
        self.K1 = ctrl.place(A1, B1, des_poles)
        self.K = self.K1[:, :dim_state]
        self.ki = self.K1[:, dim_state:]
        if print_gains:
            print(f"K1 =", self.K1)

        # TODO: decide whether to name variables with tilde or not
    def update(self, r: NDArray[np.float64], x: NDArray[np.float64]):
        error = r - self.Cr @ x

        # TODO: decide which version to use
        separate_integrator = True
        error_integral = self.integrator.update(error)
        if separate_integrator:
            u = -self.K @ x - self.ki @ error_integral
        else:
            x1 = np.hstack([x, error_integral])
            u = -self.K1 @ x1
        return u

    def add_anti_windup_saturation(
        self, u_sat: NDArray[np.float64], u_unsat: NDArray[np.float64]
    ):
        """
        Add anti-windup saturation to the integral controller.
        """
        if not np.all(self.ki == 0.0):
            self.integrator.add_anti_windup_saturation(self.ki, u_sat, u_unsat)
            
    # def update(
    # self, r_tilde: NDArray[np.float64], x_tilde: NDArray[np.float64]
    # ):
    # error = r - self.Cr @ x
    # error_integral = self.integrator.update(error)
    # u = -self.K @ x + self.ki @ error_integral
    # # x1 = np.stack([x, error_integral])
    # # u = -self.K1 @ x1
    # u_tilde = -self.K @ x_tilde + self.kr @ r_tilde
    # return u_tilde