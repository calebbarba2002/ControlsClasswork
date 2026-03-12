# standard library
from collections.abc import Sequence

# 3rd-party
import numpy as np
from numpy.typing import NDArray

# local (controlbook)
from . import DynamicsBase, SignalGenerator, ControllerBase


# print arrays with 4 decimal places, suppressing scientific notation
np.set_printoptions(precision=4, suppress=True)


def run_simulation(
    sys: DynamicsBase,
    refs: Sequence[SignalGenerator | None],
    controller: ControllerBase,
    # TODO: should this just be a bool "control_with_truth/state"
    controller_input: str = "state",
    input_disturbance: NDArray[np.float64] | None = None,
    output_noise: list[SignalGenerator] | None = None,
    t_final: float = 20.0,
    dt: float = 0.01,
):
    x_hist = [sys.state]
    r_hist = [np.array([ref.square(0.0) if ref is not None else None for ref in refs])]
    u_hist = []
    xhat_hist = [np.zeros_like(sys.state)]
    d_hist = []
    dhat_hist = []

    time = np.arange(start=0.0, stop=t_final, step=dt, dtype=np.float64)
    y = sys.h()
    for t in time[1:]:
        r = np.array([ref.square(t) if ref is not None else np.nan for ref in refs])

        # TODO: is it better to add noise to sys.h() instead of here?
        if output_noise is None:
            noise = np.zeros_like(y)
        else:
            noise = np.array([n.random(t) for n in output_noise])

        if controller_input == "state":
            u = controller.update_with_state(r, sys.state)
        elif controller_input == "measurement":
            # TODO: should we separate observation and control?
            # xhat = observer.update_xhat(y + noise)
            # u, xhat = controller.update_with_state(r, xhat)
            ret = controller.update_with_measurement(r, y + noise)
            if len(ret) == 2:
                u, xhat = ret
            elif len(ret) == 3:
                u, xhat, dhat = ret
                dhat_hist.append(dhat)
            else:
                raise ValueError(
                    "controller.update_with_measurement()"
                    f"returned {len(ret)} values, expected 2 or 3."
                )
            xhat_hist.append(xhat)
        else:
            msg = (
                f"Invalid controller_input {controller_input}"
                + ', must be "state" or "measurement".'
            )
            raise ValueError(msg)

        if input_disturbance is None:
            input_disturbance = np.zeros_like(u)
        else:
            d_hist.append(input_disturbance)

        y = sys.update(u + input_disturbance)

        # save data
        x_hist.append(sys.state)
        r_hist.append(r)
        u_hist.append(u)

    # convert data lists to numpy arrays
    u_hist = np.array(u_hist)
    x_hist = np.array(x_hist)
    r_hist = np.array(r_hist, dtype=np.float64)
    if len(xhat_hist) < 2:
        xhat_hist = None
    else:
        xhat_hist = np.array(xhat_hist, dtype=np.float64)
    if len(d_hist) == 0:
        d_hist = None
    else:
        d_hist = np.array(d_hist)
    if len(dhat_hist) == 0:
        dhat_hist = None
    else:
        dhat_hist = np.array(dhat_hist)

    return time, x_hist, u_hist, r_hist, xhat_hist, d_hist, dhat_hist
