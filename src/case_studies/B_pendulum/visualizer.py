# 3rd-party
import numpy as np
from numpy.typing import NDArray
from matplotlib.axes import Axes

# local (controlbook)
from ..common import Visualizer
from .animator import CartPendulumAnimator


class CartPendulumVisualizer(Visualizer):
    def __init__(
        self,
        t_hist: NDArray[np.float64],
        x_hist: NDArray[np.float64],
        u_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None = None,
        xhat_hist: NDArray[np.float64] | None = None,
        d_hist: NDArray[np.float64] | None = None,
        dhat_hist: NDArray[np.float64] | None = None,
    ):
        # NOTE: base Visualizer class will convert data to degrees for plotting
        # if "deg" is in the label
        x_labels = [
            "z (m)",
            r"$\theta$ (deg)",
            r"$\dot{z}$ (m/s)",
            r"$\dot{\theta}$ (rad/s)",
        ]
        u_labels = ["F (N)"]
        super().__init__(
            t_hist,
            x_hist,
            x_labels,
            u_hist,
            u_labels,
            r_hist,
            d_hist,
            xhat_hist,
            dhat_hist,
        )

    def get_system_animator(
        self,
        viz_object: Axes,
        x_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None,
        blit=True,
    ):
        assert isinstance(viz_object, Axes)
        r_hist = None  # animation does not use reference
        ax = viz_object
        return CartPendulumAnimator(ax, x_hist, r_hist, blit)
