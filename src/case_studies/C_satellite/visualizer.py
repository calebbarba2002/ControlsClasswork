# 3rd-party
import numpy as np
from numpy.typing import NDArray
from matplotlib.axes import Axes

# local (controlbook)
from ..common import Visualizer
from .animator import SatelliteAnimator


class SatelliteVisualizer(Visualizer):
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
            r"$\theta$ (deg)",
            r"$\phi$ (deg)",
            r"$\dot{\theta}$ (rad/s)",
            r"$\dot{\phi}$ (rad/s)",
        ]
        u_labels = [r"$\tau$ (Nm)"]

        # TODO: do this? or use refs = [None, phi_ref] in simulation?

        # The base Visualizer class plots indices of r_hist matching x_hist,
        # but for this system r[0] (phi_ref) corresponds to x[1] (phi).
        # This just makes it so that phi_ref is at the index 1 while theta_ref
        # at index 0 is ignored (nan's do not show up in the plot).
        # Optionally, could use same data for both theta_ref and phi_ref since
        # the system does try to keep them equal.
        if isinstance(r_hist, np.ndarray) and r_hist.shape[1] == 1:
            # r_hist is (N, 1), need to transpose and expand to (2, N)
            r_hist = r_hist.T  # Now (1, N)
            r_hist = np.block([[r_hist * np.nan], [r_hist]])  # Now (2, N)

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
        return SatelliteAnimator(ax, x_hist, r_hist, blit)
