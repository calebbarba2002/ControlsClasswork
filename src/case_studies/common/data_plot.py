# 3rd-party
import numpy as np
from numpy.typing import NDArray
from matplotlib.axes import Axes


class DataPlot:
    def __init__(
        self,
        ax: Axes,
        time: NDArray[np.float64],
        data: NDArray[np.float64],
        label: str,
        label_time: bool = False,
        ref_data: NDArray[np.float64] | None = None,
        obs_data: NDArray[np.float64] | None = None,
        legend: bool = False,
        blit: bool = True,
    ):
        self.ax = ax
        self.canvas = self.ax.figure.canvas
        self.time = time.squeeze()  # TODO: do we want to correct bad inputs?
        if self.time.ndim != 1:
            raise ValueError('"time" must be a 1D array')

        # TODO: should we automatically convert to degrees if label contains "deg"?
        # or would that be too confusing for students?
        if "deg" in label:
            self.data = np.rad2deg(data.squeeze())
        else:
            self.data = data.squeeze()
        if self.time.shape != self.data.shape:
            raise ValueError('"data" must have the same length as "time"')

        if ref_data is not None:
            ref_data = ref_data.astype(np.float64)
            if "deg" in label:
                self.ref_data = np.rad2deg(ref_data.squeeze())
            else:
                self.ref_data = ref_data.squeeze()
            if self.ref_data.shape != self.time.shape:
                raise ValueError('"ref_data" must have the same length as "time"')
        else:
            self.ref_data = None

        if obs_data is not None:
            if "deg" in label:
                self.obs_data = np.rad2deg(obs_data.squeeze())
            else:
                self.obs_data = obs_data.squeeze()
            if not self.obs_data.shape == self.time.shape:
                raise ValueError('"obs_data" must have the same length as "time"')
        else:
            self.obs_data = None

        if label_time:
            self.ax.set_xlabel("time (s)")
        self.ax.set_ylabel(label)
        self.blit = blit

        # plot observed data first (underneath)
        if self.obs_data is not None:
            (self.obs_line,) = self.ax.plot(
                self.time, self.obs_data, "m", label="observed"
            )

        # plot actual data on top of observed data
        (line,) = self.ax.plot(self.time, self.data, label="actual")
        self.data_line = line

        # plot reference data on top of actual data
        if self.ref_data is not None:
            (self.ref_line,) = self.ax.plot(
                self.time, self.ref_data, "r--", label="reference"
            )

        # TODO: decide whether to include legend and whether to disable it if
        # there is only one line
        if legend and len(self.ax.get_lines()) > 1:
            ## place legend centered above the plot
            legend_ = self.ax.legend(
                bbox_to_anchor=(0.5, 1.0), loc="lower center", ncol=3, fontsize=9
            )
            ## code below places the legend in the best location within the plot
            # self.canvas.draw()
            # bbox_disp = legend_.get_window_extent()
            # bbox_axes = bbox_disp.transformed(self.ax.transAxes.inverted())
            # bounds = bbox_axes.bounds
            # self.ax.legend(loc="lower left", bbox_to_anchor=bounds)

        self.save_background()

    def plot(self):
        if isinstance(self.obs_data, np.ndarray):
            self.obs_line.set_xdata(self.time)
            self.obs_line.set_ydata(self.obs_data)

        self.data_line.set_xdata(self.time)
        self.data_line.set_ydata(self.data)

        if isinstance(self.ref_data, np.ndarray):
            self.ref_line.set_xdata(self.time)
            self.ref_line.set_ydata(self.ref_data)

    def animation_step(self, i: int):
        if self.blit:
            canvas = self.ax.figure.canvas
            canvas.restore_region(self.background)  # type: ignore

        if isinstance(self.obs_data, np.ndarray):
            self.obs_line.set_xdata(self.time[:i])
            self.obs_line.set_ydata(self.obs_data[:i])
            self.ax.draw_artist(self.obs_line)

        self.data_line.set_xdata(self.time[:i])
        self.data_line.set_ydata(self.data[:i])
        self.ax.draw_artist(self.data_line)

        if isinstance(self.ref_data, np.ndarray):
            self.ref_line.set_xdata(self.time[:i])
            self.ref_line.set_ydata(self.ref_data[:i])
            self.ax.draw_artist(self.ref_line)

    def save_background(self):
        self.ax.figure.tight_layout()  # type: ignore
        self.data_line.set_xdata([])
        self.data_line.set_ydata([])
        # TODO: decide whether to animate reference line
        if hasattr(self, "ref_line"):
            self.ref_line.set_xdata([])
            self.ref_line.set_ydata([])
        if hasattr(self, "obs_line"):
            self.obs_line.set_xdata([])
            self.obs_line.set_ydata([])
        canvas = self.ax.figure.canvas
        canvas.draw()
        self.background = canvas.copy_from_bbox(self.ax.bbox)  # type: ignore
