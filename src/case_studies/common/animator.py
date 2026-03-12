# 3rd-party
import numpy as np
from numpy.typing import NDArray
from matplotlib.axes import Axes
from matplotlib.artist import Artist
from pyqtgraph import opengl


def check_data_shape(data: NDArray[np.float64] | None, name: str) -> None:
    """
    Check that the shape of the data array is correct.

    Args:
        data (NDArray[np.float64]): Data array to check.
        name (str): Name of the data array (for error messages).

    Raises:
        ValueError: If the shape of the data array is incorrect.
    """
    if data is None:
        return
    dim, num_time_steps = data.shape
    if dim > num_time_steps:
        dim_type = "state" if "x" in name else "reference"
        msg = [
            f"Pay attention to the shape of {name}!",
            f"{name} should have shape (dim_{dim_type}, num_time_steps).",
            "Did you mean to transpose it?",
        ]
        raise ValueError("\n".join(msg))


class MatplotlibAxisAnimator:
    def __init__(
        self,
        ax: Axes,
        x_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None = None,
        blit: bool = True,
    ):
        """
        Constructor for the AnimatorBase class. Saves self.ax and self.x_hist,
        which are available to derived classes. Derived classes do not need to
        implement their own constructor unless additional arguments are needed;
        they do, however, need to implement the `setup_background()`,
        `setup_changing_objects()`, and `update_changing_objects()` methods.

        Args:
            ax (Axes): Matplotlib Axes object where the animation will be drawn.
            x_hist (NDArray[np.float64]): State history array with shape
                (dim_state, num_time_steps).
            r_hist (NDArray[np.float64]): Reference history array with shape
                (dim_reference, num_time_steps).
            blit (bool): Whether or not to use blitting for efficient redrawing.
                Default is True. If False, the entire canvas will be redrawn
                each frame, which is less efficient but may be necessary for
                some complex plots or if you have smearing issues with blitting.
        """
        self.ax = ax
        self._blit = blit

        check_data_shape(x_hist, "x")
        check_data_shape(r_hist, "r")
        self.x_hist = x_hist.T
        self.r_hist = r_hist.T if r_hist is not None else None

        figure = self.ax.figure
        self._canvas = figure.canvas

        self.setup_background()
        figure.tight_layout()  # type: ignore
        self._save_background()

        x0 = self.x_hist[0]
        r0 = self.r_hist[0] if self.r_hist is not None else None
        self.setup_changing_objects(x0, r0)

    def setup_background(self) -> None:
        """
        Set up the static background elements of the animation (everything that
        does not change over time). These elements will all be saved as the
        background for efficient redrawing when using blitting.
        """
        raise NotImplementedError('Derived class must define "setup_background"')

    def setup_changing_objects(
        self, x0: NDArray[np.float64], r0: NDArray[np.float64] | None
    ) -> None:
        """
        Set up the dynamic elements of the animation (everything that changes
        over time). These elements will be updated in each frame of the animation
        in an efficient manner when using blitting.

        Args:
            x0 (NDArray[np.float64]): Initial state array with shape (dim_state,).
            r0 (NDArray[np.float64]): Initial reference array with shape
                (dim_reference,).
        """
        raise NotImplementedError('Derived class must define "setup_changing_objects"')

    def update_changing_objects(
        self, x: NDArray[np.float64], r: NDArray[np.float64] | None
    ) -> tuple[Artist, ...]:
        """
        Update all of the dynamic elements of the animation created in
        setup_changing_objects() to reflect the state at index i.

        Args:
            x (NDArray[np.float64]): Current state array with shape (dim_state,).
            r (NDArray[np.float64]): Current reference array with shape
                (dim_reference,).
        Returns:
            tuple[Artist, ...]: Tuple of all matplotlib artists that were updated.
                It is important that all artists that are updated are returned
                so that they can be efficiently redrawn when using blitting.
        """
        raise NotImplementedError('Derived class must define "update_changing_objects"')

    def step_animation(self, i: int) -> None:
        """
        Update the animation by one time step, drawing the state at index i.

        Args:
            i (int): Index of the time step to draw.

        Note:
            DO NOT EDIT THIS FUNCTION!
        """
        if self._blit:
            self._draw_saved_background()
        x = self.x_hist[i]
        r = self.r_hist[i] if self.r_hist is not None else None
        artists = self.update_changing_objects(x, r)
        for artist in artists:
            self.ax.draw_artist(artist)

    def _save_background(self) -> None:
        """
        Save the background of the axes for efficient redrawing when using blitting.

        Note:
            DO NOT EDIT THIS FUNCTION!
        """
        self._canvas.draw()
        self._background = self._canvas.copy_from_bbox(self.ax.bbox)  # type: ignore

    def _draw_saved_background(self) -> None:
        """
        Restore the saved background for efficient redrawing when using blitting.

        Note:
            DO NOT EDIT THIS FUNCTION!
        """
        if self._background is not None:
            self._canvas.restore_region(self._background)  # type: ignore


class OpenglWidgetAnimator:
    def __init__(
        self,
        widget: opengl.GLViewWidget,
        x_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None = None,
    ):
        """
        Constructor for the OpenglWidgetAnimator class. Saves self.widget,
        self.x_hist, and self.r_hist, which are all available to derived classes.
        Derived classes do not need to implement their own constructor unless
        additional arguments are needed; they do, however, need to implement
        the `setup_background()`, `setup_changing_objects()`, and
        `update_changing_objects()` methods.

        Args:
            x_hist (NDArray[np.float64]): State history array with shape
                (dim_state, num_time_steps).
            r_hist (NDArray[np.float64]): Reference history array with shape
                (dim_reference, num_time_steps).
        """
        self.widget = widget

        check_data_shape(x_hist, "x")
        check_data_shape(r_hist, "r")
        self.x_hist = x_hist.T
        self.r_hist = r_hist.T if r_hist is not None else None

        self.setup_background()

        x0 = self.x_hist[0]
        r0 = self.r_hist[0] if self.r_hist is not None else None
        self.setup_changing_objects(x0, r0)

    def setup_background(self) -> None:
        """
        Set up the static background elements of the animation (everything that
        does not change over time). These elements will all be saved as the
        background for efficient redrawing when using blitting.
        """
        raise NotImplementedError('Derived class must define "setup_background"')

    def setup_changing_objects(
        self, x0: NDArray[np.float64], r0: NDArray[np.float64] | None
    ) -> None:
        """
        Set up the dynamic elements of the animation (everything that changes
        over time). These elements will be updated in each frame of the animation
        in an efficient manner when using blitting.

        Args:
            x0 (NDArray[np.float64]): Initial state array with shape (dim_state,).
            r0 (NDArray[np.float64]): Initial reference array with shape
                (dim_reference,).
        """
        raise NotImplementedError('Derived class must define "setup_changing_objects"')

    def update_changing_objects(
        self, x: NDArray[np.float64], r: NDArray[np.float64] | None
    ) -> None:
        """
        Update the widget to reflect the current state x and reference r.

        Args:
            x (NDArray[np.float64]): Current state array with shape (dim_state,).
            r (NDArray[np.float64]): Current reference array with shape
                (dim_reference,).
        """
        raise NotImplementedError('Derived class must define "update_changing_objects"')

    def step_animation(self, i: int) -> None:
        """
        Update all of the dynamic elements of the animation to reflect the
        state at index i.

        Args:
            i (int): Index of the time step to draw.

        Note:
            DO NOT EDIT THIS FUNCTION!
        """
        x = self.x_hist[i]
        r = self.r_hist[i] if self.r_hist is not None else None
        self.update_changing_objects(x, r)
