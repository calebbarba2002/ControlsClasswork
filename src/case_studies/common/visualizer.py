# 3rd-party
import numpy as np
from numpy.typing import NDArray
import abracatabra as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from pyqtgraph.opengl import GLViewWidget

# local (controlbook)
from .animator import MatplotlibAxisAnimator, OpenglWidgetAnimator
from .data_plot import DataPlot


class Visualizer:
    def __init__(
        self,
        t_hist: NDArray[np.float64],
        x_hist: NDArray[np.float64],
        x_labels: list[str],
        u_hist: NDArray[np.float64],
        u_labels: list[str],
        r_hist: NDArray[np.float64] | None = None,
        d_hist: NDArray[np.float64] | None = None,
        xhat_hist: NDArray[np.float64] | None = None,
        dhat_hist: NDArray[np.float64] | None = None,
        mpl_axis: bool = True,
    ):
        self.mpl_axis = mpl_axis

        # save time data and check dimensions
        self.N = len(t_hist)
        N1 = self.N - 1
        self.t_hist = t_hist.squeeze()
        if t_hist.ndim != 1:
            raise ValueError('"t_hist" must be a 1D array')

        # save state data and check dimensions
        if x_hist.ndim != 2:
            raise ValueError('"x_hist" must be a 2D array')
        if not self.N in x_hist.shape:
            raise ValueError('"x_hist" must have the same number of rows as "t_hist"')
        self.num_states = min(x_hist.shape)
        self.num_position_states = self.num_states // 2
        self.x_hist = x_hist if x_hist.shape[1] == self.N else x_hist.T

        if len(x_labels) != self.num_states:
            raise ValueError('dimension of "x_labels" must match that of "x_hist"')
        self.x_labels = x_labels

        # save input data and check dimensions
        if u_hist.ndim != 2:
            raise ValueError('"u_hist" must be a 2D array')
            # raise ValueError(f'"u_hist" must be a 2D array, got shape {u_hist.shape}')
        if not N1 in u_hist.shape:
            raise ValueError(
                '"u_hist" must have the same number of rows as "t_hist" minus one'
            )
        self.num_inputs = min(u_hist.shape)
        self.u_hist = u_hist if u_hist.shape[1] == N1 else u_hist.T

        if len(u_labels) != self.num_inputs:
            raise ValueError('dimension of "u_labels" must match that of "u_hist"')
        self.u_labels = u_labels

        # save reference data and check dimensions
        if r_hist is not None:
            if r_hist.ndim != 2:
                raise ValueError('"r_hist" must be a 2D array')
            if not self.N in r_hist.shape:
                raise ValueError(
                    '"r_hist" must have the same number of rows as "t_hist"'
                )
            self.r_hist = r_hist if r_hist.shape[1] == self.N else r_hist.T
        else:
            self.r_hist = None

        # save distubance data and check dimensions
        if d_hist is not None:
            if d_hist.ndim != 2:
                raise ValueError('"d_hist" must be a 2D array')
            if not N1 in d_hist.shape:
                raise ValueError(
                    '"d_hist" must have the same number of rows as "t_hist" minus one'
                )
            if not self.num_inputs in d_hist.shape:
                raise ValueError(
                    '"d_hist" must have the same number of columns as "u_hist"'
                )
            self.d_hist = d_hist if d_hist.shape[1] == N1 else d_hist.T
        else:
            self.d_hist = None

        # save estimated state data and check dimensions
        if xhat_hist is not None:
            if xhat_hist.ndim != 2:
                raise ValueError('"xhat_hist" must be a 2D array')
            if not self.N in xhat_hist.shape:
                raise ValueError(
                    '"xhat_hist" must have the same number of rows as "t_hist"'
                )
            if not self.num_states in xhat_hist.shape:
                raise ValueError(
                    '"xhat_hist" must have the same number of columns as "x_hist"'
                )
            self.xhat_hist = xhat_hist if xhat_hist.shape[1] == self.N else xhat_hist.T
        else:
            self.xhat_hist = None

        # save estimated disturbance data and check dimensions
        if dhat_hist is not None:
            if self.d_hist is None:
                raise ValueError(
                    '"dhat_hist" cannot be provided without "d_hist" being provided'
                )
            if dhat_hist.ndim != 2:
                raise ValueError('"dhat_hist" must be a 2D array')
            if not N1 in dhat_hist.shape:
                raise ValueError(
                    '"dhat_hist" must have the same number of rows as "t_hist" minus one'
                )
            if not self.num_inputs in dhat_hist.shape:
                raise ValueError(
                    '"dhat_hist" must have the same number of columns as "u_hist"'
                )
            self.dhat_hist = dhat_hist if dhat_hist.shape[1] == N1 else dhat_hist.T
        else:
            self.dhat_hist = None

    def plot(self, size: tuple[float, float] = (0.5, 0.8), use_single_window=True):
        """
        Plot data and keep plots open when finished until they are manually closed.

        Args:
            size (tuple[float, float]): Size of the window as a percent of the
                screen (width, height). Values must be between 0 and 1.
            use_single_window (bool): If True, all data plots will be in a single
                window. If False, observer and disturbance plots will be in their
                own separate windows.
        """
        window = plt.TabbedPlotWindow("Sim Data", size=size)
        fig_track = window.add_figure_tab("tracking")
        fig_vel = window.add_figure_tab("velocity")

        if use_single_window:
            if self.xhat_hist is not None:
                fig_obs = window.add_figure_tab("observer")
            else:
                fig_obs = None
            if self.d_hist is not None:
                fig_d = window.add_figure_tab("disturbance")
            else:
                fig_d = None

        else:
            if self.xhat_hist is not None:
                window_obs = plt.TabbedPlotWindow(
                    "Sim Data - Observer", size=size, autohide_tabs=True
                )
                fig_obs = window_obs.add_figure_tab("observer")
            else:
                fig_obs = None

            if self.d_hist is not None:
                window_d = plt.TabbedPlotWindow(
                    "Sim Data - Disturbance", size=size, autohide_tabs=True
                )
                fig_d = window_d.add_figure_tab("disturbance")
            else:
                fig_d = None

        track_plots, vel_plots, obs_plots, dist_plots = self.setup_data_plots(
            fig_track, fig_vel, fig_obs, fig_d, blit=False
        )
        for viz in track_plots + vel_plots + obs_plots + dist_plots:
            viz.plot()
        plt.show_all_windows(tight_layout=True)

    def animate(
        self,
        # you can change these defaults if you want
        step: int = 5,
        speed_scale: float = 1.0,
        blit: bool = True,
        size: tuple[float, float] = (0.8, 0.8),
        add_animation_player: bool = True,
        use_single_window: bool = True,
        print_timing: bool = False,
    ):
        """
        Animate the system and data plots, and keep plots open when finished
        until they are manually closed.

        Args:
            step (int): How many time steps to plot between each rendering.
            speed_scale (float): Factor to speed up from (attempted) real time.
                Ex: 2 means the animation will try to run at 2x speed.
            blit (bool): Whether or not to blit. Blitting is when you only render
                changed pixels rather than the entire plot and is much faster, but
                it must be used correctly or else smearing can occur. For example,
                resizing the window during a blitted animation will cause smearing.
            size (tuple[float, float]): Size of the window as a percent of the
                screen (width, height). Values must be between 0 and 1.
            add_animation_player (bool): Whether or not to add an animation
                player (play, pause, scrubber) to the animation figure.
            use_single_window (bool): If True, all data plots will be in a single
                window. If False, observer and disturbance plots will be in their
                own separate windows.
            print_timing (bool): Whether or not to print out timing information
                during the animation (sim time vs real time).
        """
        if 1 < np.any(size) < 0:
            raise ValueError('"size" values must be between 0 and 1')

        window = plt.TabbedPlotWindow(
            "Animation", ncols=2, size=size, autohide_tabs=False
        )

        if self.mpl_axis:
            fig_ani = window.add_figure_tab(
                "animation",
                col=0,
                blit=blit,
                include_toolbar=True,
                add_animation_player=add_animation_player,
            )
            # TODO: decide if all animations should use "equal" and "box"
            ax = fig_ani.add_subplot()
            # fig_a.tight_layout()
            # ax.set_aspect("equal", "box")
            # animator = Animator(ax, self.x_hist)
            animator = self.get_system_animator(ax, self.x_hist, self.r_hist, blit)
        else:  # use OpenGL widget
            gl_widget = GLViewWidget()
            window.add_custom_tab(
                gl_widget, "animation", col=0, add_animation_player=add_animation_player
            )
            animator = self.get_system_animator(gl_widget, self.x_hist, self.r_hist)
        window.register_animation_callback(animator.step_animation, "animation", col=0)

        fig_track = window.add_figure_tab("tracking", col=1, blit=blit)
        fig_vel = window.add_figure_tab("velocity", col=1, blit=blit)

        if use_single_window:
            if self.xhat_hist is not None:
                fig_obs = window.add_figure_tab("observer", col=1, blit=blit)
            else:
                fig_obs = None
            if self.d_hist is not None:
                fig_d = window.add_figure_tab("disturbance", col=1, blit=blit)
            else:
                fig_d = None

        else:
            x, y = size
            size = (x / 2, y)  # keep windows for data plots the same size

            if self.xhat_hist is not None:
                window_obs = plt.TabbedPlotWindow(
                    "Sim Data - Observer", size=size, autohide_tabs=True
                )
                fig_obs = window_obs.add_figure_tab("observer", blit=blit)
            else:
                fig_obs = None

            if self.d_hist is not None:
                window_d = plt.TabbedPlotWindow(
                    "Sim Data - Disturbance", size=size, autohide_tabs=True
                )
                fig_d = window_d.add_figure_tab("disturbance", blit=blit)
            else:
                fig_d = None

        track_plots, vel_plots, obs_plots, d_plots = self.setup_data_plots(
            fig_track, fig_vel, fig_obs, fig_d, blit
        )

        def update_tracking_fig(i: int):
            for data_plot in track_plots:
                data_plot.animation_step(i)

        def update_velocity_fig(i: int):
            for data_plot in vel_plots:
                data_plot.animation_step(i)

        def update_observer_fig(i: int):
            for data_plot in obs_plots:
                data_plot.animation_step(i)

        def update_disturbance_fig(i: int):
            for data_plot in d_plots:
                data_plot.animation_step(i)

        window.register_animation_callback(update_tracking_fig, "tracking", col=1)
        window.register_animation_callback(update_velocity_fig, "velocity", col=1)
        if len(obs_plots) > 0:
            window.register_animation_callback(update_observer_fig, "observer", col=1)
        if len(d_plots) > 0:
            window.register_animation_callback(
                update_disturbance_fig, "disturbance", col=1
            )

        dt = self.t_hist[1] - self.t_hist[0]
        plt.animate_all_windows(
            self.N,
            dt,
            step,
            speed_scale,
            print_timing,
            use_player=add_animation_player,
            hold=True,
        )

    def setup_data_plots(
        self,
        fig_track: Figure,
        fig_vel: Figure,
        fig_obs: Figure | None,
        fig_d: Figure | None,
        blit=False,
    ) -> tuple[list[DataPlot], ...]:

        track_plots: list[DataPlot] = []
        vel_plots: list[DataPlot] = []
        obs_plots: list[DataPlot] = []
        dist_plots: list[DataPlot] = []
        num_t_plots = self.num_position_states + self.num_inputs
        for i in range(self.num_position_states):
            ax = fig_track.add_subplot(num_t_plots, 1, i + 1)
            r_hist = None
            if self.r_hist is not None and len(self.r_hist) > i:
                r_hist = self.r_hist[i]
            plot = DataPlot(
                ax,
                self.t_hist,
                self.x_hist[i],
                label=self.x_labels[i],
                ref_data=r_hist if r_hist is not None else None,
                blit=blit,
                legend=i == 0,
            )
            track_plots.append(plot)

            ax = fig_vel.add_subplot(self.num_position_states, 1, i + 1)
            j = self.num_position_states + i
            plot = DataPlot(
                ax,
                self.t_hist,
                self.x_hist[j],
                label=self.x_labels[j],
                label_time=True if i == self.num_position_states - 1 else False,
                blit=blit,
                # legend=i == 0,
            )
            vel_plots.append(plot)

        if isinstance(fig_obs, Figure) and self.xhat_hist is not None:
            for i in range(self.num_states):
                ax = fig_obs.add_subplot(self.num_states, 1, i + 1)
                plot = DataPlot(
                    ax,
                    self.t_hist,
                    self.x_hist[i],
                    label=self.x_labels[i],
                    label_time=True if i == self.num_states - 1 else False,
                    obs_data=self.xhat_hist[i],
                    blit=blit,
                    legend=i == 0,
                )
                obs_plots.append(plot)

        for i in range(self.num_inputs):
            ax = fig_track.add_subplot(num_t_plots, 1, self.num_position_states + i + 1)
            plot = DataPlot(
                ax,
                self.t_hist[:-1],
                self.u_hist[i],
                label=self.u_labels[i],
                label_time=True if i == self.num_inputs - 1 else False,
                blit=blit,
                # legend=i == 0,
            )
            track_plots.append(plot)

            if isinstance(fig_d, Figure):
                assert self.d_hist is not None
                ax = fig_d.add_subplot(self.num_inputs, 1, i + 1)
                plot = DataPlot(
                    ax,
                    self.t_hist[:-1],
                    self.d_hist[i],
                    label=self.u_labels[i],
                    label_time=True if i == self.num_inputs - 1 else False,
                    obs_data=self.dhat_hist[i] if self.dhat_hist is not None else None,
                    blit=blit,
                    legend=i == 0,
                )
                dist_plots.append(plot)

        # save the background after all plots are created for blitting to work
        for plot in track_plots + vel_plots + obs_plots + dist_plots:
            plot.save_background()

        return track_plots, vel_plots, obs_plots, dist_plots

    def get_system_animator(
        self,
        viz_object: Axes | GLViewWidget,
        x_hist: NDArray[np.float64],
        r_hist: NDArray[np.float64] | None,
        blit: bool = True,
    ) -> MatplotlibAxisAnimator | OpenglWidgetAnimator:
        msg = "Subclass must define 'get_system_animator' method."
        raise NotImplementedError(msg)
