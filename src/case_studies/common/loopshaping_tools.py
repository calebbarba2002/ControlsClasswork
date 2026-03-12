# 3rd-Party
import numpy as np
import control as cnt
from matplotlib.axes import Axes


def print_margins(sys, label, use_dB=False):
    """
    Print gain and phase margins for a given system "sys" with a label
    "label". If use_dB is true, gain margin is printed in dB.
    """
    gm, pm, Wcg, Wcp = cnt.margin(sys)
    if use_dB:
        gm = cnt.mag2db(gm)
    print(f"{label} margins:")
    print(f"\tGain Margin: {gm:.2f} at Wcg: {Wcg:.2f}")
    print(f"\tPhase Margin: {pm:.2f} at Wcp: {Wcp:.2f}")


def draw_noise_spec(axes: Axes, gamma_n: float, omega_n: float, use_dB: bool = False):
    """
    Add noise specification, defined by gamma_n for remaining magnitude
    of error, and omega_n for the frequency at which it needs to happen.
    """
    xlim = axes.get_xlim()
    ylim = axes.get_ylim()

    omega_max = xlim[1]
    x_pts = [omega_n, omega_max]
    y_upper = [ylim[1]] * 2
    y_lower = [gamma_n] * 2
    if use_dB:
        y_lower = cnt.mag2db(y_lower)
    axes.fill_between(
        x_pts, y_upper, y_lower, color="red", alpha=0.2, label="noise spec"
    )
    axes.set_xlim(xlim)
    axes.set_ylim(ylim)


def draw_input_disturbance_spec(
    axes: Axes,
    gamma_d: float,
    omega_d: float,
    system: cnt.TransferFunction,
    use_dB: bool = False,
):
    """
    Add disturbance input specification, defined by gamma_d,
    for remaining magnitude of error, and omega_d for the frequency
    at which it needs to happen. "system" is a transfer function
    object that represents the plant before any control is added.
    """
    mag, _, _ = cnt.frequency_response(system, omega=[omega_d])
    y_min = mag[0]
    y_max = mag[0] / gamma_d
    if use_dB:
        y_min = cnt.mag2db(y_min)
        y_max = cnt.mag2db(y_max)

    axes.vlines(omega_d, y_min, y_max, colors="green", label="$d_{in}$ spec")


def draw_ref_tracking_spec(axes: Axes, gamma_r: float, omega_r: float, use_dB=False):
    """
    Add reference tracking specification, defined by gamma_r for amount
    of remaining magnitude of error, and omega_r for the frequency at
    which it needs to happen.
    """
    xlim = axes.get_xlim()
    ylim = axes.get_ylim()

    omega_min = xlim[0]
    x_pts = [omega_min, omega_r]
    y_upper = [1 / gamma_r] * 2
    y_lower = [ylim[0]] * 2
    if use_dB:
        y_upper = cnt.mag2db(y_upper)
    axes.fill_between(
        x_pts, y_upper, y_lower, color="black", alpha=0.2, label="ref spec"
    )
    axes.set_xlim(xlim)
    axes.set_ylim(ylim)


def draw_step_tracking_spec(axes: Axes, gamma_r: float, use_dB=False):
    """
    Add step tracking constant error requirement, specified by gamma_r, (the
    amount of residual error), and a mag. ratio slope at low frequencies of 0.
    """
    w = np.logspace(-5, 0)
    if use_dB == False:
        axes.loglog(
            w,
            (1.0 / gamma_r - 1) * np.ones(np.size(w)),
            ".",
            color=[0, 0, 1],
            label="step tracking spec",
        )
    else:
        axes.semilogx(
            w,
            20.0 * np.log10(1 / gamma_r - 1) * np.ones(len(w)),
            ".",
            color=[0, 0, 1],
            label="step tracking spec",
        )


def draw_ramp_tracking_spec(axes: Axes, gamma_r: float, use_dB=False):
    """
    Add ramp tracking constant error requirement, specified by gamma_r, (the
    amount of residual error), and a mag. ratio slope at low frequencies of 1.
    """
    w = np.logspace(-5, 0)
    if use_dB == False:
        axes.loglog(
            w,
            np.divide(1.0 / gamma_r, w),
            ".",
            color=[0, 0, 1],
            label="ramp tracking spec",
        )
    else:
        axes.semilogx(
            w,
            20.0 * np.log10(np.divide(1.0 / gamma_r, w)),
            ".",
            color=[0, 0, 1],
            label="ramp tracking spec",
        )


def draw_parabola_tracking_spec(axes: Axes, gamma_r: float, use_dB=False):
    """
    Add parabola tracking constant error requirement, specified by gamma_r, (the
    amount of residual error), and a mag. ratio slope at low frequencies of 2.
    """
    w = np.logspace(-5, 0)
    if use_dB == False:
        axes.loglog(
            w,
            np.divide(1.0 / gamma_r, np.square(w)),
            ".",
            color=[0, 0, 1],
            label="parabola tracking spec",
        )
    else:
        axes.semilogx(
            w,
            20.0 * np.log10(np.divide(1.0 / gamma_r, np.square(w))),
            ".",
            color=[0, 0, 1],
            label="parabola tracking spec",
        )


def get_control_proportional(kp):
    """
    Generate transfer function for single gain or proportional control. This
    term can be used to change the crossover frequency.
    """

    return cnt.tf([kp], [1])


def get_control_integral(ki):
    """
    Generate transfer function for PI controller defined by gain "ki".
    Can be used to change the slope of the mag. ratio at low frequencies.
    """
    integrator = cnt.tf([1, ki], [1, 0])

    return integrator


def get_control_lag(z, M):
    """
    Makes a lag controller to add gain at low frequency such that:
        phase lag (|p|<|z|):
        M - low frequency gain
        z - frequency where effect roughly ends
    """
    lag = cnt.tf([1, z], [1, z / M])

    return lag


def get_control_lpf(p):
    """
    generates a transfer function for a low-pass filter with a
    cutoff frequency of "p"
    """
    lpf = cnt.tf(p, [1, p])

    return lpf


# phase lead (|p|>|z|): increase PM (stability)
# low frequency gain = K*z/p
# high frequency gain = K
def get_control_lead(omega_lead, M):
    """
    Generates a lead controller transfer function defined by:
        M - the amount of gain at high frequencies (corresponds to PM boost)
        omega_lead - frequency where max PM will be added
    """
    lead = cnt.tf([M, M * omega_lead / np.sqrt(M)], [1, omega_lead * np.sqrt(M)])

    return lead


def get_control_notch(ws, M):
    """
    Generates a notch filter defined by frequency of interest (ws) and gain M.
    """
    notch = cnt.tf([1, 2 * np.sqrt(M) * ws, M * ws**2], [1, (M + 1) * ws, M * ws**2])

    return notch


class DigitalFilter:
    def __init__(self, tf: cnt.TransferFunction, ts: float):
        sys_d = cnt.c2d(tf, ts, method="tustin")
        self.den_d = sys_d.den[0][0]
        self.num_d = sys_d.num[0][0]

        # internal state and input
        self.x_prev = np.zeros(len(self.num_d) - 1)
        self.u_prev = np.zeros(len(self.den_d))

    def update(self, u):
        # update vector with filter inputs (u)
        self.u_prev = np.hstack(([u], self.u_prev[:-1]))

        # use filter coefficients to calculate new output (y)
        y = self.num_d @ self.u_prev - self.den_d[1:] @ self.x_prev

        # update vector with filter outputs
        self.x_prev = np.hstack(([y], self.x_prev[:-1]))
        return y
