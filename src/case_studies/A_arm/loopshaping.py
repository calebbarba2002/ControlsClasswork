# 3rd-Party
import numpy as np
import control as cnt
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import A_arm
from case_studies.common import loopshaping_tools as ls


P = A_arm.params

use_dB = False
plant = cnt.tf(P.tf_num, P.tf_den)

pid = A_arm.ControllerPID()
kp, ki, kd, sigma = pid.kp, pid.ki, pid.kd, pid.sigma
controller = cnt.tf([kd + kp * sigma, kp + ki * sigma, ki], [sigma, 1, 0])

# original (PID) open-loop system
sys_orig_ol = cnt.series(controller, plant)

###########################################################
## design final controller by loopshaping
###########################################################

# Because our PM starts out good enough, we will skip to adding a
# low-pass filter and lag compensator to meet the low-frequency and
# high-frequency requirements.
C_lpf = ls.get_control_lpf(90.0)
C_lag = ls.get_control_lag(z=5, M=90.0)
controller = cnt.series(controller, C_lpf, C_lag)

# after checking the requirements, we need to add a lead compensator now,
# along with a proportional gain and 2nd low-pass filter to meet the noise
# specification.
C_lead = ls.get_control_lead(omega_lead=10, M=10.0)
controller = cnt.series(controller, C_lead)

mag, _, _ = cnt.frequency_response(sys_orig_ol, omega=[6.35])
C_k = ls.get_control_proportional(1 / mag[0])
C_lpf2 = ls.get_control_lpf(100.0)

# this is our final controller
controller = cnt.series(controller, C_k, C_lpf2)

sys_final_ol = cnt.series(controller, plant)

###########################################################
# add a prefilter to eliminate the overshoot
###########################################################
prefilter = ls.get_control_lpf(p=2.0)
assert prefilter is not None  # make type checkers happy


if __name__ == "__main__":
    cplt = cnt.bode(
        [sys_orig_ol, sys_final_ol],
        dB=use_dB,
        omega_limits=[10**-3, 10**5],
        display_margins=False,
        label=["$C_{pid}(s)P(s)$", "$C_{final}(s)P(s)$"],
        title="Open-Loop Bode Plots: Single Link Arm",
    )
    ax_mag = cplt.axes[0, 0]

    ls.print_margins(sys_orig_ol, "Original (C_pid) Open-Loop", use_dB)
    ls.print_margins(sys_final_ol, "Final (C_final) Open-Loop", use_dB)

    # ----------- noise specification --------
    omega_n = 1000
    improvement_factor = 10.0

    mag, _, _ = cnt.frequency_response(sys_orig_ol, omega=[omega_n])
    gamma_n = mag[0] / improvement_factor
    ls.draw_noise_spec(ax_mag, gamma_n, omega_n, use_dB)

    # ----------- general tracking specification --------
    omega_d = 0.07
    improvement_factor = 10.0
    gamma_d = 1 / improvement_factor
    ls.draw_input_disturbance_spec(ax_mag, gamma_d, omega_d, sys_orig_ol, use_dB)

    ax_mag.legend()
    plt.show()

    ############################################
    # now check the closed-loop response with prefilter
    ############################################
    # Closed loop transfer function from R to Y - no prefilter
    tf_R_to_Y_cl = cnt.feedback(sys_final_ol, 1)
    # Closed loop transfer function from R to Y - with prefilter
    tf_R_to_Y_with_F_cl = cnt.series(prefilter, tf_R_to_Y_cl)
    # Closed loop transfer function from R to U - no prefilter
    tf_R_to_U_cl = cnt.feedback(controller, plant)  # type: ignore
    # Closed loop transfer function from R to U - with prefilter
    tf_R_to_U_with_F_cl = cnt.series(prefilter, tf_R_to_U_cl)

    cnt.bode(
        [tf_R_to_Y_cl, tf_R_to_Y_with_F_cl],
        dB=use_dB,
        # plot_phase=False,
        label=[
            r"Closed-Loop $\frac{Y}{R}$ - no pre-filter",
            r"Closed-Loop $\frac{Y}{R}$ - with pre-filter",
        ],
        title="Closed-Loop Bode Plots: Single Link Arm",
    )
    plt.show()

    ## Step response plots
    fig3, axes3 = plt.subplots(2, sharex=True)
    fig3.suptitle("Closed-Loop Step Responses: Single Link Arm")

    time = np.linspace(0, 2, 100)

    ax = axes3[0]
    _, y_out_no_F = cnt.step_response(tf_R_to_Y_cl, time)
    _, y_out_F = cnt.step_response(tf_R_to_Y_with_F_cl, time)
    ax.plot(time, y_out_no_F, label="no prefilter")
    ax.plot(time, y_out_F, label="with prefilter")
    ax.legend()
    ax.grid(True)
    ax.set_ylabel("Step Response")

    ax = axes3[1]
    _, u_out_no_F = cnt.step_response(tf_R_to_U_cl, time)
    _, u_out_F = cnt.step_response(tf_R_to_U_with_F_cl, time)
    ax.plot(time, u_out_no_F, label="no prefilter")
    ax.plot(time, u_out_F, label="with prefilter")
    ax.set_ylabel("Control Effort")
    ax.set_xlabel("Time (s)")
    ax.grid(True)

    plt.show()
