# 3rd-Party
import numpy as np
import control as cnt
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import B_pendulum
from case_studies.common import loopshaping_tools as ls


P = B_pendulum.params

use_dB = False
plant_inner = cnt.tf(P.tf_inner_num, P.tf_inner_den)
plant_outer = cnt.tf(P.tf_outer_num, P.tf_outer_den)

###########################################################
## design controllers by loopshaping
###########################################################

# ----------- inner loop -----------
controller_inner = cnt.series(
    ls.get_control_proportional(kp=-200.0),
    ls.get_control_lead(omega_lead=40.0, M=15.0),
)

sys_inner_ol = cnt.series(controller_inner, plant_inner)
sys_inner_cl = cnt.feedback(sys_inner_ol, 1)

# ----------- outer loop -----------
# update the plant to include the inner loop
plant_outer = cnt.series(sys_inner_cl, plant_outer)

controller_outer = cnt.series(
    ls.get_control_proportional(kp=0.023),
    ls.get_control_lead(omega_lead=1.0, M=18.0),
    ls.get_control_lag(z=0.04, M=5.0),
    ls.get_control_lpf(p=50.0),
)

sys_outer_ol = cnt.series(controller_outer, plant_outer)
sys_outer_cl = cnt.feedback(sys_outer_ol, 1)


###########################################################
# add a prefilter to eliminate the overshoot
###########################################################
prefilter = ls.get_control_lpf(p=2.0)
assert prefilter is not None  # make type checkers happy


if __name__ == "__main__":
    ############################################
    # inner loop
    ############################################
    fig, ax = plt.subplots(2, sharex=True, figsize=(9, 7))
    w_lims = (1.0, 1e3)
    cnt.bode(plant_inner, ax=ax, dB=use_dB, omega_limits=w_lims, label="P(s)")
    cnt.bode(
        sys_inner_ol,
        ax=ax,
        dB=use_dB,
        omega_limits=w_lims,
        display_margins=True,
        label="C(s)P(s)",
    )
    cnt.bode(
        sys_inner_cl,
        ax=ax,
        dB=use_dB,
        omega_limits=w_lims,
        grid=False,
        label="closed-loop",
    )
    ax_mag = ax[0]

    ls.print_margins(sys_inner_ol, "Open-Loop (inner)", use_dB)

    # ----------- noise specification --------
    omega_n = 200.0
    gamma_n = 0.1
    ls.draw_noise_spec(ax_mag, gamma_n, omega_n, use_dB)

    # ----------- bandwidth --------
    bw_desired = 40.0
    bw_actual = cnt.bandwidth(sys_inner_cl)
    ax_mag.axvline(x=bw_actual, color="black", linestyle="-", label="bandwidth")
    ax_mag.axvline(x=bw_desired, color="red", linestyle="-.", label="bandwidth spec")

    title = ax_mag.get_title()
    idx = title.find("Gm")
    ax_mag.set_title(title[idx:], fontsize="medium")  # remove "sys[#]: " from title
    ax_mag.legend(ncol=2, loc="lower left", fontsize="small")
    fig.suptitle("Chap 18 Bode Plots: Pendulum on a Cart - Inner Loop")
    fig.tight_layout()
    plt.show()

    ############################################
    # outer loop
    ############################################
    fig, ax = plt.subplots(2, sharex=True, figsize=(9, 7))
    w_lims = (1e-3, 1e4)
    cnt.bode(plant_outer, ax=ax, dB=use_dB, omega_limits=w_lims, label="P(s)")
    cnt.bode(
        sys_outer_ol,
        ax=ax,
        dB=use_dB,
        omega_limits=w_lims,
        display_margins=True,
        label="C(s)P(s)",
    )
    # cnt.bode(
    #     sys_outer_cl,
    #     ax=ax,
    #     dB=use_dB,
    #     omega_limits=w_lims,
    #     grid=False,
    #     label="closed-loop",
    # )
    ax_mag = ax[0]

    ls.print_margins(sys_outer_ol, "Open-Loop (outer)", use_dB)

    # ----------- noise specification --------
    omega_n = 1000.0
    gamma_n = 0.0001
    ls.draw_noise_spec(ax_mag, gamma_n, omega_n, use_dB)

    # ----------- tracking specification --------
    omgega_r = 0.0032
    gamma_r = 0.00001
    ls.draw_ref_tracking_spec(ax_mag, gamma_r, omgega_r, use_dB)

    # ----------- crossover frequency --------
    tr_des = 2.0
    wc_des = 2.2 / tr_des
    # ax_mag.axvline(x=bw_actual, color="black", linestyle="-", label="bandwidth")
    ax_mag.axvline(x=wc_des, color="red", linestyle="-.", label=r"$\omega_c$ spec")

    title = ax_mag.get_title()
    idx = title.find("Gm")
    ax_mag.set_title(title[idx:], fontsize="medium")  # remove "sys[#]: " from title
    ax_mag.legend(ncol=2, loc="lower left", fontsize="small")
    fig.suptitle("Chap 18 Bode Plots: Pendulum on a Cart - Outer Loop")
    fig.tight_layout()
    plt.show()

    ############################################
    # now check the closed-loop response with prefilter
    ############################################
    # Closed loop transfer function from R to Y - no prefilter
    tf_R_to_Y_cl = cnt.feedback(sys_outer_ol, 1)
    # Closed loop transfer function from R to Y - with prefilter
    tf_R_to_Y_with_F_cl = cnt.series(prefilter, tf_R_to_Y_cl)
    # Closed loop transfer function from R to U - no prefilter
    tf_R_to_U_cl = cnt.feedback(controller_outer, plant_outer)
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
        title="Closed-Loop Bode Plots: Pendulum on a Cart",
    )
    plt.show()

    ## Step response plots
    fig3, axes3 = plt.subplots(2, sharex=True)
    fig3.suptitle("Closed-Loop Step Responses: Pendulum on a Cart")

    tf = 15.0
    time = np.linspace(0, tf, 100)

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
