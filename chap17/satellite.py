# 3rd-party
import control as cnt
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import C_satellite


P = C_satellite.params

use_dB = True
plant_inner = cnt.tf(P.tf_inner_num, P.tf_inner_den)
plant_outer = cnt.tf(P.tf_outer_num, P.tf_outer_den)

pid = C_satellite.ControllerPID()
kp_theta, kd_theta, sigma = pid.kp_theta, pid.kd_theta, pid.sigma
controller_inner = cnt.tf([kd_theta + kp_theta * sigma, kp_theta], [sigma, 1])

kp_phi, ki_phi, kd_phi = pid.kp_phi, pid.ki_phi, pid.kd_phi
controller_outer = cnt.tf(
    [kd_phi + kp_phi * sigma, kp_phi + ki_phi * sigma, ki_phi], [sigma, 1, 0]
)

# open-loop system
sys_inner_ol = cnt.series(controller_inner, plant_inner)
sys_outer_ol = cnt.series(controller_outer, plant_outer)

# closed-loop system
sys_inner_cl = cnt.feedback(sys_inner_ol, 1)
sys_outer_cl = cnt.feedback(sys_outer_ol, 1)


if __name__ == "__main__":
    # part a)

    # You could call bode() once with multiple systems, but it draws margins
    # for all of them. Here we only draw them for the open-loop system.
    # Calling bode() multiple times reuses the same axes, but removes the title
    # and legends, so we manually create the figures and axes to add them later.
    fig, ax = plt.subplots(2, sharex=True)
    cnt.bode(sys_inner_ol, ax=ax, dB=use_dB, display_margins=True, label="open-loop")
    cnt.bode(sys_inner_cl, ax=ax, dB=use_dB, grid=False, label="closed-loop")

    # cross-over frequency ≈ closed-loop bandwidth
    bandwidth_inner_cl = cnt.bandwidth(sys_inner_cl)
    print(f"\nClosed-loop bandwidth (inner): {bandwidth_inner_cl:.2f} rad/s")

    # Margin info is on the plot, so we don't really need to do this, but this
    # is how you can calculate it without plotting.
    gm, pm, wcg, wcp = cnt.margin(sys_inner_ol)
    print(f"Crossover frequency (inner): {wcp:.2f} rad/s")

    # draw bandwidth line
    ax[0].axvline(x=bandwidth_inner_cl, color="red", linestyle=":", label="bandwidth")

    # manually add annotations after running bode() multiple times
    ax[0].legend()
    fig.suptitle("HW 17 Bode Plots: Satellite - Inner Loop")
    fig.tight_layout()

    # keep figure open until closed by user
    plt.show()

    # part b)

    fig, ax = plt.subplots(2, sharex=True)
    cnt.bode(sys_outer_ol, ax=ax, dB=use_dB, display_margins=True, label="open-loop")
    cnt.bode(sys_outer_cl, ax=ax, dB=use_dB, grid=False, label="closed-loop")

    bandwidth_outer_cl = cnt.bandwidth(sys_outer_cl)
    print(f"\nClosed-loop bandwidth (outer): {bandwidth_outer_cl:.2f} rad/s")

    gm, pm, wcg, wcp = cnt.margin(sys_outer_ol)
    print(f"Crossover frequency (outer): {wcp:.2f} rad/s")

    ax[0].axvline(x=bandwidth_outer_cl, color="red", linestyle=":", label="bandwidth")
    ax[0].legend()
    fig.suptitle("HW 17 Bode Plots: Satellite - Outer Loop")
    fig.tight_layout()

    plt.show()

    # part c)

    bandwidth_separation = bandwidth_inner_cl / bandwidth_outer_cl
    print(f"\n{bandwidth_separation = :.2f}")
