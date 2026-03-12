# 3rd-Party
import control as cnt
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies import A_arm


P = A_arm.params

use_dB = False
plant = cnt.tf(P.tf_num, P.tf_den)

pid = A_arm.ControllerPID()
kp, ki, kd, sigma = pid.kp, pid.ki, pid.kd, pid.sigma
controller = cnt.tf([kd + kp * sigma, kp + ki * sigma, ki], [sigma, 1, 0])

# open-loop system
sys_ol = cnt.series(controller, plant)

# closed-loop system
sys_cl = cnt.feedback(sys_ol, 1)


if __name__ == "__main__":
    # You could call bode() once with multiple systems, but it draws margins
    # for all of them. Here we only draw them for the open-loop system.
    # Calling bode() multiple times reuses the same axes, but removes the title
    # and legends, so we manually create the figures and axes to add them later.
    fig, ax = plt.subplots(2, sharex=True)
    cnt.bode(sys_ol, ax=ax, dB=use_dB, display_margins=True, label="open-loop")
    cnt.bode(sys_cl, ax=ax, dB=use_dB, grid=False, label="closed-loop")

    # cross-over frequency ≈ closed-loop bandwidth
    bandwidth_cl = cnt.bandwidth(sys_cl)
    print(f"Closed-loop bandwidth: {bandwidth_cl:.2f} rad/s")

    # Margin info is on the plot, so we don't really need to do this, but this
    # is how you can calculate it without plotting.
    gm, pm, wcg, wcp = cnt.margin(sys_ol)
    print(f"Crossover frequency: {wcp:.2f} rad/s")

    # draw bandwidth line
    ax[0].axvline(x=bandwidth_cl, color="red", linestyle=":", label="bandwidth")

    # manually add annotations after running bode() multiple times
    ax[0].legend()
    fig.suptitle("HW 17 Bode Plots: Robot Arm")
    fig.tight_layout()

    # keep figure open until closed by user
    plt.show()
