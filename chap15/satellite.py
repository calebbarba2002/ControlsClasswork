# 3rd-party
import control as cnt
import matplotlib.pyplot as plt

# local (controlbook)
from case_studies.C_satellite import params as P


use_dB = False
plant_inner = cnt.tf(P.tf_inner_num, P.tf_inner_den)
plant_outer = cnt.tf(P.tf_outer_num, P.tf_outer_den)


if __name__ == "__main__":
    cnt.bode(
        [plant_inner, plant_outer],
        dB=use_dB,
        label=[r"$P_{inner}(s)$", "$P_{outer}(s)$"],
        legend_loc="best",
        title="HW 15 Bode Plots: Satellite",
    )

    plt.show()
