#!/usr/bin/env python3
"""
Explanatory figure: why joint torque depends on how BENT the leg is.

A joint torque is a force times a lever arm. The force is fixed -- it is the
robot's weight divided by the feet carrying it. The lever arm is the horizontal
distance from the joint axis to the foot, and that is set entirely by how far
the leg is folded.

This draws the same leg carrying the same load at three degrees of bend, with
the lever arm and the resulting knee torque marked, so the relationship is
visible rather than tabulated.

Run:  python3 tools/make_torque_figure.py
"""
from __future__ import annotations

import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = os.path.join(ROOT, "ros2_ws/src/robodog_description/config")
OUT = os.path.join(ROOT, "docs/images/why_torque.png")


def main() -> int:
    P = yaml.safe_load(open(os.path.join(CFG, "robot_parameters.yaml")))
    RS = yaml.safe_load(open(os.path.join(CFG, "robstride02.yaml")))
    g = P["geometry"]
    L1, L2, R = g["thigh_length_m"], g["shank_length_m"], g["foot_radius_m"]
    load = P["mass_budget"]["total_kg"] * 9.81 / 4.0
    cont = RS["operational_limits"]["continuous_torque_nm"]

    heights = [0.24, 0.32, 0.40]
    titles = ["crouched", "nominal stance", "nearly straight"]
    fig, axes = plt.subplots(1, 3, figsize=(11.4, 5.0))

    for ax, h, title in zip(axes, heights, titles):
        reach = h - R
        c = (reach**2 - L1**2 - L2**2) / (2 * L1 * L2)
        q2 = -math.acos(max(-1.0, min(1.0, c)))
        q1 = -math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))

        # Draw with the GROUND as the shared reference, not the hip: the point
        # is that the body rises, so the ground must stay put across panels.
        hip = np.array([0.0, h - R])
        knee = hip + np.array([-L1 * math.sin(q1), -L1 * math.cos(q1)])
        foot = knee + np.array([-L2 * math.sin(q1 + q2), -L2 * math.cos(q1 + q2)])
        lever = abs(foot[0] - knee[0])
        tau = load * lever

        ax.axhline(0.0, color="#8a8f98", lw=2.0, zorder=1)
        ax.fill_between([-0.40, 0.40], -0.075, 0.0, color="#d8dce2", zorder=0)
        # body, to show the ride height changing
        ax.add_patch(plt.Rectangle((hip[0] - 0.055, hip[1] - 0.030), 0.185, 0.060,
                                   color="#2b3038", zorder=2))
        ax.annotate("", xy=(0.155, 0.0), xytext=(0.155, hip[1]),
                    arrowprops=dict(arrowstyle="<->", color="#6b7280", lw=1.2), zorder=5)
        ax.text(0.168, hip[1] / 2, f"{h*1000:.0f}\nmm", fontsize=8.5,
                color="#4a5568", va="center")

        ax.plot(*zip(hip, knee), color="#3d4450", lw=7, solid_capstyle="round", zorder=3)
        ax.plot(*zip(knee, foot), color="#5b6470", lw=6, solid_capstyle="round", zorder=3)
        for p, col, s in ((hip, "#f0894e", 190), (knee, "#f0894e", 170)):
            ax.scatter(*p, s=s, color=col, edgecolor="#8a4a20", zorder=4)
        ax.add_patch(plt.Circle(foot, R, color="#20242b", zorder=4))

        # the lever arm: horizontal distance from the knee axis to the foot
        ax.plot([knee[0], foot[0]], [knee[1], knee[1]], color="#d13b3b",
                lw=2.2, zorder=5)
        ax.plot([foot[0], foot[0]], [knee[1], foot[1]], color="#d13b3b",
                lw=1.0, ls=(0, (3, 3)), zorder=5)
        ax.annotate("", xy=(foot[0], knee[1]), xytext=(knee[0], knee[1]),
                    arrowprops=dict(arrowstyle="<->", color="#d13b3b", lw=1.6), zorder=6)
        ax.text((knee[0] + foot[0]) / 2, knee[1] + 0.021,
                f"lever arm\n{lever*1000:.0f} mm", ha="center", va="bottom",
                fontsize=9, color="#d13b3b", fontweight="bold")

        # the load, acting down through the foot
        ax.annotate("", xy=(foot[0], foot[1] - 0.075), xytext=(foot[0], foot[1] + 0.055),
                    arrowprops=dict(arrowstyle="-|>", color="#1f6feb", lw=2.2), zorder=6)
        ax.text(foot[0] + 0.016, foot[1] + 0.03, f"{load:.1f} N",
                fontsize=9, color="#1f6feb", ha="left")

        frac = tau / cont
        col = "#2a8a55" if frac < 0.7 else ("#b8860b" if frac < 1.0 else "#b0281a")
        ax.set_title(f"{title}\n"
                     f"bend {math.degrees(abs(q2)):.0f}\u00b0    "
                     f"{tau:.2f} N\u00b7m    {frac:.0%}",
                     fontsize=10.5, color=col, pad=8)
        ax.set_xlim(-0.30, 0.235)
        ax.set_ylim(-0.085, 0.455)
        ax.set_aspect("equal")
        ax.axis("off")

    fig.suptitle("Joint torque is set by how BENT the leg is, not by how long it is",
                 fontsize=13, fontweight="bold", y=0.99)
    fig.text(0.5, 0.025,
             "torque = load × lever arm.  The load is fixed by the robot's weight; "
             "the lever arm is the horizontal knee→foot distance,\nwhich shrinks as the "
             "leg straightens. Straighter legs also leave less travel to lift a foot with.",
             ha="center", fontsize=9, color="#4a5568")
    fig.tight_layout(rect=(0, 0.075, 1, 0.94))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=140, facecolor="white")
    plt.close(fig)
    print(f"wrote {os.path.relpath(OUT, ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
