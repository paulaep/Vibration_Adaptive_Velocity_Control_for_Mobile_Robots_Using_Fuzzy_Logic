"""
plot_A_membership_functions.py

Produces Figure 3.7 (fig:memberships) of the thesis: the input and output
membership functions of the Mamdani fuzzy controller.

The membership-function parameter values plotted here are the same
values used by the live controller in fuzzy_alpha_node.py.  See:
  - Chapter 3, Section 3.7.2 (input membership functions)
  - Chapter 3, Section 3.7.3 (output membership functions)

No bag file is required.  Outputs are written to <repo>/analysis/plots/.
Run from any working directory with matplotlib + numpy installed.
"""

import os
import numpy as np
import matplotlib.pyplot as plt


# ---------- Paths (resolved relative to this script's location) ----------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)


# ---------- Membership-function primitives (Sections 3.7.2 / 3.7.3) ----------

def triangular(x, a, b, c):
    """Triangular MF tri(x; a, b, c) -- Section 3.7.2."""
    y = np.zeros_like(x)
    left = (x > a) & (x < b)
    right = (x >= b) & (x < c)
    y[left] = (x[left] - a) / (b - a)
    y[right] = (c - x[right]) / (c - b)
    y[x == b] = 1.0
    return y


def trapezoidal(x, a, b, c, d):
    """Trapezoidal MF trap(x; a, b, c, d) -- Sections 3.7.2 / 3.7.3."""
    y = np.zeros_like(x)
    left = (x > a) & (x < b)
    flat = (x >= b) & (x <= c)
    right = (x > c) & (x < d)
    y[left] = (x[left] - a) / (b - a)
    y[flat] = 1.0
    y[right] = (d - x[right]) / (d - c)
    return y


# ---------- Input membership functions over v = std(|a|) in g ----------
# Parameter values: Section 3.7.2 (equations 3.X-3.X).  Must match
# fuzzy_alpha_node.py exactly.
v = np.linspace(0, 0.25, 2000)

mu_low = trapezoidal(v, 0.000, 0.000, 0.030, 0.045)
mu_med = triangular(v, 0.035, 0.055, 0.085)
mu_high = trapezoidal(v, 0.070, 0.090, 0.20, 0.20)
mu_high[v >= 0.20] = 1.0  # Explicit saturation (Section 3.7.2)


# ---------- Output membership functions over alpha in [0.4, 1.0] ----------
# Parameter values: Section 3.7.3 (equations 3.X-3.X).  Must match
# fuzzy_alpha_node.py exactly.
s = np.linspace(0.4, 1.0, 2000)

mu_slow = trapezoidal(s, 0.40, 0.40, 0.50, 0.65)
mu_med_out = triangular(s, 0.55, 0.72, 0.85)
mu_fast = trapezoidal(s, 0.75, 0.90, 1.00, 1.00)


# ---------- Plot: two-panel figure, input MFs on the left, output on the right ----------
fig, axes = plt.subplots(1, 2, figsize=(12, 4))

axes[0].plot(v, mu_low, label='LOW', linewidth=2)
axes[0].plot(v, mu_med, label='MEDIUM', linewidth=2)
axes[0].plot(v, mu_high, label='HIGH', linewidth=2)
axes[0].set_xlabel(r'Vibration feature $v = \mathrm{std}(|a|)$  [g]')
axes[0].set_ylabel('Membership degree')
axes[0].set_title('Input membership functions')
axes[0].set_xlim(0, 0.20)
axes[0].set_ylim(0, 1.05)
axes[0].grid(alpha=0.3)
axes[0].legend()

axes[1].plot(s, mu_slow, label='SLOW', linewidth=2)
axes[1].plot(s, mu_med_out, label='MEDIUM', linewidth=2)
axes[1].plot(s, mu_fast, label='FAST', linewidth=2)
axes[1].set_xlabel(r'Speed scale $\alpha$')
axes[1].set_ylabel('Membership degree')
axes[1].set_title('Output membership functions')
axes[1].set_xlim(0.4, 1.0)
axes[1].set_ylim(0, 1.05)
axes[1].grid(alpha=0.3)
axes[1].legend()

plt.tight_layout()
out_png = os.path.join(OUT_DIR, 'plot_A_membership_functions.png')
out_pdf = os.path.join(OUT_DIR, 'plot_A_membership_functions.pdf')
plt.savefig(out_png, dpi=200, bbox_inches='tight')
plt.savefig(out_pdf, bbox_inches='tight')
print(f'Saved: {out_png}')
print(f'Saved: {out_pdf}')
plt.show()
