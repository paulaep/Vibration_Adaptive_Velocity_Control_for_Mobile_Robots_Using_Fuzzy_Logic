"""
plot_B_alpha_curve.py

Produces Figure 3.8 (fig:staticmap) of the thesis: the static input-output
characteristic alpha(v) of the fuzzy controller.

The full Mamdani inference (fuzzification, rule evaluation, clipping,
aggregation, centroid defuzzification) is reproduced here, identically
to the live implementation in fuzzy_alpha_node.py, and the resulting
alpha is computed for v swept across the input range [0, 0.20] g.

See:
  - Chapter 3, Section 3.7 (Mamdani inference)
  - Chapter 3, Section 3.7.6 (static input-output mapping)

No bag file is required.  Outputs are written to <repo>/analysis/plots/.
"""

import os
import numpy as np
import matplotlib.pyplot as plt


# ---------- Paths ----------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)


# ---------- Membership-function primitives ----------
# (same as in fuzzy_alpha_node.py)

def triangular(x, a, b, c):
    if x <= a or x >= c:
        return 0.0
    elif x == b:
        return 1.0
    elif x < b:
        return (x - a) / (b - a)
    else:
        return (c - x) / (c - b)


def trapezoidal(x, a, b, c, d):
    if x <= a or x >= d:
        return 0.0
    elif b <= x <= c:
        return 1.0
    elif a < x < b:
        return (x - a) / (b - a)
    else:
        return (d - x) / (d - c)


# ---------- Input MFs over the vibration feature v (Section 3.7.2) ----------

def mu_vibration_low(v):
    return trapezoidal(v, 0.000, 0.000, 0.030, 0.045)


def mu_vibration_medium(v):
    return triangular(v, 0.035, 0.055, 0.085)


def mu_vibration_high(v):
    # Explicit saturation for v >= 0.2 (Section 3.7.2).
    if v >= 0.2:
        return 1.0
    return trapezoidal(v, 0.070, 0.090, 0.20, 0.20)


# ---------- Output MFs over the scaling factor alpha (Section 3.7.3) ----------

def mu_speed_slow(s):
    return trapezoidal(s, 0.40, 0.40, 0.50, 0.65)


def mu_speed_medium(s):
    return triangular(s, 0.55, 0.72, 0.85)


def mu_speed_fast(s):
    return trapezoidal(s, 0.75, 0.90, 1.00, 1.00)


def fuzzy_controller(vibration_input):
    """Mamdani inference, identical to fuzzy_alpha_node.fuzzy_controller.
    See Section 3.7 for the conceptual description and Section 4.4.1
    for the numerical implementation."""
    # Step 1: fuzzification (Section 3.7.1)
    mu_low = mu_vibration_low(vibration_input)
    mu_med = mu_vibration_medium(vibration_input)
    mu_high = mu_vibration_high(vibration_input)

    # Step 2: rule evaluation -- Table 3.3 (Section 3.7.4)
    rule_fast = mu_low      # IF v is LOW    THEN alpha is FAST
    rule_medium = mu_med    # IF v is MEDIUM THEN alpha is MEDIUM
    rule_slow = mu_high     # IF v is HIGH   THEN alpha is SLOW

    # Step 3-4: clipping + aggregation (Section 3.7.4)
    s_values = np.linspace(0.4, 1.0, 1000)
    slow_c = np.array([min(rule_slow, mu_speed_slow(s)) for s in s_values])
    med_c = np.array([min(rule_medium, mu_speed_medium(s)) for s in s_values])
    fast_c = np.array([min(rule_fast, mu_speed_fast(s)) for s in s_values])

    aggregated = np.maximum.reduce([slow_c, med_c, fast_c])

    # Step 5: centroid defuzzification (Section 3.7.5, equation 3.X)
    if np.sum(aggregated) == 0:
        return 0.4
    return float(np.sum(s_values * aggregated) / np.sum(aggregated))


# ---------- Sweep v across the input range and compute alpha ----------
v_sweep = np.linspace(0, 0.20, 400)
alpha_sweep = np.array([fuzzy_controller(v) for v in v_sweep])


# ---------- Plot ----------
fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(v_sweep, alpha_sweep, linewidth=2.5, color='#1f77b4')
ax.set_xlabel(r'Vibration feature $v = \mathrm{std}(|a|)$  [g]')
ax.set_ylabel(r'Speed scale $\alpha$')
ax.set_title('Fuzzy controller static input-output mapping')
ax.set_xlim(0, 0.20)
ax.set_ylim(0.35, 1.05)
ax.grid(alpha=0.3)

# Shade the input fuzzy-set regions for context (Section 3.7.6).
ax.axvspan(0.0, 0.045, alpha=0.08, color='green', label='LOW region')
ax.axvspan(0.035, 0.085, alpha=0.08, color='orange', label='MEDIUM region')
ax.axvspan(0.070, 0.20, alpha=0.08, color='red', label='HIGH region')
ax.legend(loc='lower left')

plt.tight_layout()
out_png = os.path.join(OUT_DIR, 'plot_B_alpha_curve.png')
out_pdf = os.path.join(OUT_DIR, 'plot_B_alpha_curve.pdf')
plt.savefig(out_png, dpi=200, bbox_inches='tight')
plt.savefig(out_pdf, bbox_inches='tight')
print(f'Saved: {out_png}')
print(f'Saved: {out_pdf}')
plt.show()
