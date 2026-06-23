"""
plot_timeseries.py

Produces the four time-series figures of Chapter 5:
  - Figure 5.2 (fig:baseline)  -- baseline behaviour, fuzzy-ON quiescent interval
  - Figure 5.3 (fig:fuzzyon)   -- representative fuzzy-ON linear run
  - Figure 5.4 (fig:angular)   -- representative fuzzy-ON angular run
  - Figure 5.5 (fig:fuzzyoff)  -- representative fuzzy-OFF linear run

Each figure has four panels from top to bottom:
  (a) vibration feature v = std(|a|)
  (b) reconstructed scaling factor alpha
  (c) operator-commanded velocity (linear.x or angular.z)
  (d) executed velocity at the robot (linear.x or angular.z)

The alpha trace is reconstructed offline by re-applying the Mamdani
inference to the recorded /vibration_feature.  This is mathematically
identical to the live alpha signal because the controller is
deterministic and stateless; see Section 5.1.4 of the thesis.

See:
  - Chapter 5, Section 5.2 (baseline characterisation)
  - Chapter 5, Section 5.3 (fuzzy-ON behaviour, linear and angular)
  - Chapter 5, Section 5.4.1 (fuzzy-OFF baseline)

REQUIRES a single bag file per figure.  EDIT BAG_NAME, MODE, and
PLOT_LABEL below to switch between the four figures.
"""

import os
import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist


# ============================================================
# CONFIGURATION -- EDIT THESE THREE LINES TO SWITCH FIGURES
# ============================================================
# Uncomment exactly one BAG_NAME line at a time:
#
# Figure 5.2 / 5.3 (baseline + representative fuzzy-ON, linear):
# BAG_NAME = '20260519_thesis_disturbance_fuzzy_on_01'
# BAG_NAME = '20260519_thesis_disturbance_fuzzy_on_02'
# BAG_NAME = '20260519_thesis_disturbance_fuzzy_on_03'
#
# Figure 5.5 (representative fuzzy-OFF, linear):
# BAG_NAME = '20260519_thesis_disturbance_fuzzy_off_01'
# BAG_NAME = '20260519_thesis_disturbance_fuzzy_off_02'
# BAG_NAME = '20260519_thesis_disturbance_fuzzy_off_03'
#
# Figure 5.4 (representative fuzzy-ON, angular):
# BAG_NAME = '20260519_thesis_disturbance_angular_fuzzy_on_01'
BAG_NAME = '20260519_thesis_disturbance_angular_fuzzy_on_02'
# BAG_NAME = '20260519_thesis_disturbance_angular_fuzzy_on_03'

# 'linear'  for Figures 5.2, 5.3, 5.5
# 'angular' for Figure 5.4
MODE = 'angular'

# Used as the output filename suffix.  Examples:
#   'baseline_fuzzy_on_02'  -> Figure 5.2
#   'C_fuzzy_on_01'         -> Figure 5.3
#   'E_angular_02'          -> Figure 5.4
#   'D_fuzzy_off_02'        -> Figure 5.5
PLOT_LABEL = 'E_angular_02'
# ============================================================


# ---------- Paths (resolved relative to this script's location) ----------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
BAG_ROOT = os.path.join(REPO_ROOT, 'bags')
OUT_DIR = os.path.join(SCRIPT_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

bag_path = os.path.join(BAG_ROOT, BAG_NAME)


# ---------- Fuzzy controller (must match fuzzy_alpha_node.py exactly) ----------
# Section 3.7 (conceptual), Section 4.4.1 (numerical implementation).

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


def mu_vibration_low(v):
    return trapezoidal(v, 0.000, 0.000, 0.030, 0.045)


def mu_vibration_medium(v):
    return triangular(v, 0.035, 0.055, 0.085)


def mu_vibration_high(v):
    if v >= 0.2:
        return 1.0
    return trapezoidal(v, 0.070, 0.090, 0.20, 0.20)


def mu_speed_slow(s):
    return trapezoidal(s, 0.40, 0.40, 0.50, 0.65)


def mu_speed_medium(s):
    return triangular(s, 0.55, 0.72, 0.85)


def mu_speed_fast(s):
    return trapezoidal(s, 0.75, 0.90, 1.00, 1.00)


def fuzzy_controller(v):
    """Mamdani inference, identical to the live fuzzy_alpha_node.py.
    Deterministic and stateless, so re-applying it to the recorded
    /vibration_feature reproduces the live alpha exactly (Section 5.1.4)."""
    mu_low = mu_vibration_low(v)
    mu_med = mu_vibration_medium(v)
    mu_high = mu_vibration_high(v)
    rule_fast, rule_medium, rule_slow = mu_low, mu_med, mu_high
    s_values = np.linspace(0.4, 1.0, 1000)
    slow_c = np.array([min(rule_slow, mu_speed_slow(s)) for s in s_values])
    med_c = np.array([min(rule_medium, mu_speed_medium(s)) for s in s_values])
    fast_c = np.array([min(rule_fast, mu_speed_fast(s)) for s in s_values])
    aggregated = np.maximum.reduce([slow_c, med_c, fast_c])
    if np.sum(aggregated) == 0:
        return 0.4
    return float(np.sum(s_values * aggregated) / np.sum(aggregated))


# ---------- Bag reader ----------

def read_bag(bag_path):
    """Return a dict {topic: {'t': [...], 'msg': [...]}} for the four
    topics needed by the four-panel figure."""
    storage = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter = ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage, converter)

    type_map = {
        '/vibration_feature': Float32,
        '/imu/raw': Imu,
        '/cmd_vel_user': Twist,
        '/S2R5/cmd_vel': Twist,
    }

    data = {topic: {'t': [], 'msg': []} for topic in type_map}

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic in type_map:
            msg = deserialize_message(raw, type_map[topic])
            data[topic]['t'].append(t_ns)
            data[topic]['msg'].append(msg)

    return data


print(f'Reading bag: {bag_path}')
data = read_bag(bag_path)

for topic in data:
    print(f'  {topic}: {len(data[topic]["t"])} messages')


# ---------- Build numpy arrays with a common time origin ----------
# t = 0 corresponds to the earliest timestamp across all four topics.
all_t = []
for topic in data:
    all_t.extend(data[topic]['t'])
t0_ns = min(all_t) if all_t else 0


def to_seconds(t_list):
    return (np.array(t_list, dtype=np.int64) - t0_ns) * 1e-9


# Vibration feature.
t_vib = to_seconds(data['/vibration_feature']['t'])
v_vib = np.array([m.data for m in data['/vibration_feature']['msg']])

# Reconstruct alpha offline from the recorded vibration feature
# (Section 5.1.4).
alpha = np.array([fuzzy_controller(v) for v in v_vib])

# Operator command.  Pre-pend a synthetic (t = 0, value = 0) sample so
# the plot trace starts cleanly at zero before the first real command,
# rather than starting wherever the first published Twist appeared.
t_user_raw = to_seconds(data['/cmd_vel_user']['t'])
if MODE == 'linear':
    cmd_user_raw = np.array([m.linear.x for m in data['/cmd_vel_user']['msg']])
else:
    cmd_user_raw = np.array([m.angular.z for m in data['/cmd_vel_user']['msg']])

if len(t_user_raw) > 0 and t_user_raw[0] > 0:
    epsilon = 1e-3  # 1 ms before first real sample
    t_user = np.concatenate([[0.0, t_user_raw[0] - epsilon], t_user_raw])
    cmd_user_raw = np.concatenate([[0.0, 0.0], cmd_user_raw])
else:
    t_user = t_user_raw

# Executed velocity at the robot.
t_final = to_seconds(data['/S2R5/cmd_vel']['t'])
if MODE == 'linear':
    cmd_final_raw = np.array([m.linear.x for m in data['/S2R5/cmd_vel']['msg']])
else:
    cmd_final_raw = np.array([m.angular.z for m in data['/S2R5/cmd_vel']['msg']])

# For angular mode, plot the magnitude (sign inverted from the recorded
# negative angular.z of a left-hand turn) so that "smaller magnitude"
# reads as "down" in the panel.  This matches Figure 5.4 of the thesis.
if MODE == 'angular':
    cmd_user = -cmd_user_raw
    cmd_final = -cmd_final_raw
    axis_label = '|angular.z|  [rad/s]'
else:
    cmd_user = cmd_user_raw
    cmd_final = cmd_final_raw
    axis_label = 'linear.x  [m/s]'


# ---------- Plot: four-panel stacked time series ----------
fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)

# Panel (a): vibration feature.
axes[0].plot(t_vib, v_vib, color='#1f77b4', linewidth=1.2)
axes[0].set_ylabel(r'$v = \mathrm{std}(|a|)$  [g]')
axes[0].set_title(f'{BAG_NAME}  ({MODE} mode)')
axes[0].grid(alpha=0.3)
# Reference lines at the LOW upper bound and the HIGH lower bound of
# the input membership functions (Section 3.7.2).
axes[0].axhline(0.045, color='gray', linestyle=':', linewidth=0.8, alpha=0.6)
axes[0].axhline(0.070, color='gray', linestyle=':', linewidth=0.8, alpha=0.6)

# Panel (b): reconstructed alpha.
axes[1].plot(t_vib, alpha, color='#d62728', linewidth=1.2)
axes[1].set_ylabel(r'$\alpha$')
axes[1].set_ylim(0.35, 1.05)
axes[1].grid(alpha=0.3)
axes[1].axhline(1.0, color='gray', linestyle=':', linewidth=0.8, alpha=0.6)

# Panel (c): operator-commanded velocity.
axes[2].plot(t_user, cmd_user, color='#2ca02c', linewidth=1.2)
axes[2].set_ylabel(f'/cmd_vel_user\n{axis_label}')
axes[2].grid(alpha=0.3)

# Panel (d): executed velocity at the robot.
axes[3].plot(t_final, cmd_final, color='#ff7f0e', linewidth=1.2)
axes[3].set_ylabel(f'/S2R5/cmd_vel\n{axis_label}')
axes[3].set_xlabel('Time  [s]')
axes[3].grid(alpha=0.3)

# Match y-limits between panels (c) and (d) so that the visual
# comparison between commanded and executed velocity is fair.
y_min = min(cmd_user.min() if len(cmd_user) else 0,
            cmd_final.min() if len(cmd_final) else 0)
y_max = max(cmd_user.max() if len(cmd_user) else 0,
            cmd_final.max() if len(cmd_final) else 0)
pad = 0.05 * max(abs(y_min), abs(y_max), 0.01)
axes[2].set_ylim(y_min - pad, y_max + pad)
axes[3].set_ylim(y_min - pad, y_max + pad)
if MODE == 'angular':
    y_pad_min = -0.02
    y_pad_max = 0.05 + max(abs(cmd_user).max(), abs(cmd_final).max())
    axes[2].set_ylim(y_pad_min, y_pad_max)
    axes[3].set_ylim(y_pad_min, y_pad_max)

plt.tight_layout()
out_png = os.path.join(OUT_DIR, f'plot_{PLOT_LABEL}.png')
out_pdf = os.path.join(OUT_DIR, f'plot_{PLOT_LABEL}.pdf')
plt.savefig(out_png, dpi=200, bbox_inches='tight')
plt.savefig(out_pdf, bbox_inches='tight')
print(f'Saved: {out_png}')
print(f'Saved: {out_pdf}')
plt.show()
