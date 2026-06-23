"""
plot_F_window_sensitivity.py

Produces Figure 3.6 (fig:windows) and Table 3.2 (tab:windows) of the
thesis: the window-length sensitivity analysis for the vibration feature.

The script reads /imu/raw from a single bag, recomputes the vibration
feature std(|a|) offline for several candidate window sizes, plots the
resulting traces on a shared axis, and prints a metrics table summarising
quiescent noise, peak value, and peak time for each window size.

The window length W = 70 was selected at the end of this analysis as
the final implementation value used by vibration_feature_node.py.
See:
  - Chapter 3, Section 3.6.3 (window length selection)
  - Chapter 4, Section 4.3.3 (iterative tuning of the window length)

REQUIRES: a recorded bag file with a clearly visible disturbance event.
EDIT BAG_NAME below to pick the run for analysis.
"""

import os
from collections import deque
import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


# ============================================================
# CONFIGURATION -- EDIT THIS BLOCK
# ============================================================
# Bag from which to read /imu/raw.  Pick a run with a clear disturbance
# event so that all six window-size traces show a visible peak.
BAG_NAME = '20260519_thesis_disturbance_fuzzy_on_01'

# Window sizes to evaluate.  W = 70 is the value retained in the final
# implementation (Section 3.6.3).
WINDOW_SIZES = [20, 30, 50, 70, 100, 150]

# Quiescent reference interval, in seconds from the start of the bag.
# Used to compute the quiescent-noise metric below.  The runs of
# Chapter 5 always begin with several seconds of undisturbed motion
# before the operator applies the tapping disturbance (see the protocol
# in Section 5.1.2), so the first 4 s is a safe quiescent window.  If
# the bag layout is changed, adjust this value.
QUIESCENT_END = 4.0
# ============================================================


G = 9.81


# ---------- Paths (resolved relative to this script's location) ----------
# Assumes the repo layout: <repo>/analysis/<this>.py and <repo>/bags/<bag>.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
BAG_ROOT = os.path.join(REPO_ROOT, 'bags')
OUT_DIR = os.path.join(SCRIPT_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

bag_path = os.path.join(BAG_ROOT, BAG_NAME)


def read_imu(bag_path):
    """Read /imu/raw from the bag and return arrays of timestamps (ns)
    and |a| in g.  The g-conversion mirrors the live behavior of
    vibration_feature_node.py (m/s^2 -> g divides by 9.81)."""
    storage = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter = ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage, converter)

    t_list, mag_list = [], []
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic == '/imu/raw':
            msg = deserialize_message(raw, Imu)
            ax = msg.linear_acceleration.x / G
            ay = msg.linear_acceleration.y / G
            az = msg.linear_acceleration.z / G
            mag = float(np.sqrt(ax * ax + ay * ay + az * az))
            t_list.append(t_ns)
            mag_list.append(mag)
    return np.array(t_list, dtype=np.int64), np.array(mag_list)


def sliding_std(mags, window_size):
    """Reproduce vibration_feature_node.py exactly: maintain a deque of
    window_size samples, emit np.std(deque) only when the buffer is full
    (matching the startup-delay behavior documented in Section 4.3.1)."""
    buf = deque(maxlen=window_size)
    out = np.full(len(mags), np.nan)
    for i, m in enumerate(mags):
        buf.append(m)
        if len(buf) == window_size:
            out[i] = float(np.std(buf))
    return out


# ---------- Read the bag once ----------
print(f'Reading IMU from: {bag_path}')
t_ns, mags = read_imu(bag_path)
print(f'  {len(mags)} IMU samples')

t0 = t_ns[0]
t_s = (t_ns - t0) * 1e-9


# ---------- Compute the feature for each candidate window size ----------
features = {W: sliding_std(mags, W) for W in WINDOW_SIZES}


# ---------- Quantitative metrics for Table 3.2 (Section 3.6.3) ----------
# Quiescent noise: std of the feature itself over QUIESCENT_END seconds.
# Peak value: max of the feature across the whole run.
# Peak time: time at which the peak occurs (seconds from bag start).
mask_quiescent = t_s < QUIESCENT_END

print('\nWindow size sensitivity metrics (Table 3.2 of the thesis):')
print(f'{"W":>5} | {"quiescent std":>14} | {"peak value":>10} | {"peak time [s]":>14}')
print('-' * 56)
for W in WINDOW_SIZES:
    f = features[W]
    valid = ~np.isnan(f)
    quiescent_noise = (np.nanstd(f[mask_quiescent & valid])
                       if np.any(mask_quiescent & valid) else np.nan)
    peak_val = np.nanmax(f) if np.any(valid) else np.nan
    peak_idx = np.nanargmax(f) if np.any(valid) else 0
    peak_time = t_s[peak_idx]
    print(f'{W:>5} | {quiescent_noise:>14.5f} | {peak_val:>10.4f} | {peak_time:>14.2f}')


# ---------- Plot Figure 3.6 ----------
fig, ax = plt.subplots(figsize=(11, 5))
colors = plt.cm.viridis(np.linspace(0.15, 0.85, len(WINDOW_SIZES)))
for W, c in zip(WINDOW_SIZES, colors):
    ax.plot(t_s, features[W], color=c, linewidth=1.2, label=f'W = {W}')

# Dashed reference lines at the LOW upper bound and the HIGH lower bound
# of the input membership functions (Section 3.7.2).
ax.axhline(0.045, color='gray', linestyle=':', linewidth=0.8, alpha=0.6,
           label='LOW upper bound')
ax.axhline(0.070, color='gray', linestyle='--', linewidth=0.8, alpha=0.6,
           label='HIGH lower bound')

ax.set_xlabel('Time  [s]')
ax.set_ylabel(r'Feature  $v = \mathrm{std}(|a|)$  [g]')
ax.set_title(f'Window-size sensitivity ({BAG_NAME})')
ax.grid(alpha=0.3)
ax.legend(loc='upper right', ncol=2)

plt.tight_layout()
out_png = os.path.join(OUT_DIR, 'plot_F_window_sensitivity.png')
out_pdf = os.path.join(OUT_DIR, 'plot_F_window_sensitivity.pdf')
plt.savefig(out_png, dpi=200, bbox_inches='tight')
plt.savefig(out_pdf, bbox_inches='tight')
print(f'\nSaved: {out_png}')
print(f'Saved: {out_pdf}')
plt.show()
