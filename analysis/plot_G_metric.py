"""
plot_G_metric.py

Produces Figure 5.6 (fig:aggregate) and Table 5.2 (tab:results) of the
thesis: the aggregate SRR comparison between the fuzzy-ON and fuzzy-OFF
linear-motion runs, and the per-run metrics table.

For each of the six linear runs (three fuzzy-ON, three fuzzy-OFF) the
script:
  1. Reads the bag.
  2. Reconstructs alpha offline by re-applying the Mamdani inference
     to the recorded /vibration_feature (Section 5.1.4 explains why
     this is mathematically identical to the live alpha signal).
  3. Identifies the disturbance interval as samples where the feature
     exceeds the disturbance threshold (Section 5.4.2).
  4. Computes:
       - SRR = mean(/S2R5/cmd_vel) / mean(/cmd_vel_user), over the
         disturbance interval, restricted to samples where the operator
         is actually commanding motion.  Equation (5.X) (Section 5.4.2).
       - mean alpha during the disturbance interval.
       - mean alpha during the quiescent interval (v <= threshold).

The script also computes a "reaction latency" (the time between v first
crossing the disturbance threshold and alpha first dropping below
ALPHA_REACTION_THRESHOLD) as a diagnostic.  This metric is printed but
is NOT reported in the thesis; the thesis discusses SRR and mean alpha
(Section 5.4.3).

See:
  - Chapter 5, Section 5.4 (fuzzy ON vs OFF comparison)
  - Chapter 5, Section 5.4.2 (metric definitions)
  - Chapter 5, Section 5.4.3 (quantitative results)
"""

import os
import numpy as np
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


# ============================================================
# CONFIGURATION
# ============================================================
ON_BAGS = [
    '20260519_thesis_disturbance_fuzzy_on_01',
    '20260519_thesis_disturbance_fuzzy_on_02',
    '20260519_thesis_disturbance_fuzzy_on_03',
]
OFF_BAGS = [
    '20260519_thesis_disturbance_fuzzy_off_01',
    '20260519_thesis_disturbance_fuzzy_off_02',
    '20260519_thesis_disturbance_fuzzy_off_03',
]

# Disturbance threshold v > 0.05 g.  Matches the boundary between LOW and
# MEDIUM input regions and the threshold used in the SRR definition in
# Section 5.4.2.
DISTURBANCE_THRESHOLD = 0.05   # g

# Reaction-latency threshold on alpha.  Used only for the diagnostic
# "reaction_latency" column printed below; not a thesis metric.
ALPHA_REACTION_THRESHOLD = 0.75
# ============================================================


# ---------- Paths (resolved relative to this script's location) ----------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
BAG_ROOT = os.path.join(REPO_ROOT, 'bags')
OUT_DIR = os.path.join(SCRIPT_DIR, 'plots')
os.makedirs(OUT_DIR, exist_ok=True)


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


def mu_vibration_low(v): return trapezoidal(v, 0.000, 0.000, 0.030, 0.045)
def mu_vibration_medium(v): return triangular(v, 0.035, 0.055, 0.085)
def mu_vibration_high(v):
    if v >= 0.2:
        return 1.0
    return trapezoidal(v, 0.070, 0.090, 0.20, 0.20)
def mu_speed_slow(s): return trapezoidal(s, 0.40, 0.40, 0.50, 0.65)
def mu_speed_medium(s): return triangular(s, 0.55, 0.72, 0.85)
def mu_speed_fast(s): return trapezoidal(s, 0.75, 0.90, 1.00, 1.00)


def fuzzy_controller(v):
    """Mamdani inference (Section 3.7), identical to the live
    fuzzy_alpha_node.py.  Deterministic and stateless, so re-applying
    it to the recorded /vibration_feature reproduces the live alpha
    exactly (Section 5.1.4)."""
    mu_low = mu_vibration_low(v)
    mu_med = mu_vibration_medium(v)
    mu_high = mu_vibration_high(v)
    s_values = np.linspace(0.4, 1.0, 1000)
    slow_c = np.array([min(mu_high, mu_speed_slow(s)) for s in s_values])
    med_c = np.array([min(mu_med, mu_speed_medium(s)) for s in s_values])
    fast_c = np.array([min(mu_low, mu_speed_fast(s)) for s in s_values])
    aggregated = np.maximum.reduce([slow_c, med_c, fast_c])
    if np.sum(aggregated) == 0:
        return 0.4
    return float(np.sum(s_values * aggregated) / np.sum(aggregated))


# ---------- Bag reader ----------

def read_bag(bag_path):
    """Read the three topics needed for the metrics."""
    storage = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter = ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage, converter)

    type_map = {
        '/vibration_feature': Float32,
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


def interp_to(t_grid, t_src, v_src):
    """Linear interpolation onto a common time grid, with edge-value
    extrapolation outside the source support."""
    if len(t_src) == 0:
        return np.full_like(t_grid, np.nan, dtype=float)
    return np.interp(t_grid, t_src, v_src,
                     left=v_src[0], right=v_src[-1])


# ---------- Per-run metric computation ----------

def compute_metrics(bag_name, fuzzy_on):
    """Compute SRR, mean alpha (quiet and disturbed), and the
    reaction-latency diagnostic for a single bag."""
    bag_path = os.path.join(BAG_ROOT, bag_name)
    data = read_bag(bag_path)

    # Common time origin = earliest timestamp across all topics.
    all_t = []
    for topic in data:
        all_t.extend(data[topic]['t'])
    if not all_t:
        return None
    t0_ns = min(all_t)

    def t_sec(topic):
        return (np.array(data[topic]['t'], dtype=np.int64) - t0_ns) * 1e-9

    t_vib = t_sec('/vibration_feature')
    v_vib = np.array([m.data for m in data['/vibration_feature']['msg']])
    # Reconstruct alpha offline (Section 5.1.4).
    alpha = np.array([fuzzy_controller(v) for v in v_vib])

    t_user = t_sec('/cmd_vel_user')
    cmd_user = np.array([m.linear.x for m in data['/cmd_vel_user']['msg']])

    t_final = t_sec('/S2R5/cmd_vel')
    cmd_final = np.array([m.linear.x for m in data['/S2R5/cmd_vel']['msg']])

    if len(t_vib) == 0 or len(t_user) == 0 or len(t_final) == 0:
        return None

    # Build a common 50 Hz grid over the overlap of the three topics.
    t_start = max(t_vib[0], t_user[0], t_final[0])
    t_end = min(t_vib[-1], t_user[-1], t_final[-1])
    if t_end <= t_start:
        return None
    t_grid = np.arange(t_start, t_end, 0.02)

    v_g = interp_to(t_grid, t_vib, v_vib)
    a_g = interp_to(t_grid, t_vib, alpha)
    u_g = interp_to(t_grid, t_user, cmd_user)
    f_g = interp_to(t_grid, t_final, cmd_final)

    # Identify the disturbance interval (v > threshold).  Section 5.4.2.
    disturbed_mask = v_g > DISTURBANCE_THRESHOLD
    quiescent_mask = ~disturbed_mask

    # Restrict to samples where the operator is commanding motion
    # (u_user > 0.05 m/s).  This excludes start-up and stop intervals
    # from the SRR average; see the SRR definition in Section 5.4.2.
    moving_mask = u_g > 0.05
    disturbed_mask &= moving_mask
    quiescent_mask &= moving_mask

    if disturbed_mask.sum() < 5:
        # Disturbance was too brief or never crossed the threshold.
        srr = np.nan
        mean_alpha_dist = np.nan
    else:
        # SRR: equation (5.X) of Section 5.4.2.
        srr = np.mean(f_g[disturbed_mask]) / np.mean(u_g[disturbed_mask])
        mean_alpha_dist = np.mean(a_g[disturbed_mask])

    if quiescent_mask.sum() < 5:
        mean_alpha_quiet = np.nan
    else:
        mean_alpha_quiet = np.mean(a_g[quiescent_mask])

    # Diagnostic only -- not reported in the thesis.
    # Time between the first v-threshold crossing and the moment alpha
    # first drops below ALPHA_REACTION_THRESHOLD.
    latency = np.nan
    if fuzzy_on:
        if disturbed_mask.any():
            cross_v_idx = np.argmax(v_g > DISTURBANCE_THRESHOLD)
            # Require at least 5 consecutive samples below the threshold
            # (100 ms at 50 Hz) to avoid one-sample false triggers.
            below = a_g[cross_v_idx:] < ALPHA_REACTION_THRESHOLD
            if below.sum() >= 5:
                after_idx = np.argmax(below) + cross_v_idx
                latency = t_grid[after_idx] - t_grid[cross_v_idx]

    return {
        'bag': bag_name,
        'srr': srr,
        'mean_alpha_dist': mean_alpha_dist,
        'mean_alpha_quiet': mean_alpha_quiet,
        'latency_s': latency,
        'peak_v': float(np.nanmax(v_g)),
        'disturbed_samples': int(disturbed_mask.sum()),
    }


# ---------- Run analysis on all six bags ----------
print('Reading ON runs...')
on_results = [compute_metrics(b, fuzzy_on=True) for b in ON_BAGS]
on_results = [r for r in on_results if r is not None]

print('Reading OFF runs...')
off_results = [compute_metrics(b, fuzzy_on=False) for b in OFF_BAGS]
off_results = [r for r in off_results if r is not None]


# ---------- Print Table 5.2 ----------

def fmt(x, w=8, prec=3):
    if x is None or (isinstance(x, float) and np.isnan(x)):
        return f'{"—":>{w}}'
    return f'{x:>{w}.{prec}f}'


print('\n' + '=' * 88)
print(f'{"Run":40s} | {"SRR":>8s} | {"α dist":>8s} | {"α quiet":>8s} | {"peak v":>8s}')
print('-' * 88)

for r in on_results:
    print(f'{r["bag"]:40s} | {fmt(r["srr"])} | {fmt(r["mean_alpha_dist"])} | '
          f'{fmt(r["mean_alpha_quiet"])} | {fmt(r["peak_v"])}')

if on_results:
    srrs = [r['srr'] for r in on_results if not np.isnan(r['srr'])]
    alphas_d = [r['mean_alpha_dist'] for r in on_results if not np.isnan(r['mean_alpha_dist'])]
    alphas_q = [r['mean_alpha_quiet'] for r in on_results if not np.isnan(r['mean_alpha_quiet'])]

    print('-' * 88)
    print(f'{"Fuzzy ON  mean ± std":40s} | '
          f'{np.mean(srrs):.3f} ± {np.std(srrs):.3f} | '
          f'{np.mean(alphas_d):.3f} ± {np.std(alphas_d):.3f} | '
          f'{np.mean(alphas_q):.3f} ± {np.std(alphas_q):.3f} | '
          f'        ')

print('-' * 88)
for r in off_results:
    print(f'{r["bag"]:40s} | {fmt(r["srr"])} | {fmt(r["mean_alpha_dist"])} | '
          f'{fmt(r["mean_alpha_quiet"])} | {fmt(r["peak_v"])}')

if off_results:
    srrs = [r['srr'] for r in off_results if not np.isnan(r['srr'])]
    print('-' * 88)
    print(f'{"Fuzzy OFF mean ± std":40s} | '
          f'{np.mean(srrs):.3f} ± {np.std(srrs):.3f} | '
          f'{"—":>8s} | {"—":>8s} | {"—":>8s} | {"—":>8s}')

print('=' * 88)


# ---------- Plot Figure 5.6: two-panel aggregate metrics ----------
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))

# --- Panel 1: SRR comparison ---
on_srrs = [r['srr'] for r in on_results if not np.isnan(r['srr'])]
off_srrs = [r['srr'] for r in off_results if not np.isnan(r['srr'])]

x = np.arange(2)
means = [np.mean(on_srrs), np.mean(off_srrs)]
stds = [np.std(on_srrs), np.std(off_srrs)]
labels = ['Fuzzy ON', 'Fuzzy OFF']
colors = ['#1f77b4', '#d62728']

bars = axes[0].bar(x, means, yerr=stds, capsize=8, color=colors,
                   edgecolor='black', linewidth=0.8, alpha=0.85)
axes[0].set_xticks(x)
axes[0].set_xticklabels(labels)
axes[0].set_ylabel('Speed Reduction Ratio (SRR)')
axes[0].set_title('SRR during disturbance interval')
axes[0].axhline(1.0, color='gray', linestyle=':', linewidth=0.8, alpha=0.6)
axes[0].set_ylim(0, 1.15)
axes[0].grid(axis='y', alpha=0.3)

# Overlay individual run values as black markers.
for i, r in enumerate(on_results):
    if not np.isnan(r['srr']):
        axes[0].plot(0, r['srr'], 'o', color='black', markersize=5, zorder=3)
for i, r in enumerate(off_results):
    if not np.isnan(r['srr']):
        axes[0].plot(1, r['srr'], 'o', color='black', markersize=5, zorder=3)

# --- Panel 2: mean alpha, quiet vs disturbed (fuzzy-ON only) ---
alphas_q = [r['mean_alpha_quiet'] for r in on_results if not np.isnan(r['mean_alpha_quiet'])]
alphas_d = [r['mean_alpha_dist'] for r in on_results if not np.isnan(r['mean_alpha_dist'])]

x2 = np.arange(2)
means2 = [np.mean(alphas_q), np.mean(alphas_d)]
stds2 = [np.std(alphas_q), np.std(alphas_d)]
labels2 = ['Quiescent\n(v ≤ 0.05 g)', 'Disturbed\n(v > 0.05 g)']

axes[1].bar(x2, means2, yerr=stds2, capsize=8,
            color=['#2ca02c', '#ff7f0e'], edgecolor='black',
            linewidth=0.8, alpha=0.85)
axes[1].set_xticks(x2)
axes[1].set_xticklabels(labels2)
axes[1].set_ylabel(r'Mean $\alpha$')
axes[1].set_title(r'Mean $\alpha$ across fuzzy-ON runs')
axes[1].set_ylim(0.4, 1.05)
axes[1].grid(axis='y', alpha=0.3)

for r in on_results:
    if not np.isnan(r['mean_alpha_quiet']):
        axes[1].plot(0, r['mean_alpha_quiet'], 'o',
                     color='black', markersize=5, zorder=3)
    if not np.isnan(r['mean_alpha_dist']):
        axes[1].plot(1, r['mean_alpha_dist'], 'o',
                     color='black', markersize=5, zorder=3)

plt.tight_layout()
out_png = os.path.join(OUT_DIR, 'plot_G_aggregate_metrics.png')
out_pdf = os.path.join(OUT_DIR, 'plot_G_aggregate_metrics.pdf')
plt.savefig(out_png, dpi=200, bbox_inches='tight')
plt.savefig(out_pdf, bbox_inches='tight')
print(f'\nSaved: {out_png}')
print(f'Saved: {out_pdf}')
plt.show()
