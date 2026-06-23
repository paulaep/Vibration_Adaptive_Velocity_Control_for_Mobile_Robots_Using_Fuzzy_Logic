# Analysis scripts

This folder contains the Python scripts used to produce the figures and tables of Chapters 3 and 5 of the thesis.

## Script-to-figure mapping

| Script                          | Thesis figure / table                                                 |
|---------------------------------|----------------------------------------------------------------------|
| `plot_A_membership_functions.py`| Figure 3.7 — input and output membership functions                   |
| `plot_B_alpha_curve.py`         | Figure 3.8 — static input-output characteristic α(v)                  |
| `plot_F_window_sensitivity.py`  | Figure 3.6 and Table 3.2 — window-length sensitivity analysis        |
| `plot_G_metric.py`              | Figure 5.6 and Table 5.2 — aggregate SRR metrics                     |
| `plot_timeseries.py`            | Figures 5.2, 5.3, 5.4, 5.5 — four-signal time series per run         |

All scripts write their outputs (both `.png` and `.pdf`) to `analysis/plots/`. That directory is created automatically on first run.

## Dependencies

The scripts use:

- `numpy`, `matplotlib` — install via `pip` (`pip install -r ../requirements.txt`).
- `rclpy`, `rosbag2_py` — provided by your ROS 2 Humble installation. Source your ROS 2 environment (`source /opt/ros/humble/setup.bash`) before running any script that reads a bag.

## Which scripts need bag files

- `plot_A_membership_functions.py` and `plot_B_alpha_curve.py` are self-contained and run without any data.
- `plot_F_window_sensitivity.py`, `plot_G_metric.py`, and `plot_timeseries.py` read bag files from `../bags/`. Edit the `BAG_NAME` near the top of each script to select the run you want to analyse.

## Running the scripts

From any working directory:

```bash
python3 plot_A_membership_functions.py
python3 plot_B_alpha_curve.py
python3 plot_F_window_sensitivity.py
python3 plot_G_metric.py
python3 plot_timeseries.py
```

The plots open in a matplotlib window on screen and are also saved to `analysis/plots/`.

## `plot_timeseries.py` — switching between figures

This single script produces four different figures depending on three configuration lines near the top:

- `BAG_NAME` — which run to read.
- `MODE` — `'linear'` for Figures 5.2, 5.3, 5.5; `'angular'` for Figure 5.4.
- `PLOT_LABEL` — used as the output filename suffix.

The script docstring contains explicit examples for each thesis figure.
