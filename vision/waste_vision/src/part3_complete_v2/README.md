# Simulated Annealing Path Optimizer (World Coordinates)

This script takes in a YOLO-format `.txt` file with object detections (normalized format) and uses **simulated annealing** to generate an optimized pick-and-place path. It outputs XY coordinate pairs in **millimeters** for motion planning. The coordinate system is aligned with the camera — (0, 0) is the top-left of the image.

---

## Requirements

```bash
pip install numpy matplotlib
```

---

## Input Format

The input `.txt` file must contain YOLO-style bounding boxes:

```
<class_id> <x_center> <y_center> <width> <height>
```

- Coordinates are **normalized** (i.e., in range 0–1).
- Image resolution is assumed to be **1920×1080**.
- The image width maps to **900 mm**; height scales accordingly.
- Class IDs **0–4** are supported and mapped to drop-off bins.

---

## How to Run

```bash
python generate_path_world_coords_plot_final.py --input labels.txt --output path_world_mm.txt --plot
```

- `--input` → path to the YOLO-format `.txt` file
- `--output` → file with XY coordinates in **millimeters**
- `--plot` → optional flag to **visualize** the path and bins

---

## Output Format

The output `.txt` file contains **world coordinates in mm**, alternating pick and place steps:

```
x1 y1   # pick object 1
x2 y2   # drop object 1
x3 y3   # pick object 2
x4 y4   # drop object 2
...
```

---

## Bin Locations

The drop-off bins are placed at the camera image edges:

| Class | Bin Location      |
|-------|-------------------|
| 0     | Top-left          |
| 1     | Top-right         |
| 2     | Bottom-left       |
| 3     | Bottom-right      |
| 4     | Center-right edge |

The robot starts from the image center: **(960, 540 px → 450 mm, 253 mm)**

---

## To use from Another Python Script

```python
from generate_path_world_coords_plot_final import (
    load_yolo_txt, simulated_annealing, save_path_world_mm
)

# Load and process
objects = load_yolo_txt("labels.txt")
path = simulated_annealing(objects, bins_px, start_position_px)
save_path_world_mm(path, "path_world_mm.txt")
```

This avoids plotting and just generates the path for downstream usage.

---
