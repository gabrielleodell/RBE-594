# Simulated Annealing Pick Path Generator (World Coordinates with Class Labels)

This script takes in a YOLO-format `.txt` file with object detections (normalized format) and uses **simulated annealing** to generate an optimized pick-and-place path. It outputs **only the pick steps** in **millimeters**, with associated class IDs, for use in motion planning. The coordinate system is aligned with the camera — (0, 0) is the top-left of the image.

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

- Coordinates are **normalized** (0–1).
- Assumes image resolution of **1920×1080 pixels**.
- Image width corresponds to **900 mm**, and height scales equally.
- Class IDs **0–4** are supported and mapped to drop-off bins during planning.

---

## How to Run

```bash
python generate_pick_path_with_classes.py --input labels.txt --output object_pick_path_mm.txt --plot
```

- `--input` → YOLO-format `.txt` label file
- `--output` → path to save the pick path with class and XY (in mm)
- `--plot` → optional: visualize the planned path

---

## Output Format

The output file contains **only the object pick locations** (no bins), with class:

```
<class_id> <x_mm> <y_mm>
```

Example:

```
2 184.38 777.19
0 14.06 13.12
1 881.25 10.31
```

---

## Bin Locations (Used in Path Planning Only)

| Class | Bin Location      |
| ----- | ----------------- |
| 0     | Top-left          |
| 1     | Top-right         |
| 2     | Bottom-left       |
| 3     | Bottom-right      |
| 4     | Center-right edge |

The robot starts from center of image: **(960, 540 px → 450 mm, 253 mm)**

---

## Use in Another Python Script

```python
from generate_pick_path_with_classes import (
    load_yolo_txt, simulated_annealing, save_object_path_with_class, bins_px, start_position_px
)

# Load YOLO objects
objects = load_yolo_txt("labels.txt")

# Get optimized path
path = simulated_annealing(objects, bins_px, start_position_px)

# Save only object picks with class
save_object_path_with_class(path, "object_pick_path_mm.txt")
```

---
