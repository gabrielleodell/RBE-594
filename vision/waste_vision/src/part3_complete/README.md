# Simulated Annealing Path Optimizer (Camera Coordinates)

This script takes in a YOLO-format `.txt` file with object detections (normalized format) and uses **simulated annealing** to generate an optimized pick-and-place path. It outputs XY coordinate pairs in **pixel (camera) coordinates** for motion planning.

---

## Requirements

```bash
pip install numpy
```

---

## Input Format

The input `.txt` file must contain YOLO-style bounding boxes:

```
<class_id> <x_center> <y_center> <width> <height>
```

- Coordinates are **normalized** (i.e., in range 0–1).
- The script assumes an image resolution of **1920×1080**.
- Only class IDs **0–3** are used (mapped to drop-off bins).
- Any class not in that range will be ignored.

---

## How to Run

```bash
python generate_path_camera_coords.py --input labels.txt --output path_xy.txt
```

- `--input` → path to the YOLO-format `.txt` file
- `--output` → output file with XY coordinates in pixels

---

## Output Format

The output `.txt` file will contain camera-space XY positions (in pixels), alternating pick and place:

```
x1 y1   # pick object 1
x2 y2   # drop object 1
x3 y3   # pick object 2
x4 y4   # drop object 2
...
```

---

## Notes

- Assumes camera resolution is **1920×1080**
- Bins are positioned on the image edges:
  - Class 0 → top-left
  - Class 1 → top-right
  - Class 2 → bottom-left
  - Class 3 → bottom-right
- The robot starts from center: `(960, 540)`

---

