# Convert YOLO Labels to World Coordinate Centers

This script converts a YOLO-format `.txt` file with bounding boxes into a list of object classes and their **center positions in world coordinates (millimeters)**.

---

## Requirements

```bash
pip install numpy
```

---

## Input Format

The input `.txt` file should use standard YOLO format:

```
<class_id> <x_center> <y_center> <width> <height>
```

- All values should be normalized between 0 and 1.
- The image is assumed to be 1920 × 1080 pixels.
- The world width is 900 mm, and the height is scaled proportionally.

---

## How to Use

```bash
python convert_labels_to_world_centers.py --input labels.txt --output class_centers_mm.txt
```

---

## Output Format

The output file will contain:

```
<class_id> <x_center_mm> <y_center_mm>
```

Example:

```
3 342.97 666.41
1 486.72 558.59
```
