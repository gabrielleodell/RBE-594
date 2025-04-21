# YOLO Crop Pipeline

This script runs a YOLOv11 model on an image, saves cropped detections, and logs all bounding boxes in a single labels.txt file.

## Requirements

```bash
pip install ultralytics opencv-python
```

## Folder Structure

```
PART1_COMPLETE/
├── pm_28model.pt       # YOLOv11 model
├── example_pic.jpg     # Input image
├── yolo_crop.py        # Script
├── README.md           # This lol
```

## How to Run

```bash
python yolo_crop.py --image example_pic.jpg
```

## Output

Creates a folder `example_pic/` with:

- `bbox_0.jpg`, `bbox_1.jpg`, ... → Cropped detections
- `labels.txt` → One row per box:
  ```
  <class_id> <x_center> <y_center> <width> <height>
  ```

If no detections:

```
999 0 0 0 0
```

This will also save a bounding box image that is actually just the whole image

## Use from Another Python Script

```python
from yolo_crop import crop_with_loaded_model
from ultralytics import YOLO
from pathlib import Path

model = YOLO("p1_28model.pt")
crop_with_loaded_model("example_pic.jpg", model, Path("."), conf=0.3)
```
