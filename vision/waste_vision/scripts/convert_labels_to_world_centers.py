import numpy as np
from pathlib import Path

# Image and scale settings
IMG_WIDTH = 1920
IMG_HEIGHT = 1080
WORLD_WIDTH_MM = 900
PIXEL_TO_MM = WORLD_WIDTH_MM / IMG_WIDTH  # consistent scaling

def load_labels_to_world_centers(yolo_path):
    objects = []
    with open(yolo_path, "r") as f:
        for line in f:
            cls, xc, yc, w, h = map(float, line.strip().split())
            cls = int(cls)
            if cls == 999:
                continue
            x_mm = xc * IMG_WIDTH * PIXEL_TO_MM
            y_mm = yc * IMG_HEIGHT * PIXEL_TO_MM
            objects.append((cls, x_mm, y_mm))
    return objects

def labels_to_world(label_list):
    objects = []
    for entry in label_list:
        cls, xc, yc, w, h = map(float, entry)
        cls = int(cls)
        if cls == 999:
            continue
        x_mm = xc * IMG_WIDTH * PIXEL_TO_MM
        y_mm = yc * IMG_HEIGHT * PIXEL_TO_MM
        objects.append((x_mm, y_mm, cls))
    print(objects)
    return objects

def save_centers(objects, output_path):
    with open(output_path, "w") as f:
        for cls, x, y in objects:
            f.write(f"{cls} {x:.2f} {y:.2f}\n")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="YOLO-format labels.txt file")
    parser.add_argument("--output", default="class_centers_mm.txt", help="Output text file")
    args = parser.parse_args()

    objs = load_labels_to_world_centers(args.input)
    save_centers(objs, args.output)
    print(f"Saved {len(objs)} object centers to {args.output}")
