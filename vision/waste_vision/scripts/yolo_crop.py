from ultralytics import YOLO
import cv2
from pathlib import Path
import argparse

def crop_with_loaded_model(image_path, model, output_root, conf=0.3):
    image_path = Path(image_path)
    output_dir = output_root / image_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)

    results = model.predict(source=str(image_path), conf=conf, save=False, verbose=False)
    boxes = results[0].boxes

    img = cv2.imread(str(image_path))
    h, w = img.shape[:2]

    label_lines = []

    if boxes is None or len(boxes) == 0:
        # Save full image and single dummy row ** CAN CHANGE IF NEEDED **
        cv2.imwrite(str(output_dir / "bbox_0.jpg"), img)
        with open(output_dir / "labels.txt", "w") as f:
            f.write("999 0 0 0 0\n")
        print(f"[!] {image_path.name}: No detections, dummy saved.")
        return

    for i, box in enumerate(boxes):
        xyxy = box.xyxy[0].cpu().numpy().astype(int)
        cls = int(box.cls[0].item())

        x1, y1, x2, y2 = xyxy
        cropped = img[y1:y2, x1:x2]

        # YOLO normalized format
        xc = ((x1 + x2) / 2) / w
        yc = ((y1 + y2) / 2) / h
        bw = (x2 - x1) / w
        bh = (y2 - y1) / h

        # Save crop
        crop_file = output_dir / f"bbox_{i}.jpg"
        cv2.imwrite(str(crop_file), cropped)

        # Save label to list
        label_lines.append(f"{cls} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")

    # Save all labels to one file
    with open(output_dir / "labels.txt", "w") as f:
        f.write("\n".join(label_lines) + "\n")

    print(f"{image_path.name}: Saved {len(boxes)} crops and labels to {output_dir}")


def my_crop_with_loaded_model(image_path, model, conf=0.1):
    image_path = Path(image_path)
    results = model.predict(source=str(image_path), conf=conf, save=False, verbose=False)
    boxes = results[0].boxes

    img = cv2.imread(str(image_path))
    h, w = img.shape[:2]

    crops = []
    labels = []

    if boxes is None or len(boxes) == 0:
        # Optional: return dummy data if no detections
        crops.append(img)
        labels.append(["999", 0, 0, 0, 0])
        print(f"[!] {image_path.name}: No detections, dummy crop returned.")
        return crops, labels

    for i, box in enumerate(boxes):
        xyxy = box.xyxy[0].cpu().numpy().astype(int)
        cls = int(box.cls[0].item())

        x1, y1, x2, y2 = xyxy
        cropped = img[y1:y2, x1:x2]

        # Normalized YOLO format
        xc = ((x1 + x2) / 2) / w
        yc = ((y1 + y2) / 2) / h
        bw = (x2 - x1) / w
        bh = (y2 - y1) / h

        crops.append(cropped)
        labels.append([xc, yc, bw, bh])

    # print(f"{image_path.name}: Returned {len(crops)} crops and labels from memory.")
    return crops, labels


def test_my_function(image_path, model):
    crops, labels = my_crop_with_loaded_model(image_path, model)

    for i, crop in enumerate(crops):
        cv2.imshow(f"Crop {i}", crop)
        print(f"Label {i}: {labels[i]}")
        cv2.waitKey(0)

    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=False, help="Path to the image file")
    args = parser.parse_args()

    # model_path = Path(__file__).parent / "pm_28model.pt"
    model_path = "/home/nddixon/catkin_ws/src/waste_vision/models/yolo/pm_28model.pt"
    model = YOLO(str(model_path))
    output_root = Path(".")
    img_path = "/home/nddixon/catkin_ws/src/waste_vision/src/part1_complete/test_3.jpg"

    crop_with_loaded_model(img_path, model=model, output_root=output_root, conf=0.1)

    test_my_function(image_path=img_path, model=model)



if __name__ == "__main__":
    main()