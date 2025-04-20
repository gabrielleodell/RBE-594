import cv2
import numpy as np
import onnxruntime as ort
import torch
from pathlib import Path
import argparse

# Helper: Non‑max suppression from Ultralytics
from ultralytics.utils.ops import non_max_suppression

# Preprocess image for ONNX YOLO
def preprocess(img: np.ndarray, imgsz: int = 640):
    h0, w0 = img.shape[:2]
    r = imgsz / max(h0, w0)
    nh, nw = int(h0 * r), int(w0 * r)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((imgsz, imgsz, 3), 114, dtype=np.uint8)
    top, left = (imgsz - nh) // 2, (imgsz - nw) // 2
    canvas[top:top+nh, left:left+nw] = resized
    # BGR to RGB, HWC to CHW, float32, normalize
    tensor = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return tensor, r, top, left

# Select CPU or GPU for ONNXRuntime
def select_provider():
    providers = ort.get_available_providers()
    if 'CUDAExecutionProvider' in providers:
        provider = 'CUDAExecutionProvider'
    else:
        provider = 'CPUExecutionProvider'
    print(f"ONNXRuntime providers available: {providers}. Using: {provider}")
    return provider

class ONNXDetector:
    def __init__(self, model_path: str):
        provider = select_provider()
        try:
            self.session = ort.InferenceSession(
                model_path,
                providers=[provider]
            )
        except Exception as e:
            print(f"Warning: failed to init {{provider}}, falling back to CPU. {{e}}")
            self.session = ort.InferenceSession(
                model_path,
                providers=['CPUExecutionProvider']
            )
        self.input_name = self.session.get_inputs()[0].name

    def detect(self, img: np.ndarray, conf: float = 0.3, iou: float = 0.2):
        tensor, r, top, left = preprocess(img)
        preds_np = self.session.run(None, {self.input_name: tensor})[0]
        preds = torch.from_numpy(preds_np).float()
        dets_list = non_max_suppression(preds, conf_thres=conf, iou_thres=iou)
        dets = dets_list[0] if isinstance(dets_list, (list, tuple)) else dets_list
        results = []
        if dets is not None and len(dets):
            for x1, y1, x2, y2, conf_score, cls in dets.cpu().numpy():
                # reverse letterbox
                x1 = (x1 - left) / r
                y1 = (y1 - top) / r
                x2 = (x2 - left) / r
                y2 = (y2 - top) / r
                results.append((int(cls), float(conf_score), [int(x1), int(y1), int(x2), int(y2)]))
        return results

# Crop images and save labels
def crop_with_onnx(image_path: str, detector: ONNXDetector, output_root: Path, conf: float = 0.3, iou: float = 0.2):
    image_path = Path(image_path)
    output_dir = output_root / image_path.stem
    output_dir.mkdir(parents=True, exist_ok=True)
    img = cv2.imread(str(image_path))
    h, w = img.shape[:2]

    detections = detector.detect(img, conf=conf, iou=iou)
    if not detections:
        cv2.imwrite(str(output_dir / "bbox_0.jpg"), img)
        with open(output_dir / "labels.txt", "w") as f:
            f.write("999 0 0 0 0\n")
        print(f"[!] {image_path.name}: No detections, dummy saved.")
        return

    label_lines = []
    for i, (cls, score, box) in enumerate(detections):
        x1, y1, x2, y2 = box
        crop = img[y1:y2, x1:x2]
        cv2.imwrite(str(output_dir / f"bbox_{i}.jpg"), crop)
        xc = ((x1 + x2) / 2) / w
        yc = ((y1 + y2) / 2) / h
        bw = (x2 - x1) / w
        bh = (y2 - y1) / h
        label_lines.append(f"{cls} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")

    with open(output_dir / "labels.txt", "w") as f:
        f.write("\n".join(label_lines) + "\n")
    print(f"{image_path.name}: Saved {len(detections)} crops and labels to {output_dir}")

def my_crop_with_onnx(image_path: str, detector: ONNXDetector, conf=0.3, iou=0.2):
    image_path = Path(image_path)
    # output_dir = output_root / image_path.stem
    # output_dir.mkdir(parents=True, exist_ok=True)
    img = cv2.imread(str(image_path))
    h, w = img.shape[:2]

    detections = detector.detect(img, conf=conf, iou=iou)
    if not detections:
        print(f"[!] {image_path.name}: No detections.")
        return

    label_lines = []
    crops = []
    for i, (cls, score, box) in enumerate(detections):
        x1, y1, x2, y2 = box
        crop = img[y1:y2, x1:x2]
        crops.append(crop)
        # cv2.imwrite(str(output_dir / f"bbox_{i}.jpg"), crop)
        xc = ((x1 + x2) / 2) / w
        yc = ((y1 + y2) / 2) / h
        bw = (x2 - x1) / w
        bh = (y2 - y1) / h
        label_lines.append([xc, yc, bw, bh])

    # with open(output_dir / "labels.txt", "w") as f:
    #     f.write("\n".join(label_lines) + "\n")
    # print(f"{image_path.name}: Saved {len(detections)} crops and labels to {output_dir}")
    return crops, label_lines

def run():
    onnx_path = "/home/nddixon/catkin_ws/src/waste_vision/models/onnx/pm_28model.onnx"
    detector = ONNXDetector(onnx_path)
    img_path = "/home/nddixon/catkin_ws/src/waste_vision/test_images/multi_test.jpg"
    crops, labels = my_crop_with_onnx(detector=detector, image_path=img_path)
    print(f'Labels: {labels}')
    print(len(crops))
    for i, crop in enumerate(crops):
        cv2.imshow(f'img_{i}', crop)
        
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# Entrypoint
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True, help="Path to the image file")
    parser.add_argument("--onnx", required=True, help="Path to the ONNX model file (.onnx)")
    parser.add_argument("--conf", type=float, default=0.3, help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.2, help="NMS IoU threshold")
    args = parser.parse_args()

    detector = ONNXDetector(str(args.onnx))
    output_root = Path(".")
    crop_with_onnx(args.image, detector, output_root, conf=args.conf, iou=args.iou)

if __name__ == "__main__":
    run()

