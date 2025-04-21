import numpy as np
import random
from pathlib import Path
import matplotlib.pyplot as plt

IMG_WIDTH = 1920
IMG_HEIGHT = 1080
WORLD_WIDTH_M = 0.9
PIXEL_TO_MM = WORLD_WIDTH_M / IMG_WIDTH

bins_px = {
    0: np.array([0, 0]),
    1: np.array([IMG_WIDTH - 1, 0]),
    2: np.array([0, IMG_HEIGHT - 1]),
    3: np.array([IMG_WIDTH - 1, IMG_HEIGHT - 1]),
    4: np.array([IMG_WIDTH - 1, IMG_HEIGHT // 2])
}

start_position_px = np.array([IMG_WIDTH // 2, IMG_HEIGHT // 2])

def dist(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def total_path_distance(order, bins, start_pos):
    pos = start_pos.copy()
    total_dist = 0
    for obj in order:
        total_dist += dist(pos, (obj[0], obj[1]))
        total_dist += dist((obj[0], obj[1]), bins[obj[2]])
        pos = bins[obj[2]]
    return total_dist

def path_from_order(order, bins, start_pos):
    pos = start_pos.copy()
    path = []
    for obj in order:
        path.append((obj[0], obj[1], obj[2]))
        path.append((bins[obj[2]][0], bins[obj[2]][1], None))
        pos = bins[obj[2]]
    return path

def simulated_annealing(objects, bins, start_pos, T=1.0, T_min=1e-4, alpha=0.995, max_iter=10000):
    if len(objects) <= 1:
        # Nothing to optimize, just return the path for the single object (or empty)
        return path_from_order(objects, bins, start_pos)
    
    current = objects[:]
    best = current[:]
    best_cost = total_path_distance(best, bins, start_pos)
    current_cost = best_cost
    iteration = 0

    while T > T_min and iteration < max_iter:
        i, j = random.sample(range(len(objects)), 2)
        new = current[:]
        new[i], new[j] = new[j], new[i]
        new_cost = total_path_distance(new, bins, start_pos)
        if new_cost < current_cost or random.random() < np.exp((current_cost - new_cost) / T):
            current = new
            current_cost = new_cost
            if new_cost < best_cost:
                best = new
                best_cost = new_cost
        T *= alpha
        iteration += 1

    return path_from_order(best, bins, start_pos)

def load_yolo_txt(txt_path):
    with open(txt_path, "r") as f:
        lines = f.read().strip().splitlines()
    objects = []
    for line in lines:
        cls, xc, yc, w, h = map(float, line.strip().split())
        cls = int(cls)
        if cls == 999 or cls not in bins_px:
            continue
        px = int(xc * IMG_WIDTH)
        py = int(yc * IMG_HEIGHT)
        objects.append((px, py, cls))
    return objects

def parse_yolo_labels(label_list):
    objects = []
    for entry in label_list:
        cls, xc, yc, w, h = map(float, entry)
        if int(cls) == 999:
            continue
        abs_x = int(xc * IMG_WIDTH)
        abs_y = int(yc * IMG_HEIGHT)
        objects.append((abs_x, abs_y, int(cls)))
    print(objects)
    return objects

def save_object_path_with_class(path, out_file):
    with open(out_file, "w") as f:
        for x_px, y_px, cls in path:
            if cls is not None:
                x_mm = x_px * PIXEL_TO_MM
                y_mm = y_px * PIXEL_TO_MM
                f.write(f"{cls} {x_mm:.2f} {y_mm:.2f}\n")

def return_path_with_class(path):
    cls_positions = []
    for x_px, y_px, cls in path:
        if cls is not None:
            x_mm = x_px * PIXEL_TO_MM
            y_mm = y_px * PIXEL_TO_MM
            cls_positions.append((cls, x_mm, y_mm))
            # f.write(f"{cls} {x_mm:.2f} {y_mm:.2f}\n")
    return cls_positions

def visualize_path(path):
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.set_title("Optimized Pick and Place Path (World Coordinates, mm)")
    xs, ys = zip(*[(x, y) for x, y, _ in path])
    xs_mm = [x * PIXEL_TO_MM for x in xs]
    ys_mm = [y * PIXEL_TO_MM for y in ys]
    ax.plot(xs_mm, ys_mm, '-o', color='green', label="Pick & Place Path")
    for i, (x, y) in enumerate(zip(xs_mm, ys_mm)):
        ax.text(x + 10, y, str(i), fontsize=8, color='blue')
    for cls, (x_px, y_px) in bins_px.items():
        x_mm = x_px * PIXEL_TO_MM
        y_mm = y_px * PIXEL_TO_MM
        ax.plot(x_mm, y_mm, 'rs')
        ax.text(x_mm + 10, y_mm, f"Bin {cls}", fontsize=9, color='red')
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="YOLO-format label file")
    parser.add_argument("--output", default="object_pick_path_mm.txt", help="Output pick path with class IDs")
    parser.add_argument("--plot", action="store_true", help="Visualize the path")
    args = parser.parse_args()
    objects = load_yolo_txt(args.input)

    path = simulated_annealing(objects, bins_px, start_position_px)
    save_object_path_with_class(path, args.output)
    print(f"Saved object pick path with classes to {args.output}")
    if args.plot:
        visualize_path(path)
