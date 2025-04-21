import numpy as np
import random
from pathlib import Path

# Image resolution
IMG_WIDTH = 1920
IMG_HEIGHT = 1080

## CHANGE BASED ON MOTION TEAM REQS ##
# Bin locations (left and right side of the image, top/bottom)
bins = {
    0: np.array([int(0.05 * IMG_WIDTH), int(0.1 * IMG_HEIGHT)]),   # Top-left
    1: np.array([int(0.95 * IMG_WIDTH), int(0.1 * IMG_HEIGHT)]),   # Top-right
    2: np.array([int(0.05 * IMG_WIDTH), int(0.9 * IMG_HEIGHT)]),   # Bottom-left
    3: np.array([int(0.95 * IMG_WIDTH), int(0.9 * IMG_HEIGHT)])    # Bottom-right
}

start_position = np.array([IMG_WIDTH // 2, IMG_HEIGHT // 2])

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
        path.append((int(obj[0]), int(obj[1])))               # Pick
        path.append((int(bins[obj[2]][0]), int(bins[obj[2]][1])))  # Place
        pos = bins[obj[2]]
    return path

def simulated_annealing(objects, bins, start_pos, T=1.0, T_min=1e-4, alpha=0.995, max_iter=10000):
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
        if int(cls) == 999:
            continue
        abs_x = int(xc * IMG_WIDTH)
        abs_y = int(yc * IMG_HEIGHT)
        objects.append((abs_x, abs_y, int(cls)))
    return objects

def save_path_xy(path, out_file):
    with open(out_file, "w") as f:
        for x, y in path:
            f.write(f"{x} {y}\n")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input YOLO-format .txt file")
    parser.add_argument("--output", default="path_xy.txt", help="Output file with XY path")
    args = parser.parse_args()

    objects = load_yolo_txt(args.input)
    path = simulated_annealing(objects, bins, start_position)
    save_path_xy(path, args.output)

    print(f"Saved path with {len(path)} XY points to {args.output}")
