import re
import numpy as np
import matplotlib.pyplot as plt

file_path_list = ["LiDAR Cartesian Plot (Circle Motion).csv", "LiDAR Cartesian Plot (Line Motion).csv", "LiDAR Cartesian Plot (Spiral Motion).csv"]

def parse_scan(line):
    """Parse a single ROS2 LaserScan-style line."""
    arr_match = re.search(r"array\('f',\s*\[(.*?)\]\)", line)
    if not arr_match:
        return None, None
    ranges_str = arr_match.group(1)

    vals = []
    for v in ranges_str.split(','):
        v = v.strip()
        if v == 'inf':
            vals.append(np.inf)
        else:
            try:
                vals.append(float(v))
            except ValueError:
                pass
    ranges = np.array(vals, dtype=float)

    angle_match = re.search(r"\),\s*([0-9.]+)", line)
    if not angle_match:
        return ranges, None
    angle_increment = float(angle_match.group(1))

    return ranges, angle_increment

x_all, y_all = [], []

for file_path in file_path_list:
    with open(file_path, 'r') as f:
        for line in f:
            if "array('f'," not in line:
                continue

            ranges, angle_inc = parse_scan(line)
            if ranges is None or angle_inc is None:
                continue

            # Compute full angles first
            angles = np.arange(len(ranges)) * angle_inc

            # Filter invalids
            valid = np.isfinite(ranges) & (ranges > 0)
            ranges = ranges[valid]
            angles = angles[valid]

            # Polar → Cartesian
            x = ranges * np.cos(angles)
            y = ranges * np.sin(angles)

            x_all = x
            y_all = y
            break
    plt.figure(figsize=(7, 7))
    plt.scatter(x_all, y_all, s=2, color='blue')
    plt.title(file_path[:-4])
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.savefig(file_path[:-4]+".svg", bbox_inches='tight', pad_inches=0.1, dpi=300)
    plt.show()
        
