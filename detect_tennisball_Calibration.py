# Author: Sarah Downs
# Second
# This code uses the csv file output from detect_tennisball_csvCal.py to calibrate the system for the
# transformation matrix. This is to identify the matrix between the camera and robot frames. 

import numpy as np
import csv
import os
from glob import glob

print("Running calibration...")

def load_latest_csv(folder='.'):
    csv_files = glob(os.path.join(folder, "zed_ur_tennisball_*.csv"))
    if not csv_files:
        raise FileNotFoundError("No matching CSV files found.")
    latest_file = max(csv_files, key=os.path.getctime)
    print(f"[INFO] Using file: {latest_file}")
    return latest_file

def read_points_from_csv(filename):
    ball_points = []
    tcp_points = []
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            ball = [float(row["Ball_X"]), float(row["Ball_Y"]), float(row["Ball_Z"])]
            tcp = [float(row["TCP_X"]), float(row["TCP_Y"]), float(row["TCP_Z"])]
            ball_points.append(ball)
            tcp_points.append(tcp)
    return np.array(ball_points), np.array(tcp_points)

def compute_transformation(camera_points, robot_points):
    assert camera_points.shape == robot_points.shape
    N = camera_points.shape[0]

    centroid_cam = np.mean(camera_points, axis=0)
    centroid_tcp = np.mean(robot_points, axis=0)

    cam_centered = camera_points - centroid_cam
    tcp_centered = robot_points - centroid_tcp

    H = cam_centered.T @ tcp_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # Reflection case
    if np.linalg.det(R) < 0:
        print("[WARNING] Reflection detected. Fixing.")
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_tcp - R @ centroid_cam

    # Assemble homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

if __name__ == "__main__":
    latest_csv = load_latest_csv(".")  # Change directory if needed
    camera_pts, robot_pts = read_points_from_csv(latest_csv)

    if camera_pts.shape[0] < 3:
        print("[ERROR] Need at least 3 points for transformation estimation.")
        exit(1)

    T = compute_transformation(camera_pts, robot_pts)
    print("\n=== Homogeneous Transformation Matrix (Camera → Robot) ===")
    print(T)

