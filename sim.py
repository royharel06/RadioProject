# Re-importing needed libraries and re-writing the updated simulation code

import numpy as np
import matplotlib.pyplot as plt
import imageio
from scipy.optimize import least_squares
from pathlib import Path

# --- Simulation Parameters ---
SIGNAL_CUTOFF = 800.0
RANGE_STD_DEV = 46.0
HOME_POS = np.array([0.0, 0.0])
DOCK_RADIUS = 10.0
ANCHOR_AREA_RADIUS = 300.0
STEP_SIZE = 20.0
MAX_STEPS = 200
CONFIDENCE_DISTANCE_CHANGE_THRESHOLD = 10.0  # meters

frames = []

def add_range_noise(true_distance):
    return true_distance + np.random.normal(0, RANGE_STD_DEV)

def simulate_ranging(drone_pos, anchors):
    ranges, positions = [], []
    for anchor in anchors:
        dist = np.linalg.norm(drone_pos - anchor)
        if dist <= SIGNAL_CUTOFF:
            ranges.append(add_range_noise(dist))
            positions.append(anchor)
    return np.array(positions), np.array(ranges)

def estimate_position(anchors, distances):
    def residuals(p): return np.linalg.norm(anchors - p, axis=1) - distances
    return least_squares(residuals, x0=np.array([0.0, 0.0])).x

def plot_simulation_path(path, anchors, step_num):
    fig, ax = plt.subplots()
    ax.plot(*zip(*path), marker='o', label='Drone Path')
    ax.scatter(*zip(*anchors), color='red', label='Anchors')
    ax.scatter(*HOME_POS, color='green', label='Home')
    circle = plt.Circle(HOME_POS, DOCK_RADIUS, color='green', fill=False, linestyle='--')
    ax.add_patch(circle)
    ax.set_xlim(-400, 600)
    ax.set_ylim(-400, 600)
    ax.set_title(f'Step {step_num}')
    ax.legend()
    fig.canvas.draw()
    image = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    image = image.reshape(fig.canvas.get_width_height()[::-1] + (4,))
    frames.append(image)
    plt.close()

def generate_random_anchors(n):
    angles = np.random.uniform(0, 2 * np.pi, n)
    radii = np.random.uniform(0, ANCHOR_AREA_RADIUS, n)
    x = HOME_POS[0] + radii * np.cos(angles)
    y = HOME_POS[1] + radii * np.sin(angles)
    return np.column_stack((x, y))

# --- User Configuration ---
#num_anchors = 4
#behavior_mode = "smart"  # options: "frequent" or "smart"

# Get user input for number of anchors and mode
num_anchors = int(input("How many anchors would you like to simulate? "))
while num_anchors < 3:
    print("You need at least 3 anchors for triangulation.")
    num_anchors = int(input("How many anchors would you like to simulate? "))
    
behavior_mode = input("Would you like to use 'frequent' or 'smart' behavior mode? (default is 'smart'): ") or "smart"
while behavior_mode != "frequent" and behavior_mode != "smart":
    print("Invalid input. Please enter 'frequent' or 'smart'.")
    num_anchors = input("Would you like to use 'frequent' or 'smart' behavior mode? (default is 'smart'): ") or "smart"

ANCHORS = generate_random_anchors(num_anchors)

# Drone starts randomly between 200–500 meters from home
r = np.random.uniform(200, 500)
theta = np.random.uniform(0, 2 * np.pi)
INITIAL_POS = HOME_POS + r * np.array([np.cos(theta), np.sin(theta)])

# --- Simulation Loop ---
drone_pos = INITIAL_POS.copy()
path = [drone_pos.copy()]
last_known_direction = None
last_position_estimate = None
ranging_requests = 0

for step in range(MAX_STEPS):
    do_range = True

    if behavior_mode == "smart" and last_position_estimate is not None:
        current_dist = np.linalg.norm(drone_pos - HOME_POS)
        predicted_next_pos = drone_pos + last_known_direction * STEP_SIZE
        predicted_est_to_home = np.linalg.norm(predicted_next_pos - HOME_POS)
        confidence_drop = current_dist - predicted_est_to_home

        if confidence_drop >= CONFIDENCE_DISTANCE_CHANGE_THRESHOLD:
            do_range = False

        if current_dist <= STEP_SIZE:
            do_range = True

    if do_range:
        anchors_visible, ranges = simulate_ranging(drone_pos, ANCHORS)
        if len(ranges) >= 3:
            ranging_requests += 1
            estimated_pos = estimate_position(anchors_visible, ranges)
            last_position_estimate = estimated_pos
            direction = HOME_POS - estimated_pos
            direction /= np.linalg.norm(direction)
            last_known_direction = direction
        else:
            print("Not enough anchors in range. Skipping step.")
            continue

    if last_known_direction is not None:
        drone_pos += last_known_direction * STEP_SIZE
        path.append(drone_pos.copy())
        plot_simulation_path(path, ANCHORS, step)

    if np.linalg.norm(drone_pos - HOME_POS) <= DOCK_RADIUS:
        print(f"Drone landed successfully at step {step}.")
        break

if not frames:
    print("No frames generated! Likely not enough anchors were ever in range.")
else:
    print(f"Total ranging requests: {ranging_requests}")
    imageio.mimsave('simulation.gif', frames, fps=3)
