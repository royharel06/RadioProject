# Drone Navigation via LoRa Ranging Simulation

This project simulates a drone trying to return to a home position using only distance-based ranging information from LoRa SX1280-like modules. The goal is to evaluate how a drone can navigate in 2D space using a minimal number of ranging queries to nearby anchors and accounting for realistic distance measurement noise.

## Project Overview

- The drone starts at a random distance between 200–500 meters from its home.
- Multiple fixed LoRa-style anchors are placed randomly around the home.
- Each anchor can provide noisy ranging data (distance measurement).
- The drone uses multilateration to estimate its position and moves toward home.
- The simulation ends when the drone is within a 10-meter radius of home.
- A GIF of the full flight path is generated as output.

## Behavior Modes

You can choose between:
- **Frequent ranging:** The drone estimates its position on every step using ranging data.
- **Lazy/Smart ranging:** The drone remembers its last direction and only ranges when it's uncertain about its trajectory.

## Requirements

Install dependencies using:

```bash
pip install -r requirements.txt
```

## Running the Simulation

```bash
python sim.py
```

You will be prompted for:
- Number of anchors.
- Behavior mode (frequent vs minimal ranging).

## Output

- `simulation.gif`: shows the drone's path and anchor positions.

## Inspired By

This project is inspired by:
- Real-world performance of the [Semtech SX1280](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1280) module.
- Papers like “Outdoor Ranging and Positioning Based on LoRa Modulation” (Müller et al., 2021) which reported ~46m mean ranging error over ~1.4km.

## Examples

Here are sample simulations showing the behavior under different conditions:

### Smart Mode (confidence-based)
  ![Smart Example](examples/Smart%20Example%204%20Requests.gif)
  The drone confidently navigates to the target with minimal ranging.

### Frequent Mode (noisy environment)
  ![Standard Frequency](examples/Frequency%20Mode.gif)
  The drone uses frequent ranging and successfully reaches home.
  
  ![Error Frequency](examples/Frequency%20Mode%20Error%20Example.gif)
  Due to high noise and error accumulation, the drone ends up lost despite frequent updates.

These visualizations help demonstrate the difference between naive frequent updates and confidence-based optimization strategies."# RadioProject" 
