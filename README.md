# Automated Chess Board Setup with Delta Parallel Robot

Computer vision pipeline for autonomous chess piece detection and placement 
using a delta parallel robot and YOLOv5m. Published at **ICRoM 2023**.

📄 [Read the paper](YOUR_PAPER_LINK_HERE)

## Results
- 90% mAP on chess piece detection (YOLOv5m)
- 95% grasping success rate
- 20% reduction in top-view detection errors via camera calibration

## System Overview
1. Camera detects chessboard corners → computes perspective transform
2. YOLOv5m detects chess pieces and classifies them
3. `camera.py` converts 2D pixel coordinates → 3D robot coordinates
4. Delta robot picks each piece and places it on the correct starting square

## Hardware
- Delta parallel robot (TCP/IP control)
- Robotiq 2F-85 gripper (serial)
- Top-mounted USB camera

## Setup
```bash
pip install -r requirements.txt
```
1. Update `config.py` with your robot's IP, port, and camera index
2. Run `Robot_camera_offset_calibrating.py` to calibrate camera-to-robot offset
3. Run `python finalVersion.py`

## Repository Structure
| File | Purpose |
|------|---------|
| `finalVersion.py` | Main pipeline — board detection, piece detection, pick & place |
| `delta_manager.py` | High-level robot API |
| `client.py` | TCP socket communication with robot controller |
| `camera.py` | Lens undistortion + pixel→3D coordinate transform |
| `config.py` | Robot IP, port, camera index |
| `Robot_camera_offset_calibrating.py` | Camera-robot calibration utility |
| `parameters/` | Pre-calibrated camera matrices and offsets (.npy) |

*M.Sc. Mechanical Engineering (Robotics & Mechatronics), 
Politecnico di Milano · B.Sc. University of Tehran*