# AIbotix: Optimizing Construction Workflows Through Spatial Cognition and AI Agent in Robotics

<hr>

<div align="center">
  <b>Spatial Cognition, AI Agent 🤖📦</b><br>
  A vision-based robotic system for real-time assembly using ARUCO markers and ROS2.
</div>
<br>

<div align="center">

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-blue)
![Hardware](https://img.shields.io/badge/Robot-UR10e-green)
![Camera](https://img.shields.io/badge/Camera-Orbbec%20Femto%20Bolt-orange)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

</div>

> [!IMPORTANT]
> 📚 **New to this project?** Check out the full demo video and step-by-step guide below!

---

## 🎥 Project Video: AIbotix
A vision-guided robotic system that autonomously assembles wood blocks using ROS2, rgb perception, and ArUco markers.

[![Spot YouTube Thumbnail](https://github.com/Shu980101/AIbotix/raw/main/asset/Start%20Page.jpg)](https://youtu.be/OANXGHBe3eQ)

---

## 🚀 Quick Start

### Requirements

#### ✅ Hardware
- **Robot Arm**: Universal Robots UR10e
- **Camera**: Orbbec Femto Bolt (RGB-D)
- **Gripper**: Suction cup end-effector
- **Calibration Tool**: ArUco marker board

#### 🧠 Software
- **Operating System**: Ubuntu 22.04
- **ROS 2**: Humble Hawksbill
- **Python**: 3.10+
- **Libraries**:  
  - `OpenCV`  
  - `Open3D`  
  - `MQTT`  
  - `pymoveit2`  
  - `numpy`, `transforms3d`, `cv_bridge`  

---

## 📦 Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/Shu980101/AIbotix.git
   cd Aibotix
   ```
   
2. Build image:
   ```bash
   .docker build_image.sh
   ```

3. Run image:
   ```bash
   .docker run_user_nvidia.sh
   ```

---

## 🧰 Launch Instructions

### Start the Camera Node and Aruco Detection
```bash
ros2 launch pick_n_place_bring_up multi_aruco_detection.launch.xml
```

### Run Simulation
```bash
ros2 launch pick_n_place_bring_up pick_n_place.launch.xml sim:=true pipeline:=pilz
```

### Run Pick-and-Place Node

Open ur_commander/scripts/pick_n_place_wrapup.ipynb and follow the steps outlined in the Jupyter Notebook one by one.

> ⚠️ Ensure hand-eye calibration is completed before running the main task.  
> Refer to the **Easy Hand Eye 2** in the Wiki for setup steps.

---
## 🤖 AI Agnent Integration (TBC)

I am integrating an AI agent that interprets multi-source data and dynamically updates robot plans in real time—enabling adaptive, collaborative construction workflows.

---

## 🧪 System Overview

| Module           | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| **Design Input** | Rhino+Grasshopper model with block layout + ArUco ID tagging                |
| **Perception**   | Depth sensing + marker detection using Orbbec + OpenCV                      |
| **Planning**     | Pose transformation with TF2 and calibrated camera-to-robot coordinates     |
| **Execution**    | ROS2 MoveIt2-based pick-and-place with motion planning                      |
| **Bridge**       | Json connects Rhino to ROS for design data                                  |


---

## 📌 Citation

If you use this work, please cite:

```bibtex
@project{shuxiao_aibotix_2025,
  title={AIbotix: Optimizing Construction Workflows Through Spatial Cognition and AI Agent in Robotics},
  author={Shu Xiao},
  institution={IAAC, MRAC 2025},
  year={2025}
}
```

---

## 👤 Author

**Shu Xiao**  
Master in Robotics and Advanced Construction  
[IAAC Barcelona](https://www.iaac.net)

