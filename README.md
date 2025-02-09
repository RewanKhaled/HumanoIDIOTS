# HumanoIDIOTS: A Humanoid for Empowering the Visually Impaired

![Project Banner]
(https://raw.githubusercontent.com/RewanKhaled/HumanoIDIOTS/main/humanoidiot.png)

## 📌 Project Overview
HumanoIDIOT is a cost-effective humanoid robot designed to assist visually impaired individuals. It integrates navigation assistance, face recognition, voice interaction, and health monitoring using **ROS (Robot Operating System)** and **affordable hardware components**. 

## 🚀 Features
- **Navigation Assistance**: Autonomous and manual navigation using IR sensors and ultrasonic sensors.
- **Face Recognition**: Identifies known individuals and alerts the user about unknown persons.
- **Voice Command Interface**: Allows hands-free control using an MIT App Inventor-based mobile application.
- **Health Monitoring**: Includes an oximeter sensor for heart rate and oxygen level monitoring.
- **Cleaning Mode**: Additional functionality for minor cleaning tasks.
- **Simulation Support**: ROS-based simulation using Gazebo for testing and validation.

## 🛠️ Technologies Used
- **Hardware:** ESP32, Arduino Mega, IR sensors, Ultrasonic sensors, Servo & DC motors, MAX30102 Oximeter.
- **Software:** ROS, MATLAB, OpenCV, MIT App Inventor.
- **Power System:** Custom PCB designs for efficient power distribution.

## 📂 Repository Structure
```bash
📦 HumanoIDIOTS
├── 📁 hardware/           # Hardware schematics and PCB designs
├── 📁 software/           # ROS nodes, Arduino scripts, and App Inventor code
├── 📁 simulation/         # Gazebo models and simulation configurations
├── 📁 documentation/      # Reports, papers, and user manuals
├── README.md             # Project overview and setup guide
```

## 📖 How to Use
### 1️⃣ Prerequisites
- Install **ROS (Robot Operating System)**.
- Setup **Arduino IDE** with ESP32 and Arduino Mega libraries.
- Install **MATLAB** (optional for simulation & data processing).
- Download the **MIT App Inventor application** for mobile control.

### 2️⃣ Installation
Clone the repository:
```bash
git clone https://github.com/RewanKhaled/HumanoIDIOTS.git
cd HumanoIDIOTS
```

### 3️⃣ Running the Robot
1. **Power the Robot:** Ensure all components are connected to the power PCB.
2. **Launch ROS Nodes:**
   ```bash
   roslaunch humanoidiot main.launch
   ```
3. **Run Face Recognition:**
   ```bash
   python3 face_recognition.py
   ```
4. **Use Mobile App:** Install the mobile app and connect via Bluetooth.
5. **Monitor Health Data:** Heart rate and SpO2 data will be displayed on the LCD and sent to the mobile app.

## 🔬 Performance Metrics
| Feature | Accuracy | Latency |
|---------|---------|---------|
| Face Recognition | 95% | 1.2s |
| Voice Commands | 90% | 2s |
| Obstacle Avoidance | 98% | 0.5s |
| Health Monitoring | ±2 BPM / ±2% SpO2 | 1s |

## 🏗️ Future Improvements
- **LiDAR integration** for enhanced navigation.
- **AI-based voice assistant** for improved interaction.
- **Expanded health monitoring** with blood pressure sensors.
