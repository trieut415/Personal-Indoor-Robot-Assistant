# Personal Indoor Robot Assistant

**Date**: December 10, 2024  

---

## Overview

The **Personal Indoor Robot Assistant** project demonstrates the design and implementation of a wheeled robot capable of autonomous navigation through predefined waypoints, real-time obstacle avoidance, and remote control functionality. Using the ESP32 microcontroller, the robot integrates indoor localization via the Optitrack system, obstacle detection through Sharp-IR sensors, and real-time data visualization through a Streamlit interface.

This project combines advanced sensor integration, feedback control, and network communication to create a versatile indoor robot assistant. It is a proof-of-concept for personal and industrial robot systems requiring precise navigation and multi-modal control.

---

## Key Features

1. **Autonomous Navigation**
   - Follows predefined waypoints using real-time positional data from the Optitrack system.
   - Implements PID control to maintain a trajectory within ±10cm of the target path.

2. **Obstacle Avoidance**
   - Uses a Sharp-IR distance sensor for real-time detection and avoidance of obstacles.
   - Prioritizes collision avoidance over strict path adherence.

3. **Remote Control Mode**
   - Enables manual control via WASD keyboard inputs.
   - Switches seamlessly between manual and autonomous modes.

4. **Real-Time Monitoring and Visualization**
   - Displays robot positions and waypoints in a 2D graphical interface using Streamlit.
   - Logs positional data in TingoDB for historical analysis and visualization.

5. **Concurrent Robot Operation**
   - Supports multiple robots on the same track, with position updates and collision avoidance coordinated over a shared network.

---

## System Design

### Hardware Integration

1. **ESP32 Microcontroller**
   - Central component for motor control, navigation logic, and network communication.
   
2. **Sharp-IR Distance Sensor**
   - Detects obstacles and triggers avoidance maneuvers.
   - Configured with ADC for calibrated distance measurements in millimeters.

3. **Motors and H-Bridge**
   - Controlled using PWM (via LEDC) for precise speed and directional control.

4. **Optitrack Indoor Positioning System**
   - Provides highly accurate positional data (`x, z, theta`) via the Motive software.
   - Coordinates received from a Node.js server over UDP.

---

### Network Communication

- **Node.js Server**
  - Receives position data from the Optitrack system and relays it to robots over a shared Wi-Fi network.
  - Processes user commands for remote control and mode switching.

- **UDP Protocol**
  - Handles low-latency communication between the robots and the server.
  - Ensures real-time updates for position, commands, and status.

---

### Control Systems

1. **Autonomous Navigation**
   - Predefined waypoints guide the robot along a figure-8 track.
   - PID control dynamically adjusts motor speeds to correct heading errors and maintain smooth navigation.

2. **Obstacle Avoidance**
   - Priority-based processing ensures obstacle detection overrides path adherence during navigation.

3. **Manual Mode**
   - WASD inputs control motor states, allowing users to navigate the robot manually.
   - `P` key toggles between manual and autonomous modes.

---

## Software Components

### FreeRTOS Tasks

1. **Motor Control Task**
   - Implements a Finite State Machine (FSM) for directional control:
     - Forward, backward, left, right, and stop states.
     - Dynamic speed adjustments based on PID output or manual commands.

2. **Sensor Integration Task**
   - Reads Sharp-IR sensor data for obstacle detection.
   - Converts ADC readings to distances in millimeters.

3. **Position Request Task**
   - Periodically queries the Node.js server for real-time position updates.
   - Parses responses and updates the robot's internal navigation state.

4. **Navigation Task**
   - Calculates distance and heading to the next waypoint.
   - Invokes PID corrections for precise trajectory adjustments.

5. **Remote Control Task**
   - Listens for WASD and mode-switch commands from the server.
   - Processes commands and updates motor states in real time.

---

### Visualization and Data Management

1. **Streamlit Interface**
   - Displays real-time 2D visualization of robot positions and waypoints.
   - Accessible from any browser connected to the network.

2. **TingoDB Integration**
   - Logs positional data for historical analysis and real-time updates.
   - Ensures persistent storage of robot status and track progress.

---

## Results and Achievements

### Achievements

1. **Autonomous Navigation**
   - Successfully traversed a figure-8 track with ±10cm accuracy using PID control.

2. **Obstacle Avoidance**
   - Sharp-IR sensors reliably detected and avoided obstacles in real time.

3. **Remote Control**
   - WASD inputs provided responsive and intuitive manual control.

4. **Visualization**
   - Real-time graphical display of robot positions and waypoints via Streamlit.

5. **Concurrent Operation**
   - Two robots operated simultaneously on the same track without collisions.

---

### Challenges

1. **Real-Time Communication**
   - Ensuring reliable UDP message passing for position updates and command handling in a shared network.

2. **PID Control Tuning**
   - Iteratively adjusting proportional, integral, and derivative gains to achieve stable navigation without oscillations.

3. **Sensor Calibration**
   - Fine-tuning the Sharp-IR sensor for accurate distance readings in varying environments.

---

## Potential Improvements

1. **Hardware Enhancements**
   - Use hardware interrupts for Sharp-IR sensor readings to improve performance.
   - Upgrade motors for smoother and more precise control.

2. **Network Optimization**
   - Implement error-checking and retry mechanisms for UDP communication.

3. **Enhanced Visualization**
   - Add real-time 3D visualization of the robot’s environment to complement the 2D Streamlit interface.

---

## Demonstrations

- [Report Video](https://drive.google.com/file/d/1IIHNn9ovCAJhkweRPx5khYDsRs2hRVlT/view?usp=sharing)  
- [Design Walkthrough](https://drive.google.com/file/d/1VPvRS4MaWk1iu6lj2RG2DcL4Mfvxja_7/view?usp=sharing)  

---

## Conclusion

The **Personal Indoor Robot Assistant** project successfully integrates autonomous navigation, obstacle avoidance, and real-time monitoring into a cohesive system. Leveraging the ESP32, Optitrack positioning, and Sharp-IR sensors, this project demonstrates the potential for scalable and efficient indoor robot applications. Future improvements will enhance robustness, scalability, and user experience, paving the way for advanced personal and industrial robot systems.
