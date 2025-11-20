# Self Balancing Prism

An inverted-pendulum device that balances on its edges using a reaction-wheel. This project demonstrates the integration of control systems, embedded hardware, and mechanical design optimization into one self-stabilizing system.

### Watch the video

[![Watch the video](https://img.youtube.com/vi/W3pju5vKch4/0.jpg)](https://www.youtube.com/watch?v=W3pju5vKch4)

### Check out the interactive project render     

<a href="https://sarveshpatil7.github.io/Self-Balancing_Prism/">
  <img src="https://github.com/SarveshPatil7/Self-Balancing_Prism/blob/main/docs/images/Frame000373.png?raw=true" alt="View Render" width="500"/>
</a>

### Assembly Procedure

[![Watch the video](https://img.youtube.com/vi/uf6nLatY1kM/0.jpg)](https://youtube.com/shorts/uf6nLatY1kM)
---

### System Architecture

| Component | Function |
|------------|-----------|
| **ESP32-S3 DevKitC** | Central controller handling sensor fusion, PID control, and BLE telemetry |
| **MPU6050 IMU** | Provides real-time accelerometer and gyroscope data |
| **MT6701 Magnetic Encoder** | Measures precise angular position of the reaction wheel |
| **BLDC Motor + DRV 8313 based SimpleFOC mini Driver** | Executes field-oriented control (FOC) for torque generation |
| **3D-Printed Enclosure** | Lightweight structure optimized using **topology optimization** in Abaqus to reduce mass while maintaining stiffness |
| **21700 battery pack** | 7.4V power supply |
| **MP1584EN** | DC-DC Buck converter for 5V supply to the ESP32 |
| **MT3608** | DC-DC Boost converter for 12V supply to the motor driver |

---

### Control Flow  

1. **Sensor Task** — A FreeRTOS task reads the IMU and stores shared variables.  
2. **Control Task** — Runs at ~1 kHz, computes filtered angle (`tri_angle`), PID output, observer term, and commands motor torque.  
3. **Calibration** — Computes gyro bias and initial tilt before engaging control.  
4. **Safety & Windup Handling** — Monitors excessive tilt and resets integral term.  

---

### Future Improvements
- Integrate **BLE-based tuning interface** for control parameters    
- Install the topology optimized enclosure plates (not fabricated yet)
- State machine for stand‑up / fall‑recovery / locomotion modes
  
---
