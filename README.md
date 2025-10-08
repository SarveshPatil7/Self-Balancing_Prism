# Self Balancing Prism

An inverted-pendulum device that balances on its edges using a reaction-wheel. This project demonstrates the integration of control systems, embedded hardware, and mechanical design optimization into one self-stabilizing system.

See the project render at https://sarveshpatil7.github.io/Self-Balancing_Prism/

---

# System Architecture

| Component | Function |
|------------|-----------|
| **ESP32-S3 DevKitC** | Central controller handling sensor fusion, PID control, and BLE telemetry |
| **MPU6050 IMU** | Provides real-time accelerometer and gyroscope data |
| **MT6701 Magnetic Encoder** | Measures precise angular position of the reaction wheel |
| **BLDC Motor + SimpleFOC Driver** | Executes field-oriented control (FOC) for torque generation |
| **3D-Printed Enclosure** | Lightweight structure optimized using **topology optimization** in Abaqus to reduce mass while maintaining stiffness |

---

## Future Improvements
- Integrate **BLE-based tuning interface** for PID parameters   
- Install the topology optimized enclosure plates (not fabricated yet)
  
---
