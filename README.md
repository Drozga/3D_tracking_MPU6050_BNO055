# 3D Mapping with MPU6050 and GY-BNO055 Sensors

This repository contains the code, explanations, and findings from my bachelor's thesis on *3D mapping using the MPU6050 and GY-BNO055 sensors*. The project explores sensor-based motion tracking, enabling the mapping of movement in three-dimensional space through oscillatory, circular, and more complex motion patterns. The insights and results of this research provide a foundation for further applications in fields requiring precise spatial orientation and position tracking.

The simulation of movement tracking was performed in MATLAB. Using sensor data, various movement patterns, including oscillatory, circular, and complex motions, were accurately visualized in a 3D environment. MATLAB's powerful data processing and visualization capabilities allowed for detailed analysis of the sensor data, showcasing the effectiveness of the MPU6050 and GY-BNO055 sensors for 3D mapping applications.


## Project Overview

The project is divided into two primary parts, each documented in its own folder:

1. **Oscillatory and Circular Movements**  
   This folder includes code and documentation on experiments with oscillatory (back-and-forth) and circular motions. Using the MPU6050 and GY-BNO055 sensors, these experiments demonstrate the sensors' ability to capture consistent patterns in controlled environments. The oscillatory movement showcases the sensor's ability to detect repetitive motion, while circular movement highlights rotational tracking capabilities around a central point.
![kruzno](https://github.com/user-attachments/assets/222a9a93-0dda-4818-93cf-83243cb32903)

2. **Complex Movements: Stair-Climbing Simulation**  
   The second folder contains code and detailed analysis of experiments simulating stair-climbing. These tests involve tracking the sensor's position during simulated steps, showcasing how the GY-BNO055 sensor, with its higher accuracy, captures more intricate movements. In these experiments, stationary periods are detected to simulate moments when a foot remains on a step, emphasizing precise 3D mapping under real-world constraints.
![steps](https://github.com/user-attachments/assets/a2bf0473-09e0-4acb-b332-20b38b8ceb2b)


## Conclusions

This project provided valuable insights into the capabilities and limitations of the MPU6050 and GY-BNO055 sensors for 3D mapping applications. The MPU6050 sensor, while affordable and easy to integrate, lacks the precision required for accurate spatial tracking, especially over extended periods. Due to its limited accuracy, I was able to simulate only basic movements, such as oscillatory and circular motions. For more complex movement simulations, like stair-climbing, I transitioned to the GY-BNO055 sensor, which offers significantly higher accuracy and provides pre-filtered data, making it well-suited for such detailed applications.

The MPU6050 sensor still holds potential for future exploration in basic 3D mapping; however, its current limitations make it more suitable as an educational tool for understanding fundamental 3D mapping concepts. Despite its challenges, the MPU6050 remains valuable in applications where absolute accuracy is less critical, providing a low-cost introduction to motion tracking. For precise and reliable 3D mapping, the GY-BNO055 or similar advanced sensors are recommended.


## Repository Structure

- **Oscillatory and Circular Movements**  
  Code and data for basic motion patterns captured by both sensors.

- **Complex Movements**  
  Code, data, and analysis for advanced motion tracking, focusing on simulating stair-climbing movements.

## Getting Started

To replicate or extend this project, each folder contains a README file with specific instructions on setup, dependencies, and usage. Ensure you have the required sensors and follow the provided guidelines for optimal results.
![uredjaj1](https://github.com/user-attachments/assets/0b86bd08-073f-4b47-996c-4419dc2aab01)

**SD:**
![sd](https://github.com/user-attachments/assets/1f4988a2-5587-464e-9dfe-8d60fedf7881)

**SENSOR:**
![mpu](https://github.com/user-attachments/assets/f5a99957-ce67-4d42-8eaf-9fc4eb8fc6a0)



---

Thank you for exploring this repository, and I hope the insights provided here assist others in the field of 3D motion tracking and mapping.
