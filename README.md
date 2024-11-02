# 3D Mapping with MPU6050 and GY-BNO055 Sensors

This repository contains the code, explanations, and findings from my bachelor's thesis on 3D mapping using the MPU6050 and GY-BNO055 sensors. The project explores sensor-based motion tracking, enabling the mapping of movement in three-dimensional space through oscillatory, circular, and more complex motion patterns. The insights and results of this research provide a foundation for further applications in fields requiring precise spatial orientation and position tracking.

## Project Overview

The project is divided into two primary parts, each documented in its own folder:

1. **Oscillatory and Circular Movements**  
   This folder includes code and documentation on experiments with oscillatory (back-and-forth) and circular motions. Using the MPU6050 and GY-BNO055 sensors, these experiments demonstrate the sensors' ability to capture consistent patterns in controlled environments. The oscillatory movement showcases the sensor's ability to detect repetitive motion, while circular movement highlights rotational tracking capabilities around a central point.

2. **Complex Movements: Stair-Climbing Simulation**  
   The second folder contains code and detailed analysis of experiments simulating stair-climbing. These tests involve tracking the sensor's position during simulated steps, showcasing how the GY-BNO055 sensor, with its higher accuracy, captures more intricate movements. In these experiments, stationary periods are detected to simulate moments when a foot remains on a step, emphasizing precise 3D mapping under real-world constraints.

## Conclusions

Through this project, I have developed a deeper understanding of the practical applications and limitations of the MPU6050 and GY-BNO055 sensors for 3D mapping. While the MPU6050 is affordable and easy to use, it presents limitations in accuracy, making it suitable for basic movement detection rather than precise spatial tracking. Conversely, the GY-BNO055 sensor offers higher accuracy with pre-filtered data, enabling it to handle complex motion scenarios, such as stair-climbing.

## Repository Structure

- **Oscillatory and Circular Movements**  
  Code and data for basic motion patterns captured by both sensors.

- **Complex Movements**  
  Code, data, and analysis for advanced motion tracking, focusing on simulating stair-climbing movements.

## Getting Started

To replicate or extend this project, each folder contains a README file with specific instructions on setup, dependencies, and usage. Ensure you have the required sensors and follow the provided guidelines for optimal results.

---

Thank you for exploring this repository, and I hope the insights provided here assist others in the field of 3D motion tracking and mapping.
