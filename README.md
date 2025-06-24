# Hovercraft Project

## Overview

This repository contains the code, hardware designs, and documentation for our Autonomous Hovercraft built with an Arduino Nano, MPU6050 IMU, ultrasonic sensors, and hobby servos. The craft navigates a predefined course by detecting obstacles, scanning for open paths, and executing precise IMU-assisted turns. Throughout development, our team adopted a "learn-as-we-go" approach, iterating on both hardware and software to achieve reliable performance.

## Team Achievement

Our hovercraft secured **1st place** in the ENGR 290 challenge, demonstrating exceptional speed, accuracy, and robustness under timed conditions.

## Key Features

* **Autonomous Obstacle Detection**: Front and overhead ultrasonic sensors detect walls and bars.
* **IMU-Based Turning**: Real-time yaw integration from MPU6050 for precise 90° turns.
* **Drift Correction**: Continuous servo adjustments maintain straight-line travel.
* **Path Scanning**: Servo-mounted sensor sweeps to identify the best opening.
* **Adjustable Fan Control**: Software PWM on Arduino Nano to vary thrust fan speed during turns.
* **Learn-As-You-Go Documentation**: Inline comments and clear module separation to help new contributors understand each step.

## Hardware Components

* **Microcontroller**: Arduino Nano
* **IMU**: MPU6050 (accelerometer + gyroscope)
* **Ultrasonic Sensors**: HC-SR04 modules for front and upward detection
* **Servos**: HS-422 hobby servos for pan/tilt and thrust vectoring
* **Fans**: Brushless DC fans controlled via digital and software PWM

## Software Architecture

1. **`calibrateGyroZ()`**: Samples gyro readings to compute Z-axis bias.
2. **Ultrasonic Routines**: Fast inline trigger functions for reliable distance measurements.
3. **Scan & Turn Logic**:

   * On wall detection, fans shut off and servo scans from 0°–180°.
   * Chosen angle directs IMU-driven turn routine.
4. **Drift Correction**: Integrates filtered yaw rates to adjust servo and counter drift.
5. **Fan Speed Control**: Software PWM maintains thrust levels, adjustable per mode.

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/aburiz/Hovercraft.git
cd Hovercraft
```

### 2. Install Dependencies

* Arduino IDE (version 1.8.13 or later)
* MPU6050 library: `https://github.com/jrowberg/i2cdevlib`
* Servo library (built-in)

### 3. Hardware Wiring

1. Connect MPU6050 to I2C (A4 = SDA, A5 = SCL).
2. Front HC-SR04: TRIG → D13, ECHO → D3.
3. Upward HC-SR04: TRIG → D11, ECHO → D2.
4. Thrust fan control → D4 (software PWM).
5. Lift fan control → D7 (digital on/off).
6. Servo signal → D9, power servos from external 5 V supply.

### 4. Upload Code

Open `Hovercraft.ino` in Arduino IDE, select "Arduino Nano" and appropriate COM port, then click Upload.

## Usage

1. Place the hovercraft at the start line.
2. Power on; the craft will calibrate its gyro for 5 s.
3. It will then engage lift and thrust fans and begin autonomous navigation.
4. On detecting a wall: fans stop, scan for open path, perform IMU-based turn, then resume.

## Learn-As-You-Go Reflections

1. **Calibration Matters**: Spending extra time on gyro bias calibration dramatically improved turn accuracy.
2. **Software PWM on Digital Pins**: Emulating PWM allowed fine thrust control without using hardware PWM pins.
3. **Iterative Testing**: Rapid prototyping cycles—adjusting delays, gains, and thresholds—were key to reliable behavior.
4. **Documentation**: Inline comments and modular functions made debugging and team collaboration much smoother.

## Competition Outcome

Our hovercraft outperformed all other entries by completing the course in the shortest time. The robust scanning and IMU-based control proved decisive in tight turns and unexpected obstacles.

## Gallery


## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
