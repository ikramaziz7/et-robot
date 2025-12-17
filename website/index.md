# Resource Mining Robot – OS Project

## Team

* **Member 1** – Hardware & Movement
* **Member 2** – Sensors & Strategy
* **Member 3** – Integration & Website

---

## Project Overview

This project was developed as part of the *Operating Systems* course. The goal is to design and program an autonomous robot capable of navigating an arena, detecting resources or obstacles, and executing a predefined strategy.

The robot is built using a LEGO Mindstorms platform and programmed in **C** under a Linux-based environment. The software architecture is modular, separating low-level hardware control from high-level decision making.

---

## Robot Architecture

### Hardware Architecture

The robot consists of:

* A **differential drive base** with two independently controlled wheels
* A **vertical mechanical structure** used to transport or push resources
* One or more **sensors** (ultrasonic and/or color sensor) for environment perception
* Motors controlled through the EV3 interface

*(Insert labeled photo of the robot here)*

### Software Architecture

The software is divided into independent modules:

* `main.c` – Program entry point and main control loop
* `movement.c` – Low-level motor control and robot movement
* `sensors.c` – Sensor access and abstraction
* `strategy.c` – High-level decision making and robot behavior

This modular design improves readability, testing, and maintenance.

---

## Algorithms and Strategy

### Global Behavior

The robot operates in a continuous control loop:

```
while robot is running:
    read sensors
    if obstacle or resource detected:
        execute avoidance or collection behavior
    else:
        move forward
```

### Navigation Strategy

* The robot moves forward by default
* When an object is detected in front of the robot, it reacts by stopping, turning, or activating the vertical mechanism
* The behavior is intentionally simple to ensure robustness during competition

This strategy allows autonomous operation without external control.

---

## Usage Instructions

### Compilation

To compile the project:

```
make
```

### Execution

To run the program on the robot:

```
./robot
```

Ensure that the robot is powered on and all sensors and motors are properly connected before execution.

---

## Development Process

* Version control managed using **GitLab**
* Each team member contributed code and documentation
* Regular testing was performed directly on the robot

The project emphasizes both technical implementation and clear documentation.

---

## Media

* Photos of the robot during assembly
* Videos demonstrating autonomous behavior and movement

*(Add images/videos here)*

---

## Conclusion

This project demonstrates the design of a modular embedded system combining hardware control, sensor feedback, and autonomous decision making. The chosen architecture and strategy prioritize clarity, reliability, and ease of explanation.
