# Autonomous Race Car with Obstacle Avoidance  
MAE 148 Final Project | Summer Session 2 - Team 1

<img width="509" height="114" alt="截屏2025-09-06 03 57 07" src="https://github.com/user-attachments/assets/72e18831-4872-490a-8546-28c49a2865d7" />


---

## Overview  
This project focuses on developing an autonomous racecar capable of navigating the EBU2 track while avoiding obstacles. The system integrates OAK-D-Lite camera vision for lane detection and LiDAR sensing for obstacle recognition, creating a fusion system that detects traffic cones and adjusts the car’s path in real time.  

---

## Project Goals  

Must Have  
- Implement lane following algorithms using the OAK-D-Lite camera  
- Detect traffic cones using LiDAR/RGB-fusion  
- Plan paths around cones by switching between left and right lane following  

Nice to Have  
- Smooth driving without unnecessary stops  
- Emergency stopping if both lanes are blocked  

---

## Robocar Picture  
<img width="310" height="264" alt="截屏2025-09-06 03 59 03" src="https://github.com/user-attachments/assets/09158eec-4ddb-4ebd-8d26-8af79ea6c69e" />
<img width="299" height="395" alt="截屏2025-09-06 03 59 16" src="https://github.com/user-attachments/assets/269b23a8-f175-468a-b870-d312d97b84ef" />


---

## Hardware Configuration  
- Jetson Nano as the main processor  
- OAK-D-Lite camera for lane detection  
- LiDAR for obstacle sensing (mounted above the camera for better coverage)  
- VESC motor controller  
- 16V battery with a DC-DC converter to supply 5V for the Jetson Nano  
- 3D-printed mounts and chassis components

  We integrated the required sensors—an OAK-D-Lite camera and a LiDAR—with the Jetson Nano for our project. First we 3D printed out the camera mount, jetson nano case, chassis plate to set up all the components we needed. Both the Jetson Nano and the VESC were powered by a 16V battery, but since the Jetson requires only 5V, we added a DC-DC converter to step down the voltage. We positioned the LiDAR on top of the camera mount to provide a clearer field of view and improve the sensing environment.
---

## Software and Code  
We implemented three main ROS2 packages:  
1. Lane Detection Package – processes camera input to follow track lines  
2. Nav2 Package – integrates navigation and obstacle handling  
3. Actuator Package – controls the car’s movement through cmd_vel  

- The lane detection package keeps the car within track boundaries  
- LiDAR data is subscribed via the laser scan topic to detect cones  
- When an obstacle is detected, the actuator package issues stop commands  

---

## Accomplishments  
- Successfully trained a model to detect traffic cones  
- Car can autonomously follow the yellow track line (partial lane following)  
- Car stops when a cone is detected within a set distance (tested on stand)  
- Car resumes moving once the cone is removed (tested on stand)  

---

## Challenges  
- Excessive time spent on model training  
- Issues setting up RViz and LiDAR initially  
- Difficulty interpreting LiDAR data outside of RViz  

---

## Team Members  
- Hanyu Wu  
- Charlie Barber  
- Aaryan Agrawal  
- Aarav Savla  

---

## Acknowledgments  
We thank the UC San Diego Jacobs School of Engineering, Professor Jack Silberman, Alexander Haken, and our classmates for their valuable support and collaboration.  
