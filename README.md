# Engineering Materials

This repository contains engineering materials for a self-driven vehicle developed for the WRO Future Engineers competition, including hardware documentation, electromechanical schematics, software, and supporting media.

## Introduction

This project implements a dual-compute architecture where a Raspberry Pi 5 performs vision and high-level decision making while an ESP32 executes low-level sensing and motion control.  
The hardware stack integrates a BNO055 IMU for orientation, TF Luna LiDAR and ultrasonic sensors for distance and obstacle detection, a TB6612FNG motor driver for an N20 DC motor, and an MG90 servo for steering, powered by a 3S Li‑ion battery for the drive system and a separate powerbank for the Raspberry Pi.  
Control software includes Arduino sketches for bot control and low‑level sensing, and Python scripts for the Raspberry Pi for vision and serial communication

## Content

- t-photos — A directory with indivisual team photos.  
- v-photos — This includes six view vehicle photos (front, back, left, right, top, bottom).
- video — A folder containing a video.md file linking to a driving demonstration.
- schemes — Diagrams detailing all electromechanical connections and components.  
- src — Source code for programmed components, including ESP32 Arduino sketches for ultrasonic sensing, IMU heading estimation, servo steering, motor control, and turn logic; and Raspberry Pi Python skethches for vision sensing and serial communication. 
- models — Fabrication files for 3D‑printed parts used in the robot.
- other — Contains documentation including the Enginnering Notebook detailing our journey, the bot, and the process of making it.

