# UAV Strategic Deconfliction System

## Overview
This system provides strategic deconfliction for drone missions in shared airspace by checking for conflicts in both space and time against other simulated drone flights.

## Features
- 4D conflict detection (3D space + time)
- Custom trajectory interpolation
- Detailed conflict reporting
- 3D visualization of missions and conflicts

## Installation
1. Clone the repository
2. Install dependencies:
   use below commands open in terminal enter following commands to setup and run.
   
   1.git clone git@github.com:Azaruddin7882/uav_deconfliction_system.git
   2.cd uav_deconfliction_system

   3.pip install -r requirements.txt

   -> For MP4 export, FFmpeg must be installed system-wide (not just a Python package)
   4.sudo apt install ffmpeg

   5. python src/main.py
   6. check output folder inside that ouputs are generated for given inputs
   7. To run unitest cases follow below command
   8. python -m unittest tests/test_conflict.py

