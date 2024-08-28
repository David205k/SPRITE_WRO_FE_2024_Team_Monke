# SP RITE WRO FE 2024

## Team Information:
![image](https://github.com/user-attachments/assets/395f8e3e-d5fa-4c2a-b530-951f97a5dc62)

### Team ID:			`FE24_324_02`


### Team Name: 		`Team Monke` 


### Team Members: 	
- **Lim Jun Hong David**
- **Chong Qi Yuan**
- **Sachin Ilangovan**

## Overview
#### Introduction	
Hello and welcome to Team `FE24_324_02` (Team Monke) GitHub repository. This is the GitHub repository for SINGAPORE World Robot Olympiad Future Engineers 2024. For WRO FE 2024, the competition focuses on Self-Driving Cars and in the challenge a robotic vehicle needs to drive autonomously on a parkours that randomly changes for each competition round.

In an ever-increasing automated world, the use of Autonomous Guided Vehicles (AGV) has become more prevalent. Furthermore, with recent improvements in Artificial Intelligence, specifically Computer Vision (CV), the integration of CV with AGVs is in high demand. Some possible applications with this integration include: 
-	Commercial: Self-Driving cars, housekeeping robots
-	Industry: Fully autonomous factory robots
-	Defence: Autonomous drones

This `README` will provide you with a quick overview of our process and resources we use for the competition. If you prefer a more thorough and detailed documentation of our journey for this competition, please check out our Engineering Journal that is also available in this GitHub repository `doc/`. Thank you


## Repository Content

- `doc/`:  Documentation
  - `Engineering Documentation.docx`
  - `Parts list`

- `models/`:
  - `Vehicle 3D Models` all ipt & iam files for vehicle design
    
  - `Engineering Drawings`
    - `Bottom Section Exploded view.png`
    - `Isometric view.png`
    - `Orthographic view.png`
    - `Top Section Exploded view.png`
    - `bottom disassembly.dwg`
    - `top disassembly.dwg`
    - `vehicle isometric view.dwg`
    - `vehicle orthogonal drawing.dwg`

  - `Disassembly Video`
    - `bottom disassembly.ipn`
    - `bottom disassembly.mp4`
    - `top disassembly.ipn`
    - `top disassembly.mp4`
  - `MONKE_BUST.stl`
  - `WRO_FE_2024.ipj.lnk`

- `ref/`: References for design idea & WRO_FE Rules, Field overview
  - ``

- `schemes/`: schematic diagrams of the electromechanical components illustrating all the elements used in the vehicle and how they connect to each other
  - `PCB Schematic Diagram.png`
  - `PCB ver.1.jpg`
  - `WRO_FE_PCB.fzz`
  - `WRO_FE_etch_copper_bottom.pdf`
  - `WRO_FE_schematic.fzz`

- `src/`: All python raspberry pi code to control software for all components
  - `main`
    - `main.py`
    - `pwmControl.py`
    - `tb6612fng.py`
    - `ultrasonicSensor_control.py`
  - `basic openCV/`:  basic programs using opencv and distance for objectDetection
    - `objectDetection.py`
    - `draw.py`
    - `read images.py`
    - `read videos.py`
  - `basic programs/`
    - `servo motor control ver.2.py`
    - `tb6612fng_motorDriver_control.py`

- `t-photos/`: 2 photos => Official Team photo and funny Team photo
  - `Official.png`
  - `Funny.png`

- `v-photos`: 6 photos of vehicle from all 6 side
  - `Top.png`
  - `Right.png`
  - `Left.png`
  - `Back.png`
  - `Front.png`
  - `Bottom.png`

- video: YouTube Video of vehicle demonstration in challenge
  - `Open Challenge.md`
  - `Obstacle Challenge.md`

- .gitignore: gitignore file
- README.md: This file

#### Hardware used

- MG996R Servo Motor
- 12v 380rpm 1.4kgcm Brushed DC Motor
- Full Metal Differential Gear Set
- Raspberry Pi 4 Model B
- High precision Ultrasonic Range Finder US-015
- Raspberry Pi Camera Module 3
- TB6612FNG Motor Driver
- Ball Bearings 6x13x5 mm
- 80mm Plastic Wheels
- Small Plastic Wheels SPG330/SPG50 (80mm)
- HMC5883L Module Triple Axis compass
- Turnigy Lipo Pack 5000mAh 3S 25C W/XT-90
- LM2596 3A Buck Module with Display
- Custom designed PCB
