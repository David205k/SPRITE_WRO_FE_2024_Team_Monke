# SP RITE WRO FE 2024

## Team Information:
![image](https://github.com/user-attachments/assets/395f8e3e-d5fa-4c2a-b530-951f97a5dc62)

### Team ID:			`FE24_324_02`


### Team Name: 		`Team Monke` 


### Team Members: (from left to right)

- **Lim Jun Hong David <** [davidlimjunhong@gmail.com](mailto:davidlimjunhong@gmail.com) >
- **Chong Qi Yuan <** [Qiyuan146@gmail.com](mailto:Qiyuan146@gmail.com) >
- **Sachin Ilangovan <** [sachin5iconic@gmail.com](mailto:sachin5iconic@gmail.com) >

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
  - `References.txt`

- `schemes/`: schematic diagrams of the electromechanical components illustrating all the elements used in the vehicle and how they connect to each other
  - `PCB Schematic Diagram.png`
  - `PCB ver.1.jpg`
  - `WRO_FE_PCB.fzz`
  - `WRO_FE_etch_copper_bottom.pdf`
  - `WRO_FE_schematic.fzz`
  - `front_PCB.png`
  - `Back_PCB.png`
  - `Soldered_PCB.jpg`

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
| Hardware              | Description       | Price (SGD)|
|-----------------------|-------------------|-------------|
| Raspberry Pi 4 Model B | Control unit      | $82.99 |
| MG996R Servo Motor    | Steering actuator | $10.20 |
| 12v 380rpm 1.4kgcm Brushed DC Motor/ <br> DC 12V 1000RPM Gear motor | Main actuator | $19.64 |
| Full Metal Differential Gear set| Differential gear | $10.68|
| TB6612FNG Motor Driver | DC motor driver | $13.95 |
| LM2596 3A Buck Module with Display | DC voltage step down regulator | $4.96 |
| Raspberry Pi Camera Module 3 | Camera for Computer Vision | $6.99|
| US-015/ <br> VL53L1X Time of Flight sensor| High precision Ultrasonic Range Finder <br> / Long range, High precision IR Range Finder| $3.54 |
| HMC 5883L | Triple Axis compass | $12.80 |
| Turnigy Lipo Pack 5000mAh 3S 25C W/XT-90/ <br> Turnigy Lipo pack 1300mAh 3S | Battery | $32.58 |
| 80mm Plastic Wheels SPG30/SPG50 | Main Driving wheel | $1.78 |
| Small Plastic Wheel | 34mm Steering Wheel | $2.00 |
| Ball Bearing | 6x13x5mm ball bearing | $3.41 |
| Printed Circuit Board | Custom PCB for mounting components | $5 |

For links to website for purchase of components, visit our either our /partslist or /engineering journal

## Overview of Robot
![image alt](https://github.com/David205k/SPRITE_WRO_FE_2024_Team_Monke/blob/main/v-photos/overview3.jpg?raw=true)
### Chassis
#### Steering
Originally, our robot featured a tricycle steering design. Where the steering is done directly by the servo turning (like a bicycle). This allowed us to simplify the steering design and also achieve tighter turns. 
However, we encountered some difficulties. Therefore we decided to swap to the ackerman steering for the internationals. 
This mainly offered advantageous in steering robustness, control and stability. However, this came at the cost of a more complex mechanism and a larger turning radius. 
#### Driving
Our robot is using a standard differential gear drive system, where 1 DC motor powers both wheels via a differential gear. A differential gear is used to prevent skidding and offer more stability and control. 

### Sense
Computer Vision: Our robot uses a Raspberry Pi to perform OpenCV computer vision to enable detection and avoidance of the obstacles. 
Distance Sensing: Our robot uses Time of Flight/Ultrasonic sensors to detect distances of walls, obstacles etc.
Vehicle direction: Our robot uses a Magnetometer Compass to obtain the bearing of the robot to enable accurate turns. 
By using this fleet of sensors, our robot is able to make informed decisions based on the information. 

For a more comprehensive overview of our design, proceed to our engineering document at `doc/`. 

