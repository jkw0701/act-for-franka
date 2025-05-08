# act_for_franka : Action Chunking with Transformer for Franka Panda

This Repository provides the [**ACT (Action Chunking with Transfomer)**](https://github.com/tonyzhaozh/act) Algorithms to the **Franka Emika Panda Research Robot Arms** for performing multiple manipulation tasks.
The system was developed for both <ins>simulation</ins> and <ins>real-world</ins> environments, and its result was verified through a series of experiments in both settings.

### Credits 
> The Simulation environment was derived from **[ACTfranka](https://github.com/sainavaneet/ACTfranka)**, But several modifications were made to suit our tasks and architecture.
>
> 
> Special thanks to the original contributors for their foundational work. 
<br />

## Prerequisites
To run this project, ensure the following tools and hardware system are available.

* **Operating System** - [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/)
* **ROS1 Noetic** - For robot communication and control
* **Gazebo** - Simulation environment
* **NUC PC** - For real-time robot control and computation
* **Joystick** - For teleoperation control
* **Cameras**
  > **2** x <ins>Intel RealSense L515</ins> (RGB Only) - Left and right viewpoints\
  > **1** x <ins>Intel RealSense D435</ins> (RGB)- Mounted on gripper for close-up viewpoints

  **`Optional Tools`**
  
  * **Conda Virtual Environment** - Recommended for managing dependencies cleanly
  * **Text Editor** e.g., [Visual Studio Code](https://code.visualstudio.com/)
<br />

## Quick Start Guide 
A step-by-step guide to running ACT in both gazebo simulation and real-world settings. 

### Simulation
- [Requirements](나중 readme url로 채우기)
- [Record Episodes](https://github.com/jkw0701/ACTFranka_jkw/blob/kny/simulation/record_episodes.py)
- [Training](https://github.com/jkw0701/ACTFranka_jkw/blob/kny/continued_train.py)
- [Evaluation](https://github.com/jkw0701/ACTFranka_jkw/blob/kny/simulation/evaluate.py)
  
### Real-World
- [Requirements](나중 readme url로 채우기)
- [Record Episodes](https://github.com/jkw0701/ACTFranka_jkw/blob/real-world/real/live_record_mod.py)
- [Training](https://github.com/jkw0701/ACTFranka_jkw/blob/real-world/train.py)
- [Evaluation](https://github.com/jkw0701/ACTFranka_jkw/blob/real-world/real/real_eval.py) 
<br />

## Experiments 
### Simulation 
![record videos in gazebo simulation](https://github.com/user-attachments/assets/cc8b541b-66fa-443d-922a-480361df50bc)
![evaluate in gazebo simulation](https://github.com/user-attachments/assets/1dc99504-630e-440e-8426-7806122326e8)

### Real-World
![record videos in real-world](https://github.com/user-attachments/assets/2e42247b-fa25-49c3-ab0f-ac18bd06ea19)
![evaluate in gazebo simulation](https://github.com/user-attachments/assets/09a474c4-1821-4dbe-bf42-105b2e3d501a)
![evaluate in gazebo simulation](https://github.com/user-attachments/assets/5a200755-d5c3-4643-a677-b870d9c4087c)
<br />

## About 
### Related 
### License

