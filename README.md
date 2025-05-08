# act_for_franka on Real-World 

This branch enables the application of [ACT (Action Chunking with Transformer)](https://tonyzhaozh.github.io/aloha/) to the real-world Franka Panda robot. It specifically uses the Franka Panda Research 3 model.

## Quick Start

### Environments

* **Operating System** - [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/)
* **ROS1 Noetic** - For robot communication and control
* **NUC PC** - For real-time robot control and computation
* **Joystick** - For teleoperation control
  > Logitech Wireless Gamepad F710

  
* **Cameras**
  > **2** x <ins>Intel RealSense L515</ins> (RGB Only) - Left and right viewpoints\
  > **1** x <ins>Intel RealSense D435</ins> (RGB)- Mounted on gripper for close-up viewpoints


### Setup 

1. Create Conda Environment
   *  While not strictly required, we recommend creating a virtual environment to avoid potential version conflicts with yours
```
  $ conda create --name act4fr3 python=3.8
