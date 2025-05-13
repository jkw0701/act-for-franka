# act_for_franka on Simulation

This branch enables the application of **[ACT (Action Chunking with Transformer)](https://tonyzhaozh.github.io/aloha/)** to the real-world Franka Panda robot. It specifically uses the **Franka Panda Research 3** model.

## Quick Start

To reproduce our simulation results with the Franka Panda Research3 using ACT, follow the steps below. 

### Environments

* **Operating System** - [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/)
* **ROS1 Noetic** - For robot communication and control
* **Gazebo 11** - For simulation
  
* **Simulation enviroment**
  > **2** x <ins>multisense_sl</ins> (RGB Only) - Left and right viewpoints\
  > **1** x <ins>gripper_camera</ins> (RGB)- Top viewpoints\
  > <ins>stone, red_plate, blue_plate</ins> - pick green stone on red plate to blue plate


---


### Setup 


Follow the steps below to complete the installation on your PC.


**1. Create Conda Environment**
> While not strictly required, we recommend creating a virtual environment to avoid potential version conflicts with your system.
```bash
  $ conda create --name act4fr3sim python=3.8.10
  $ conda activate act4fr3sim
```


**2. Install Python Requirements**

```bash
  $ pip install -r requirements.txt
```


**3. Clone `act_for_franka` Repository (sim branch)**

```bash
  $ mkdir act_for_franka/sim
  $ cd act_for_franka/sim
  $ git clone -b kny https://github.com/jkw0701/act_for_franka.git
```


**4. Download Franka ROS Workspace** 

> A pre-built Franka ROS workspace is provided via [Google Drive](https://drive.google.com/file/d/19SXBxrk2MRcsHXWO2sJT7QOOudFGGgxg/view?usp=sharing)
locate workspace to ros/ws_ACTfranka and build
```bash
$ cd ros/ws_ACTfranka
$ catkin_make
```

**6. Launch `roscore` and Source Franka ROS Workspace**
```bash
$ roscore
$ cd ros/ws_ACTfranka
$ source devel/setup.bash
```

---


### Record Videos

**1. In the Franka ROS Workspace**
```bash
  $ cd ros/ws_ACTfranka
  $ roslaunch panda_gazebo pick_and_place.launch
```


**3. In the ACT repository**
```bash
  $ cd act_for_franka/sim/simulatin
  $ python record_episode.py
```


---


### Train 


```bash
  $ cd act_for_franka/sim
  $ python train.py
```



### Evaluate 


```bash
  $ cd act_for_franka/sim/simulation
  $ python evaluate.py
```

