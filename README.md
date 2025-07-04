# act_for_franka on Real-World 

This branch enables the application of **[ACT (Action Chunking with Transformer)](https://tonyzhaozh.github.io/aloha/)** to the real-world Franka Panda robot. It specifically uses the **Franka Panda Research 3** model.

## Quick Start

To reproduce our real-world results with the Franka Panda Research3 using ACT, follow the steps below. 

### Environments

* **Operating System** - [Ubuntu 20.04 LTS](https://releases.ubuntu.com/focal/)
* **ROS1 Noetic** - For robot communication and control
* **NUC PC** - For real-time robot control and computation
* **Joystick** - For teleoperation control
  > Logitech Wireless Gamepad F710

  
* **Cameras**
  > **2** x <ins>Intel RealSense L515</ins> (RGB Only) - Left and right viewpoints\
  > **1** x <ins>Intel RealSense D435</ins> (RGB)- Mounted on gripper for close-up viewpoints


---


### Setup 


Follow the steps below to complete the installation on your NUC PC.


**1. Create Conda Environment**
> While not strictly required, we recommend creating a virtual environment to avoid potential version conflicts with your system.
```bash
  $ conda create --name act4fr3 python=3.8.10
  $ conda activate act4fr3
```


**2. Install Python Requirements**

```bash
  $ pip install -r requirements.txt
```


**3. Clone `act_for_franka` Repository (real-world branch)**

```bash
  $ git clone -b real-world https://github.com/jkw0701/act_for_franka.git
```


**4. Download Franka ROS Workspace** 

> A pre-built Franka ROS workspace is provided via [Google Drive](https://drive.google.com/file/d/19SXBxrk2MRcsHXWO2sJT7QOOudFGGgxg/view?usp=sharing)


**5.  Download Realsense Workspace**

> A pre-built Realsense workspace is also provided via [Google Drive](https://drive.google.com/file/d/1KLahQyssJ2mv-CsWITaOr2sm_1KAxmn1/view?usp=sharing)


**6. Launch `roscore` and Source Franka ROS Workspace**
```bash
$ roscore
$ cd franka_ros_ws
$ source devel/setup.bash
```


**7. Check Joystick Connection**
```bash
  $ ls /dev/input/js0 # Ensure joystick is connected 
  $ rosrun joy joy_node # Launch joystick ROS node
  $ rostopic echo /joy # Check published joystick messages 
```


**8. Source Realsense Workspace**
```bash
  $ cd realsense_ws
  $ catkin_make
  $ source devel/setup.bash
```

**9. Configure Your Local Paths**
- Update the dataset and checkpoint paths in [settings/var.py](https://github.com/jkw0701/act_for_franka/blob/sim/settings/var.py) :
```bash
    # settings/var.py

    # Paths
      CHECKPOINT_DIR = 'path/to/your/checkpoints'
      DATASET_DIR = "path/to/your/dataset"
```

> Replace `'/path/to/your/...` with the actual directory on your system.

---


### Record Videos

**1. In the Franka ROS Workspace**
```bash
  $ Home # Run in any bash terminal to send panda to home pose 
  $ cd franka_ros_ws
  $ roslaunch franka_example_controllers cartesian_impedance_example_controller.launch \
    robot_ip:=[your_robot_ip] load_gripper:=true robot:=panda 
  $ roslaunch franka_example_controllers joystick_pose_publishser.launch 
```


**2. In the Realsense Workspace**
```bash
  $ cd realsense_ws
  $ roslaunch realsense2_camera rs_multiple_devices.launch
```


**3. In the ACT repository**
```bash
  $ cd act_for_franka/real
  $ python3 live_record_mod.py
```

---


### Train 


```bash
  $ cd act_for_franka
  $ python3 train.py
```


---


### Evaluate 


```bash
  $ cd act_for_franka/real
  $ python3 real_eval.py
```
