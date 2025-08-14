# Franka_Manipulation
Manipulate a Franka Emika Robot Arm in Simulation using various algorithms classical and learned algorithms.  

Instructions:  
Clone this repo, create a conda environment and install the requirements.  
```
conda create -n libfranka python=3.10  
conda activate libfranka
cd Franka_Manipulation
pip install -r requirements.txt
```
To run the simulator use this command:  
```
run-franka-sim-server -v
```
You might need to hold the option key (if using Mac) and drag robot to the center of the simulation.  
All the algorithms will generate robot motions in this simulation.  

### Classical Control  
In another terminal window, activate the same conda env and navigate to the libfranka-python subfolder:  
```
conda activate libfranka
cd Franka_Manipulation/libfranka-python/Classical_Control
```

- [x] Sinusoidal y-axis joint 4 motion
```
python sinusoidal_motion.py
```
<div style="text-align: left;">
<video style="width: 300; height: auto;" controls>
  <source src="https://github.com/PRISHIta123/Franka_Manipulation/blob/main/libfranka-python/Classical_Control/demos/franka_sinusoidal.mp4" type="video/mp4">
</video>
</div>


- [x] Pick and place joints 0 and 4 motion
```
python pick_and_place.py
```
<div style="text-align: left;">
<video style="width: 300; height: auto;" controls>
  <source src="https://github.com/PRISHIta123/Franka_Manipulation/blob/main/libfranka-python/Classical_Control/demos/franka_pick_place.mp4" type="video/mp4">
</video>
</div>

- [x] Proportional-Derivative (PD) control  
  - Follow reference joint trajectories
```
python pd_control.py
```
<div style="text-align: left;">
<video style="width: 300; height: auto;" controls>
  <source src="https://github.com/PRISHIta123/Franka_Manipulation/blob/main/libfranka-python/Classical_Control/demos/franka_pd_control.mp4" type="video/mp4">
</video>
</div>

- [x] Inverse Kinematics (IK) / Operational Space Control    
  - Cartesian control of the end-effector
```
python inverse_kinematics_osc.py
```
<div style="text-align: left;">
<video style="width: 300; height: auto;" controls>
  <source src="https://github.com/PRISHIta123/Franka_Manipulation/blob/main/libfranka-python/Classical_Control/demos/franka_inverse_kinematics.mp4" type="video/mp4">
</video>
</div>

- [x] Impedance Control    
  - Simulate compliant behaviors (e.g., pushing, insertion)
```
python impedance_control.py
```
<div style="text-align: left;">
<video style="width: 300; height: auto;" controls>
  <source src="https://github.com/PRISHIta123/Franka_Manipulation/blob/main/libfranka-python/Classical_Control/demos/franka_impedance_control.mp4" type="video/mp4">
</video>
</div>
     
### Learning-based Control  
In another terminal window, activate the same conda env and navigate to the libfranka-python subfolder:  
```
conda activate libfranka
cd Franka_Manipulation/libfranka-python/Learned_Control
```

- [x] Behavior Cloning (BC)  
  - Imitate trajectory from expert demonstration
```
cd "Behavior Cloning"
python collect_data.py
```
Will store the expert path in trajectory_data.pkl, and display expert path in the simulator.
```
python train_policy.py
```
Will save trained model in trajectory_model.pth
```
python run_policy.py
```
<div style="text-align: left;">
<video style="width: 600; height: auto;" controls>
  <source src="https://github.com/PRISHIta123/Franka_Manipulation/blob/main/libfranka-python/Learned_Control/Behavior Cloning/demos/BC.mp4" type="video/mp4">
</video>
</div>

- [x] Reinforcement Learning (RL)  
  - Algorithms: SAC / PPO  
  - Use `stable-baselines3` or custom RL code
```
cd "Reinforcement Learning"
python franka_env.py
```
Creates a state-action-reward environment within libfranka-sim for end effector goal  
```
python reinforcement_learning.py
```
PPO model is saved as ppo_franka_joint.zip    
<div style="text-align: left;">
<video style="width: 600; height: auto;" controls>
  <source src="https://github.com/PRISHIta123/Franka_Manipulation/blob/main/libfranka-python/Learned_Control/Reinforcement Learning/demos/PPO_RL.mp4" type="video/mp4">
</video>
</div>

- [x] Residual Policy Learning  
  - Combine scripted control with learned corrections
```
cd "Residual Policy Learning"  
python residual_policy_learning.py
```
Creates a state-action-reward environment within libfranka-sim for end effector goal.  
```
python train_residual_policy.py
```
PPO finetuning model over base oscillatory controller is saved as ppo_residual_franka.zip  

```
python deploy_residual_policy.py
```
<div>
<video style="width: 600; height: auto;" controls>
  <source src="https://github.com/PRISHIta123/Franka_Manipulation/blob/main/libfranka-python/Learned_Control/Residual Policy Learning/demos/RPL.mp4" type="video/mp4">
</video>
</div>
