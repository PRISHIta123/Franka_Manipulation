# Franka_Manipulation
Manipulate a Franka Emika Robot Arm in Simulation using various algorithms  


### Classical Control  

- [x] Sinusoidal y-axis joint 4 motion  
- [x] Pick and place joints 0 and 4 motion  
- [x] Proportional-Derivative (PD) control  
  - Follow reference joint trajectories  
- [x] Inverse Kinematics (IK) / Operational Space Control    
  - Cartesian control of the end-effector  
- [x] Impedance Control    
  - Simulate compliant behaviors (e.g., pushing, insertion)
     
### Learning-based Control  

- [x] Behavior Cloning (BC)  
  - Imitate trajectory from expert demonstration
- [x] Reinforcement Learning (RL)  
  - Algorithms: SAC / PPO  
  - Use `stable-baselines3` or custom RL code
- [x] Residual Policy Learning  
  - Combine scripted control with learned corrections

