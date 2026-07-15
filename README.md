[Click here and press view raw to download and view a 170 seconds simulation video explaining the whole process.](A%20video.mp4) 

# Parameter-Estimation-For-Accurate Model-based-control-Featuring-UR3-robot-arm



An end-to-end parameter estimation framework to design model-based gravity compensation controller for the 6-DOF UR3 robotic manipulator. Built using Simulink, Simscape Multibody, and MATLAB.
## 📝 Project Overview

Conventional model-free controllers (like standalone PD loops) suffer from severe tracking degradation under dynamic loads. Unmodeled gravitational forces pull robotic link masses downward, causing permanent steady-state position sag. Reactive controllers require tracking errors to generate corrective torques, making high-precision paths impossible. 

Model-based control strategy (like Computed Torque Control) resolves this problem by canceling out the robot's physical weight and inertia equations on the fly. However, its tracking accuracy depends strictly on the baseline precision of the internal plant model. This project establishes an end-to-end parameter estimation pipeline to solve this modeling requirement through non-invasive system identification.

### Why Parameter Estimation is Essential in Robotics
* **Unavailability of Direct Measurements**: Real link masses and inertia tensors cannot be verified without dismantling the mechanical robot.
* **Inaccuracy of CAD Baselines**: Theoretical factory data ignores structural variations caused by age, wear, or retrofitted modifications.
* **Workspace Physical Realism**: Linear regression can return physically impossible variables like negative friction or non-positive-definite matrices.
* **Enforcing Structural Consistency**: Constrained non-linear optimization ensures that the estimated parameters map strictly to real-world physics.
  
 --- ---------------------------------------------------------------
  
<img width="1389" height="731" alt="Screenshot 2026-07-15 192151" src="https://github.com/user-attachments/assets/4e89793a-2b7a-4268-a172-16afdc5b28a3" />


  
 Figure 1 The inertial parameter excitation experiment by standard PD controller.
  
  ---------------------------------------------------------------
  
 <img width="1678" height="722" alt="Screenshot 2026-07-15 192007" src="https://github.com/user-attachments/assets/974a2d9a-b01c-4243-ab75-504caa66119a" />

 
  Figure 2 The UR3 Simscape model For digital twining .
  
-----------------------------------------------------------------
  
  <img width="1180" height="550" alt="image" src="https://github.com/user-attachments/assets/dff34182-3e03-4cc3-8f43-013719417bbc" />
  
  Figure 3 A link configuration with Viscous-Columb friction forces applied .
  
  ---------------------------------------------------------------
  <img width="746" height="422" alt="UR3Excitation_step2" src="https://github.com/user-attachments/assets/30a4910a-c255-41be-b362-d4a4c86f52d1" />
  
  Figure 4 A visualzation of how we excite the inertial parameters of the UR3 robot arm.
  
  ---------------------------------------------------------------

  <img width="1917" height="922" alt="image" src="https://github.com/user-attachments/assets/68ea9611-1a74-43c0-9e01-e9d76e3ce1a7" />
  
   Figure 5 The Parameter estimation validation results.
   
-----------------------------------------------------------------

  <img width="1025" height="563" alt="image" src="https://github.com/user-attachments/assets/5f464711-4b4b-4b1c-b4de-283ce57d0c8e" />
  
  Figure 6 Testing the estimated gravity matrix at zero joint position .
  
-----------------------------------------------------------------

   <img width="1917" height="925" alt="image" src="https://github.com/user-attachments/assets/7260d6fc-3db2-4899-a9dd-a4e2f72c6df5" />
   
   Figure 7 The proactive applied torques from the estimated gravity matrix to cancel the gravity forces (see the shoulder joints (2 and 3) ).
   
-----------------------------------------------------------------


  <img width="746" height="422" alt="UR3G_Matrix_testing_step7" src="https://github.com/user-attachments/assets/5732ad18-c48c-4a11-a77b-4fb4cfc96f9f" />
  
  Figure 8 A Visualization to show that the robot arm stand still at zero joint positon commands.
  
-----------------------------------------------------------------
  <img width="1917" height="927" alt="image" src="https://github.com/user-attachments/assets/62b26d70-8496-4e95-bde9-818bcbabd181" />
  
  Figure 9 The reactive applied torques from PD controller.
  
-----------------------------------------------------------------
  <img width="746" height="422" alt="PDatzero" src="https://github.com/user-attachments/assets/3828b2f1-47f0-4cd3-9538-4174feca8dee" />
  
  Figure 10 A Visualzation of the robot arm behavior with standard standalone PD controller.
  
------------------------------------------------------------



  







## 🛠️ Pipeline & File Architecture

The repository is structured sequentially to take a robot from raw DH parameters to a verified gravity-compensating controller:

### 1. Kinematic & Symbolic Modeling
* **`GetTheRegressorMatrix_step1.m`**: Generates the symbolic regressor matrix and base parameter vector for the manipulator, utilizing custom code reverse-engineered from the BIRDy benchmark.

### 2. Trajectory Generation & Data Extraction
* **`UR3Excitation_step2.slx`**: Simscape excitation trajectory simulation designed to maximize parameter visibility and excite the robot's dynamic properties.
* **`UR3DataExtraction_step3.slx`**: Extracts the resulting joint kinematic states—Position, Velocity, and Acceleration (PVA)—from the Simscape environment.

### 3. Dynamics & Identification
* **`BaseParameterEstimation_step4.m`**: Runs the parameter identification algorithms matching the raw PVA data against the symbolic regressor model.
* **`ReformulatingTheEstimatedMatrices_step5.m`**: Reconstructs the symbolic dynamic matrices using the newly identified base parameters.
* **`GetTheEstimatedINVDynamics_step6.m`**: Formulates the estimated inverse dynamics used to feed the controller feedforward loops.

### 4. Verification & Control
* **`UR3G_Matrix_testing_step7.slx`**: Validates the accuracy of the isolated Gravity Matrix $G(q)$ at the zero-joint position state. 
* **`Gravity_compensationController_step8.slx`**: The final closed-loop gravity compensation controller operating inside Simscape.

### 5. CAD & Digital Twin Assets
* Contains the foundational URDF (`ur3.urdf`) along with the associated visual and collision geometry meshes (`.stl` and `.dae` files for the base, shoulder, upperarm, forearm, and wrist) to compile the Simscape Multibody environment.

---

## ⚠️ Hardware & Memory Constraints Note

Symbolically evaluating or validating a full joint-space mass matrix $M(q)$ across dynamic trajectories introduces severe RAM allocation bottlenecks in MATLAB. To bypass these hardware limitations while ensuring mathematical correctness, the validation phase was intentionally decoupled:
* Virtual validation is strictly isolated to the **Gravity Matrix $G(q)$ at the zero-joint position state**.
* This reduction successfully verifies the accuracy of the base parameter estimation pipeline without risking out-of-memory engine crashes.
