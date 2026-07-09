[Click here and press view raw to download and view a 170 seconds simulation video explaining the whole process.](A%20video.mp4) 

# Parameter-Estimation-For-Model-based-control-UR3-robot-arm



An end-to-end parameter estimation framework and model-based gravity compensation controller for the 6-DOF UR3 robotic manipulator. Built using Simulink, Simscape Multibody, and MATLAB.
## 📝 Project Overview

Conventional model-free controllers (like standalone PD loops) suffer from severe tracking degradation under dynamic loads. Unmodeled gravitational forces pull robotic link masses downward, causing permanent steady-state position sag. Reactive controllers require tracking errors to generate corrective torques, making high-precision paths impossible. 

Model-based control strategy (like Computed Torque Control) resolves this problem by canceling out the robot's physical weight and inertia equations on the fly. However, its tracking accuracy depends strictly on the baseline precision of the internal plant model. This project establishes an end-to-end parameter estimation pipeline to solve this modeling requirement through non-invasive system identification.

### Why Parameter Estimation is Essential in Robotics
* **Unavailability of Direct Measurements**: Real link masses and inertia tensors cannot be verified without dismantling the mechanical robot.
* **Inaccuracy of CAD Baselines**: Theoretical factory data ignores structural variations caused by age, wear, or retrofitted modifications.
* **Workspace Physical Realism**: Linear regression can return physically impossible variables like negative friction or non-positive-definite matrices.
* **Enforcing Structural Consistency**: Constrained non-linear optimization ensures that the estimated parameters map strictly to real-world physics.
  
  <img width="1023" height="560" alt="image" src="https://github.com/user-attachments/assets/dd2ce280-8b6c-4701-a67a-dc2245986965" />
  Figure 1 The inertial parameter excitation experiment by standard PD controller.
 <img width="1646" height="672" alt="image" src="https://github.com/user-attachments/assets/6ff16ebc-8ced-45f8-9986-92182359505c" />

  Figure 2 The UR3 Simscape model For digital twining .
  <img width="1132" height="598" alt="image" src="https://github.com/user-attachments/assets/125ae9f2-f7d2-488e-a311-13724edd71e5" />
  Figure 3 A link configuration with Viscous-Columb friction forces applied .
  <img width="746" height="422" alt="UR3Excitation_step2" src="https://github.com/user-attachments/assets/30a4910a-c255-41be-b362-d4a4c86f52d1" />




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
