[Click here and press view raw to download and view the Simscape simulation video.](A%20video.mp4) 

# Estimated Model-Based Gravity Controller for UR3 Arm


An end-to-end parameter estimation framework and model-based gravity compensation controller for the 6-DOF UR3 robotic manipulator. Built using Simulink, Simscape Multibody, and MATLAB.

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
