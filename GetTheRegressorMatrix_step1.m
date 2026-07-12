%This code is For the Regressor Model constuction you can choose If your model
%robot has friction in it you can choose which type of friction you need
%or if you need to exclude it in the main model i have Viscous columb
%friction in the robot
%%The Code is Reverse Engineered By Abdelrahman Elzemrany From BIRDy
%%Benchmark
robot = struct();
robot.name = 'Graduate';                                                    % Name of the robot
robot.nbDOF = 6;                                                        % Number of Degrees Of Freedom (DOF)
robot.rootFrame.name = 'world';                                         % Name of the world reference frame
robot.rootFrame.transform = sym(eye(4));                              	% Homogeneous transform between the world reference frame and the robot base frame
robot.rootFrame.transform(3,4) = 1;
robot.jointType = ['revol';'revol';'revol';'revol';'revol';'revol'];    % Type of the robot joints: either 'revol' or 'prism'                        
robot.frictionIdentModel = 'ViscousCoulomb';    % Go to line 216 and navigate which type of friction you need
%If you choose (no) go to line 1103 and remove a4 from the array and go to
%line 30 and read the note 
robot.symbolicParameters = generateRobotSymbolicParameters(robot.nbDOF, robot.frictionIdentModel);   
robot.symbolicParameters.Xhi = [robot.symbolicParameters.Xhi];
robot.symbolicParameters.Xhi_aug = [robot.symbolicParameters.Xhi_aug];
%%D-H convention (The Essential part to obtain the regressor model)
robot.dhParameter.theta = [robot.symbolicParameters.Q(1); robot.symbolicParameters.Q(2);robot.symbolicParameters.Q(3);robot.symbolicParameters.Q(4);robot.symbolicParameters.Q(5);robot.symbolicParameters.Q(6)];
robot.dhParameter.alpha = sym([0; pi/2; 0; 0; pi/2; -pi/2]);
robot.dhParameter.a     = sym([0; 0; -0.24365; -0.21325; 0; 0]);
robot.dhParameter.d     = sym([0.15190; 0; 0; 0.11235; 0.08535; 0.0819]);
robot.dhConvention = 'proximal';       
% CORRECTED NUMERICAL GEOMETRY MATRIX FOR NOMINAL UR3
robot.numericalParameters.Geometry = ...
   [0       0          0.15190   0;     ... % Joint 1
    0       0          0         pi/2;  ... % Joint 2
    0      -0.24365    0         0;     ... % Joint 3
    0      -0.21325    0.11235   0;     ... % Joint 4
    0       0          0.08535   pi/2;  ... % Joint 5
    0       0          0.0819   -pi/2;  ... % Joint 6
    ];
    %theta   a         d         alpha
robot.numericalParameters.Gravity = [0;0;-9.81];                         % Gravity vector in world frame [m/s²]
robot.numericalParameters.numParam=[12;12;12;12;12;12];                 %If you have 10 parameters per link(no friction case) change the number 12 to 10


robot.dhConvention = 'proximal';% Robot Denavit-Hartenerg convention: either 'proximal' or 'distal'
Q = robot.symbolicParameters.Q;
Qp = robot.symbolicParameters.Qp;
Qpp = robot.symbolicParameters.Qpp;
Geometry = robot.symbolicParameters.Geometry;
GeometryCOM = robot.symbolicParameters.GeometryCOM;
Moment = robot.symbolicParameters.Moment;
InertiaCOM = robot.symbolicParameters.InertiaCOM;
Ia = robot.symbolicParameters.Ia;
Mass = robot.symbolicParameters.Mass;
Gravity = robot.symbolicParameters.Gravity;
Xhi= robot.symbolicParameters.Xhi;
Friction = robot.symbolicParameters.friction;
Z = robot.symbolicParameters.Z;
dt = robot.symbolicParameters.dt;
robotName = robot.name;
 HT_dhi_world = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of DH frame i w.r.t world frame
    HT_cmi_world = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of Center of Mass i w.r.t world frame
    HT_cmi_dhi = sym(zeros(4,4,robot.nbDOF));     % Homogeneous Transformation of Center of Mass i w.r.t DH frame i
    HT_cmi_cmi_1 = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of Center of Mass i w.r.t Center of Mass i-1
    HT_cmi_world_Moment = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of Center of Mass i w.r.t world frame expressed wrt moment standard parameter (MX, MY, MZ)
    HT_cmi_dhi_Moment = sym(zeros(4,4,robot.nbDOF));     % Homogeneous Transformation of Center of Mass i w.r.t DH frame i expressed wrt moment standard parameter (MX, MY, MZ)
    HT_cmi_cmi_1_Moment = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of Center of Mass i w.r.t Center of Mass i-1 expressed wrt moment standard parameter (MX, MY, MZ)
    HT_dhi_dhi_1 = sym(zeros(4,4,robot.nbDOF));   % Homogeneous Transformation of DH frame i w.r.t DH frame i-1
    HT_base_world = robot.rootFrame.transform;    % Homogeneous Transformation of Robot base frame w.r.t world (static)
 for joint=1:robot.nbDOF
        HT_cmi_dhi(:,:,joint) = eye(4);
        HT_cmi_dhi(1:3,4,joint) =robot.symbolicParameters.GeometryCOM(1:3, joint);
        HT_cmi_dhi_Moment(:,:,joint) = eye(4);
        HT_cmi_dhi_Moment(1:3,4,joint) =robot.symbolicParameters. Moment(1:3, joint)./robot.symbolicParameters.Mass(joint);
        HT_dhi_world(:,:,joint) = HT_base_world*computeHomogeneousTransformation(joint, 0, robot);
        HT_cmi_cmi_1(:,:,joint) = computeHomogeneousTransformation(joint, joint-1, robot, true, HT_cmi_dhi(:,:,joint));
        HT_cmi_cmi_1_Moment(:,:,joint) = computeHomogeneousTransformation(joint, joint-1, robot, true, HT_cmi_dhi_Moment(:,:,joint));
        HT_dhi_dhi_1(:,:,joint) = computeHomogeneousTransformation(joint, joint-1, robot);
        HT_cmi_world(:,:,joint) = HT_base_world*computeHomogeneousTransformation(joint, 0, robot,true, HT_cmi_dhi(:,:,joint));
        HT_cmi_world_Moment(:,:,joint) = HT_base_world*computeHomogeneousTransformation(joint, 0, robot,true, HT_cmi_dhi_Moment(:,:,joint));
 end
    J_dhi_world = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    J_cmi_world = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    J_cmi_world_Moment = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    Jd_dhi_world = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    Jd_cmi_world = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    Jd_cmi_world_Moment = sym(zeros(6,robot.nbDOF,robot.nbDOF));
    for joint = 1:robot.nbDOF %#ok<PFUIX>
        
        % Link Jacobians:
        J_dhi_world(:,:,joint) = computeGeometricJacobian(robot, joint, HT_base_world, HT_dhi_world);
        fprintf('Jacobian link %d, successfully computed\n',joint);
        
        % Center of Mass Jacobians:
        J_cmi_world(:,:,joint) = computeGeometricJacobian(robot, joint, HT_base_world, HT_dhi_world, true, HT_cmi_world);
        J_cmi_world_Moment(:,:,joint) = computeGeometricJacobian(robot, joint, HT_base_world, HT_dhi_world, true, HT_cmi_world_Moment);
        fprintf('Jacobian Center of Mass %d, successfully computed\n',joint);
        
        % Time derivative of the Link Jacobians:
        Jd_dhi_world(:,:,joint) = timeDerivative(J_dhi_world(:,:,joint),robot.symbolicParameters.Q,robot.symbolicParameters.Qp);
        fprintf('Time derivative of Jacobian link %d, successfully computed\n',joint);
        
        % Time derivative of the Center of Mass Jacobians:
        Jd_cmi_world(:,:,joint) = timeDerivative(J_cmi_world(:,:,joint), robot.symbolicParameters.Q,robot.symbolicParameters. Qp);
        Jd_cmi_world_Moment(:,:,joint) = timeDerivative(J_cmi_world_Moment(:,:,joint),robot.symbolicParameters. Q, robot.symbolicParameters.Qp);
        fprintf('Time derivative of Jacobian Center of Mass %d, successfully computed\n',joint);
    end
     disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    disp('Symbolic Matrix form of the Euler-Lagrange Dynamics Equations...')
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
    
    options.algorithm = 'newton'; % 'newton' or 'lagrange'
    options.verif = false;
    [M_symb, C_symb, G_symb, kineticEnergy_symb, potentialEnergy_symb] = computeDynamicModel(robotName, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, robot.symbolicParameters.Mass, robot.symbolicParameters.InertiaCOM,robot.symbolicParameters.Ia, robot.symbolicParameters. Gravity  , robot.symbolicParameters.Q,robot.symbolicParameters. Qp, options);
    Tau_Friction_symb_aug = computeFrictionModel(robotName, Qp, Friction, Z, dt);
    Tau_symb = M_symb*Qpp + C_symb*Qp + G_symb; % With friction
     options.method = 'blockTriangular'; % 'oldMethod', 'blockTriangular' or 'baseParametersNum'
     %[Y_r,Xhi_r]=computeIdentificationModel(J_dhi_world, Jd_dhi_world, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, HT_dhi_dhi_1, robot, Tau_symb, Tau_Friction_symb_aug, options);
     
     [Y_b, Y_d, Beta, Xhi_b, Xhi_d, qr_P, K_d] =  computeIdentificationModel(J_dhi_world, Jd_dhi_world, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, HT_dhi_dhi_1, robot, Tau_symb, Tau_Friction_symb_aug, options);
     G=matlabFunction(Y_b,"file","Y_b_handle","optimize",false) ;
      




function [robot] = loadRobotModelParameters(robotName, options)

robot = feval(robotName, options);

if options.checkPhysicality == true
   
    robot.K_d = K_d;
    robot.Perm = Perm;
    if checkPhysicalConsistency(robot) == false
        error('The parameters of the simulated robot are not physically consistent !');
    else
        disp('The robot parameters seems to be physically consistent. Moving forward...');
    end
end
end

function [I] = inertiaTensorDH2COM(L, Mi, Moment)



for i=1:numel(Mi)
    I(:,:,i) = L(:,:,i) - Skew(Moment(:,i))'*Skew(Moment(:,i))./Mi(i);
end

end
function [L] = inertiaTensorCOM2DH(I, Mi, Moment)



for i=1:numel(Mi)
    L(:,:,i) = I(:,:,i) + Skew(Moment(:,i))'*Skew(Moment(:,i))./Mi(i);
end

end
function [I] = inertiaMatrixDH2COM(L, Mi, Moment)



I = L - Skew(Moment)'*Skew(Moment)./Mi;

end

function [L] = inertiaMatrixCOM2DH(I, Mi, Moment)

 
L = I + Skew(Moment)'*Skew(Moment)./Mi;

end
function [I] = inertiaMatrix(XXi, XYi, XZi, YYi, YZi, ZZi)



I = [XXi, XYi, XZi; ...
    XYi, YYi, YZi; ...
    XZi, YZi, ZZi];
end
function [Xhi, numParam, Xhi_aug] = getStandardParameterVector(InertiaDH, Moment, Mass, Ia, frictionModel, frictionParameters, varargin)




% Check dimentionality

if (size(InertiaDH,3) == size(Moment,2)) && (size(InertiaDH,3) == length(Mass)) && (size(InertiaDH,3) == length(Ia)) 
    nbDOF = size(InertiaDH,3);
    numParam = zeros(nbDOF,1);
else
    error('getStandardParameterVector(): Error in parameter dimentionality !');
end

for i=1:nbDOF
    XXi = InertiaDH(1,1,i); 
    XYi = InertiaDH(1,2,i); 
    XZi = InertiaDH(1,3,i); 
    YYi = InertiaDH(2,2,i);
    YZi = InertiaDH(2,3,i);
    ZZi = InertiaDH(3,3,i);
    Mi = Mass(i);
    MXi = Moment(1,i);
    MYi = Moment(2,i);
    MZi = Moment(3,i);
    switch frictionModel
        % Only Coulomb and viscous frictions are linear and can be identified simultaneously with the other parameters.
        % Nonlinear friction models require state dependant parameter identification
        case 'no'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi ];
        case 'Viscous'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; frictionParameters.Fv(i); frictionParameters.Tau_off(i)];
        case 'Coulomb'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; frictionParameters.Fc(i); frictionParameters.Tau_off(i)];
        case 'ViscousCoulomb'
            Xhi_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi;  frictionParameters.Fv(i); frictionParameters.Fc(i)];
       
    end
    numParam(i) = numel(Xhi_block);
    Xhi((i-1)*numel(Xhi_block)+1:i*numel(Xhi_block),1)=Xhi_block;
    % Total Dynamic parameters vector:
    Xhi_tot_block = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; Ia(i); frictionParameters.Fv(i); frictionParameters.Fc(i); frictionParameters.Fs(i); frictionParameters.Vs(i); frictionParameters.Es(i); frictionParameters.Sigma_0(i); frictionParameters.Sigma_1(i); frictionParameters.Sigma_2(i); frictionParameters.Z0(i)]; % Only Coulomb and viscous frictions are linear and can be identified simultaneously with the other parameters...
    Xhi_aug((i-1)*numel(Xhi_tot_block)+1:i*numel(Xhi_tot_block),1) = Xhi_tot_block;
end
end

function  symbolicParameters = generateRobotSymbolicParameters(nbDOF, frictionIdentModel)



% Allocating memory (Joint variables):
symbolicParameters.Q = sym(zeros(nbDOF,1));                                     % Joint position [rad]
symbolicParameters.Qp = sym(zeros(nbDOF,1));                                    % Joint velocity [rad/s]
symbolicParameters.Qpp = sym(zeros(nbDOF,1));                                   % Joint acceleration [rad/s^2]
symbolicParameters.Tau = sym(zeros(nbDOF,1));                                   % Joint torque [N.m]

% Allocating memory (Kinematic and Dynamic variables):
symbolicParameters.Geometry = sym(zeros(nbDOF,4));                              % Position of the robot DH links [m]
symbolicParameters.GeometryCOM = sym(zeros(3,nbDOF));                           % Position of the links' center of mass w.r.t the link's DH frame[m]
symbolicParameters.Moment = sym(zeros(3,nbDOF));                                % First Moment
symbolicParameters.Mass = sym(zeros(nbDOF,1));                                  % Mass of the robot links [kg]
symbolicParameters.InertiaDH = sym(zeros(3,3,nbDOF));                           % Inertia tensor around dh frame
symbolicParameters.InertiaCOM = sym(zeros(3,3,nbDOF));                          % Inertia tensor around COM
symbolicParameters.Ia = sym(zeros(nbDOF,1));                                    % Actuator and transmission inertias



% Allocating memory (Friction parameters):
symbolicParameters.Z = sym(zeros(nbDOF,1));                                     % LuGre State
symbolicParameters.friction.Fv = sym(zeros(nbDOF,1));                        	% Viscous Friction
symbolicParameters.friction.Fc = sym(zeros(nbDOF,1));                           % Coulomb Friction
symbolicParameters.friction.Fs = sym(zeros(nbDOF,1));                         	% Static Friction: only in Stribeck and LuGre models
symbolicParameters.friction.Vs = sym(zeros(nbDOF,1));                         	% Stribeck velocity: only in Stribeck and LuGre models
symbolicParameters.friction.Es = sym(zeros(nbDOF,1));                        	% Exponent: only in Stribeck and LuGre models
symbolicParameters.friction.Sigma_0 = sym(zeros(nbDOF,1));                      % Contact stiffness: only in LuGre model
symbolicParameters.friction.Sigma_1 = sym(zeros(nbDOF,1));                      % Damping coefficient of the bristle: only in LuGre model
symbolicParameters.friction.Sigma_2 = sym(zeros(nbDOF,1));                      % Viscous friction coefficient of the bristle: only in LuGre model
symbolicParameters.friction.Z0 = sym(zeros(nbDOF,1));                           % Initial deflection of the contacting asperities: only in LuGre model
symbolicParameters.friction.Tau_off = sym(zeros(nbDOF,1));                     	% Resulting nonlinear friction torque: only in Stribeck and LuGre models
symbolicParameters.friction.Fvm = sym(zeros(nbDOF,1));                        	% Coupling Viscous Friction
symbolicParameters.friction.Fcm = sym(zeros(nbDOF,1));                        	% Coupling Coulomb Friction

% Gravity:
gx = sym('gx','real');
gy = sym('gy','real');
gz = sym('gz','real');
symbolicParameters.Gravity = [gx;gy;gz];                                        % Gravity vector in world frame

% Control Period
symbolicParameters.dt = sym('dt','real');



for i=1:nbDOF
    % Joint variables:
    symbolicParameters.Q(i,1) = sym(sprintf('q%d',i),'real');
    symbolicParameters.Qp(i,1) = sym(sprintf('qp%d',i),'real');
    symbolicParameters.Qpp(i,1) = sym(sprintf('qpp%d',i),'real');
    symbolicParameters.Tau(i,1) = sym(sprintf('tau%d',i),'real');
    
    % Robot link geometry [Beta d a alpha]:
    symbolicParameters.Geometry(i,1) = sym(sprintf('theta%d',i),'real');
    symbolicParameters.Geometry(i,2) = sym(sprintf('d%d',i),'real');
    symbolicParameters.Geometry(i,3) = sym(sprintf('a%d',i),'real');
    symbolicParameters.Geometry(i,4) = sym(sprintf('alpha%d',i),'real');
    
    % Robot link center of mass geometry:
    symbolicParameters.GeometryCOM(1,i) = sym(sprintf('X%d',i),'real');
    symbolicParameters.GeometryCOM(2,i) = sym(sprintf('Y%d',i),'real');
    symbolicParameters.GeometryCOM(3,i) = sym(sprintf('Z%d',i),'real');
    
    % Actuator and transmission inertias:
    symbolicParameters.Ia(i) = sym(sprintf('Ia%d',i),'real');
    
    % Robot link masses:
    symbolicParameters.Mass(i) = sym(sprintf('M%d',i),'real');
    
    % Robot link first moment:
    symbolicParameters.Moment(1,i) = sym(sprintf('MX%d',i),'real');
    symbolicParameters.Moment(2,i) = sym(sprintf('MY%d',i),'real');
    symbolicParameters.Moment(3,i) = sym(sprintf('MZ%d',i),'real');
    
    % Robot link inertias (around the link DH frame):
    XXi = sym(sprintf('XX%d',i),'real');
    XYi = sym(sprintf('XY%d',i),'real');
    XZi = sym(sprintf('XZ%d',i),'real');
    YYi = sym(sprintf('YY%d',i),'real');
    YZi = sym(sprintf('YZ%d',i),'real');
    ZZi = sym(sprintf('ZZ%d',i),'real');
    symbolicParameters.InertiaDH(:,:,i) = inertiaMatrix(XXi, XYi, XZi, YYi, YZi, ZZi);
    symbolicParameters.InertiaCOM(:,:,i) = inertiaMatrixDH2COM(symbolicParameters.InertiaDH(:,:,i), symbolicParameters.Mass(i), symbolicParameters.Moment(:,i));
    
    % Symbolic Friction parameters:
    symbolicParameters.Z(i,1) = sym(sprintf('Z%d',i),'real');
    symbolicParameters.friction.Fv(i,1) = sym(sprintf('Fv%d',i),'real');                % Viscous Friction
    symbolicParameters.friction.Fc(i,1) = sym(sprintf('Fc%d',i),'real');            	% Coulomb Friction
    symbolicParameters.friction.Fs(i,1) = sym(sprintf('Fs%d',i),'real');             	% Static Friction: only in Stribeck and LuGre models
    symbolicParameters.friction.Vs(i,1) = sym(sprintf('Vs%d',i),'real');            	% Stribeck velocity: only in Stribeck and LuGre models
    symbolicParameters.friction.Es(i,1) = sym(sprintf('Es%d',i),'real');                % Exponent: only in Stribeck and LuGre models
    symbolicParameters.friction.Sigma_0(i,1) = sym(sprintf('Sigma_0%d',i),'real');  	% Contact stiffness: only in LuGre model
    symbolicParameters.friction.Sigma_1(i,1) = sym(sprintf('Sigma_1%d',i),'real');  	% Damping coefficient of the bristle: only in LuGre model
    symbolicParameters.friction.Sigma_2(i,1) = sym(sprintf('Sigma_2%d',i),'real');   	% Viscous friction coefficient of the bristle: only in LuGre model
    symbolicParameters.friction.Z0(i,1) = sym(sprintf('Z0%d',i),'real');             	% Initial deflection of the contacting asperities: only in LuGre model
    symbolicParameters.friction.Tau_off(i,1) = sym(sprintf('tau_off%d',i),'real');      % Resulting nonlinear friction torque: only in Stribeck and LuGre models
    symbolicParameters.friction.Fvm(i,1) = sym(sprintf('Fvm%d',i),'real');              % Couping Viscous friction parameters as described in Gautier et al. 2011.
    symbolicParameters.friction.Fcm(i,1) = sym(sprintf('Fcm%d',i),'real');              % Couping Coulomb friction parameters as described in Gautier et al. 2011.
end

% Generate the vector Xhi of standard parameters:
[symbolicParameters.Xhi, ~, symbolicParameters.Xhi_aug] = getStandardParameterVector(symbolicParameters.InertiaDH, symbolicParameters.Moment, symbolicParameters.Mass, symbolicParameters.Ia, frictionIdentModel, symbolicParameters.friction);

end

function [status] = checkPhysicalConsistency(robot)



status = true;

disp('Checking physical consistency of the robot parameters...');

for i = 1: robot.nbDOF
    
    % Check positivity of the friction terms:
    if robot.numericalParameters.friction.Fc(i) < 0 
        status = false;
        error('Fc(%d) < 0 !\n',i);
    end
    if robot.numericalParameters.friction.Fv(i) < 0
        status = false;
        error('Fv(%d) < 0 !\n',i);
    end
    
    % Check positivity of the transmission chain inertia term Ia:
    if robot.numericalParameters.Ia(i) < 0
        status = false;
        error('Ia(%d,%d) < 0 !\n',i,i);
    end
    
    % Check positive-definiteness of the inertia matrix I:
    I = robot.numericalParameters.InertiaCOM(:,:,i);
    
    [~,FLAG] = chol(I);
    if FLAG ~= 0
        status = false;
        I
        error('Inertia matrix of link %d wrt COM is NOT positive-definite !\n',i);
    end
     % Check positive-definiteness of the pseudo-inertia matrix P:
    P = pseudoInertiaMatrix(robot.numericalParameters.InertiaDH(:,:,i), robot.numericalParameters.Mass(i), robot.numericalParameters.Mass(i)*robot.numericalParameters.GeometryCOM(:,i), robot.numericalParameters.Ia(i), robot.numericalParameters.friction.Fv(i), robot.numericalParameters.friction.Fc(i));
    
    [~,FLAG] = chol(P);
    if FLAG ~= 0
        status = false;
        P
        error('Pseudo inertia matrix of link %d is NOT positive-definite !\n',i);
    end
    
end

end


function [Xhi, InertiaDH, InertiaCOM, numberParam] = aggregateNumericalParameters(nbDOF, frictionIdentModel, numericalParameters, linkRadius, linkLength, eulerAngles)



if nargin < 4 % Inertias are provided manually
    cylinderInertia = false;
else
    cylinderInertia = true;
end
InertiaDH = zeros(3,3,nbDOF);
InertiaCOM = zeros(3,3,nbDOF);

for i=1:nbDOF
    if cylinderInertia == true % Inertias are not provided manually and have to be computed considering a cylindrical link geometry:
        R = RotZ(eulerAngles(1,i))*RotY(eulerAngles(2,i))*RotX(eulerAngles(3,i));
        I = computeZAxisCylinderInertiaMatrix(linkRadius(i), linkLength(i), numericalParameters.Mass(i)); % Inertia of a cylinder of axis Z around DH frame
        InertiaDH(:,:,i) = R*I*R';
        InertiaCOM(:,:,i) = inertiaMatrixDH2COM(InertiaDH(:,:,i), numericalParameters.Mass(i), numericalParameters.Moment(:,i));   % Transform the inertia matrix to match the orientation of the cylinder
    else
        if numericalParameters.InertiaDH(1,:,i) == zeros(3)
            error('The inertia matrix of your robot cannot be zero !');
        end
        InertiaDH(:,:,i) = numericalParameters.InertiaDH(:,:,i); % Around DH frames
        InertiaCOM(:,:,i) = inertiaMatrixDH2COM(InertiaDH(:,:,i), numericalParameters.Mass(i), numericalParameters.Moment(:,i)); % Around COM frames
    end
end

% Generate the vector Xhi of standard parameters:
[Xhi, numberParam, ~] = getStandardParameterVector(numericalParameters.InertiaDH, numericalParameters.Moment, numericalParameters.Mass, numericalParameters.Ia, frictionIdentModel, numericalParameters.friction);

end

function [XXi, XYi, XZi, YYi, YZi, ZZi] = computeZAxisCylinderInertia(radius, length, mass)



XXi = 0.0833333 * mass * (3 * radius * radius + length * length);
XYi = 0;
XZi = 0;
YYi = 0.0833333 * mass * (3 * radius * radius + length * length);
YZi = 0;
ZZi = 0.5 * mass * radius * radius;
end

function [I] = computeZAxisCylinderInertiaMatrix(radius, length, mass)



XXi = 0.0833333 * mass * (3 * radius * radius + length * length);
XYi = 0;
XZi = 0;
YYi = 0.0833333 * mass * (3 * radius * radius + length * length);
YZi = 0;
ZZi = 0.5 * mass * radius * radius;
I = [XXi XYi XZi; ...
    XYi YYi YZi; ...
    XZi YZi ZZi];
end
function [P_i] = pseudoInertiaMatrix(Li, Mi, li, Iai, Fvi, Fci)



P_i = [[(trace(Li)/2)*eye(3)-Li, li; li', Mi], zeros(4,3);...
    zeros(3,4), diag([Iai; Fvi; Fci])];

end
function [P_i] = pseudoInertiaMatrix_Xhi(Xhi_i)


% Xhi_i = [XXi; XYi; XZi; YYi; YZi; ZZi; MXi; MYi; MZi; Mi; numericalParameters.Ia(i); numericalParameters.friction.Fv(i); numericalParameters.friction.Fc(i)];

Li = inertiaMatrix(Xhi_i(1), Xhi_i(2), Xhi_i(3), Xhi_i(4), Xhi_i(5), Xhi_i(6));
li = Xhi_i(7:9);
Mi = Xhi_i(10);
Iai = Xhi_i(11);
Fvi = Xhi_i(12);
Fci = Xhi_i(13);

P_i = [[(trace(Li)/2)*eye(3)-Li, li; li', Mi], zeros(4,3);...
    zeros(3,4), diag([Iai; Fvi; Fci])];

end
function yd = diffcent(y,pas)


ny=length(y);
yd=[(y(2)-y(1)),(y(3:ny)-y(1:ny-2))/2,(y(ny)-y(ny-1))]/pas;
end
function [upt] = discreteTimeDerivative(ut, ut_1, dt)
% Compute the discrete-time derivative
upt  = (ut - ut_1)/dt;
end

function [iHT] = invHT(HT)



iHT=HT;
iHT(1:3,1:3)=HT(1:3,1:3)';
iHT(1:3,4)=-HT(1:3,1:3)'*HT(1:3,4);
end

function [Tau] = RecursiveNewtonEuler(Q,Qp,Qpp,robot,Xhi)



for i=1:nbDOF
    J = feval(sprintf('J_dh%d_world_%s', i,robot.name),Q,robot.numericalParameters.Geometry);
    Jp = feval(sprintf('Jd_dh%d_world_%s', i,robot.name),Q,Qp,robot.numericalParameters.Geometry);
    Xp(:,i) = J*Qp;
    Xpp(:,i) = Jp*Qp + J*Qpp;
end

% Backward recursion:

for i=nbDOF:2
    Li = inertiaMatrix(Xhi_i(1), Xhi_i(2), Xhi_i(3), Xhi_i(4), Xhi_i(5), Xhi_i(6));
    li = Xhi_i(7:9);
    Mi = Xhi_i(10);
    
    pseudoInertia = [Mi*eye(3), -Skew(li);...
                Skew(li), Li];
    
    H_dhi_dhi_1 = invHT(feval(sprintf('HT_dh%d_dh%d_%s', i,i-1,robot.name),Q, robot.numericalParameters.Geometry)); % Homogeneous transform of frame i-1 wrt frame i
    R = H_dhi_dhi_1(1:3,1:3);
    r = H_dhi_dhi_1(1:3,4);
    T = [R, -R*Skew(r); zeros(3), R];
    W(:,i-1) = pseudoInertia*Xpp(:,i) + [Skew(Xp(4:6,i))*(Skew(Xp(4:6,i))*li); Skew(Xp(4:6,i))*(Li*Xp(4:6,i))];
    Tau(i) = J'*W(:,i-1);
end
end
function [ Rx ] = RotX(alphaX, degrad)



if nargin > 1 && strcmp(degrad, 'deg')
    alphaX = alphaX *pi/180;
end

ca=cos(alphaX);
sa=sin(alphaX);

Rx=[1 0   0;
    0 ca -sa;
    0 sa  ca];

end
function [ Ry ] = RotY(betaY, degrad)



if nargin > 1 && strcmp(degrad, 'deg')
    betaY = betaY *pi/180;
end

cb=cos(betaY);
sb=sin(betaY);

Ry=[cb  0 sb;
    0   1 0 ;
    -sb 0 cb];

end
function [ Rz ] = RotZ(gammaZ, degrad)



if nargin > 1 && strcmp(degrad, 'deg')
    gammaZ = gammaZ *pi/180;
end

cg=cos(gammaZ);
sg=sin(gammaZ);

Rz=[cg -sg 0;
    sg  cg 0;
    0   0  1];

end

function [S] = Skew(v)



S = [0,-v(3), v(2);
    v(3),0,-v(1);
    -v(2), v(1), 0];
end
function [ A_dot ] = timeDerivative(A, Var, Var_diff)



[m,n] = size(A);

A_dot = sym(zeros(size(A)));

for i = 1:m
    for j = 1:n
	tmp = sym(0);
        for k = 1:n
            A_dot(i,j) = A_dot(i,j) + diff(A(i,j),Var(k))*Var_diff(k);
        end
    end
end

A_dot = combine(A_dot);

end
function HT = computeHomogeneousTransformation(highIndex, lowIndex, robotModelParameters, compute_COM_Transform, HT_cmi_dhi, varargin)


if nargin < 3 || isempty(robotModelParameters.dhConvention)
    robotModelParameters.dhConvention = 'distal'; % Use distal convention by default
end

if nargin < 4 || isempty(compute_COM_Transform)
    compute_COM_Transform = false; % Compute link transform by default
end


HT = sym(eye(4));


for k = lowIndex+1:1:highIndex
    TransX = [1 0 0 robotModelParameters.dhParameter.a(k);...
        0 1 0 0;...
        0 0 1 0;...
        0 0 0 1];
    
    TransZ = [1 0 0 0;...
        0 1 0 0;...
        0 0 1 robotModelParameters.dhParameter.d(k);...
        0 0 0 1];
    
    rotX = [1 0 0 0;...
        0 cos(robotModelParameters.dhParameter.alpha(k)) -sin(robotModelParameters.dhParameter.alpha(k)) 0;...
        0 sin(robotModelParameters.dhParameter.alpha(k)) cos(robotModelParameters.dhParameter.alpha(k)) 0;...
        0 0 0 1];
    
    rotZ = [cos(robotModelParameters.dhParameter.theta(k)) -sin(robotModelParameters.dhParameter.theta(k)) 0 0;...
        sin(robotModelParameters.dhParameter.theta(k)) cos(robotModelParameters.dhParameter.theta(k)) 0 0;...
        0 0 1 0;...
        0 0 0 1];
    
    switch (robotModelParameters.dhConvention)
        case 'distal'
            % Distal convention:
            HT = HT*(rotZ*TransZ*TransX*rotX);
        case 'proximal'
            % Proximal convention:
            HT = HT*(rotX*TransX*rotZ*TransZ);
        otherwise
            error('Unknown DH convention !');
    end
    if compute_COM_Transform == true &&  k == highIndex
        HT = HT*HT_cmi_dhi;
    end
end

 HT = combine(simplify(HT));


end
function J = computeGeometricJacobian(robot, jointId, HT_base_world, HT_dhi_world, compute_COM_Jacobian, HT_cmi_world, varargin)



if nargin < 5 || isempty(compute_COM_Jacobian) || isempty(HT_cmi_world)
    compute_COM_Jacobian = false; % Compute the link jacobian
end

J= sym(zeros(6, robot.nbDOF));

switch robot.dhConvention
    case 'distal'
        for id = 1:jointId
            if id == 1
                Z_im1 = HT_base_world(1:3,1:3)*[0;0;1];
                if  compute_COM_Jacobian == true % Jacobian of the center of mass or of the skin cell
                    P = HT_cmi_world(1:3,4,jointId)-HT_base_world(1:3,4);
                else
                    P = HT_dhi_world(1:3,4,jointId)-HT_base_world(1:3,4);
                end
            else
                Z_im1 = HT_dhi_world(1:3,1:3,id-1)*[0;0;1];
                if  compute_COM_Jacobian == true % Jacobian of the center of mass or of the skin cell
                    P = HT_cmi_world(1:3,4,jointId)-HT_dhi_world(1:3,4,id-1);
                else
                    P = HT_dhi_world(1:3,4,jointId)-HT_dhi_world(1:3,4,id-1);
                end
            end
            
            if(strcmp(robot.jointType(id,:), 'revol')) % Revolute joint
                J(1:3,id) = cross(Z_im1,P);
                J(4:6,id) = Z_im1;
            elseif(strcmp(robot.jointType(id,:), 'prism')) % Prismatic joint
                J(1:3,id) = Z_im1;
                J(4:6,id) = [0;0;0];
            else
                error('Error in the symbolic expression of the robot Jacobian matrix ! There must be exactly one joint parameter per row of the DH table !')
            end
        end
    case 'proximal'
        for id = 1:jointId
            Z_im1 = HT_dhi_world(1:3,1:3,id)*[0;0;1];
            if  compute_COM_Jacobian == true % Jacobian of the center of mass or of the skin cell
                P = HT_cmi_world(1:3,4,jointId)-HT_dhi_world(1:3,4,id);
            else
                P = HT_dhi_world(1:3,4,jointId)-HT_dhi_world(1:3,4,id);
            end
            
            if(strcmp(robot.jointType(id,:), 'revol')) % Revolute joint
                J(1:3,id) = cross(Z_im1,P);
                J(4:6,id) = Z_im1;
            elseif(strcmp(robot.jointType(id,:), 'prism')) % Prismatic joint
                J(1:3,id) = Z_im1;
                J(4:6,id) = [0;0;0];
            else
                error('Error in the symbolic expression of the robot Jacobian matrix ! There must be exactly one joint parameter per row of the DH table !')
            end
        end
    otherwise
        error('Unknown DH convention !')
end

 J = combine(simplify(J));
end
function [M, C, G, kineticEnergy, potentialEnergy] = computeDynamicModel( robotName,J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, Mass, InertiaCOM, Ia, Gravity, Q, Qp, options, varargin)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

if nargin < 8 || isempty(options)
    options.algorithm = 'newton';
    options.verif = false;
end


[~,~,nbDOF] = size(HT_cmi_world_Moment);

%% Robot Dynamics:

% Potential Energy:
P = sym(zeros(nbDOF,1));
for i=1:nbDOF
    P(i) = Mass(i)*Gravity'*HT_cmi_world_Moment(1:3,4,i);
end
potentialEnergy = sum(P);

switch (options.algorithm)
    
    case 'lagrange'
        % Compute the symbolic equation for the inertia tensor:
        
        fprintf('Computing the inertia tensor M... \n');
        M = sym(zeros(nbDOF));
        for i = 1:nbDOF
            M = M + Mass(i)*J_cmi_world_Moment(1:3,:,i)'*J_cmi_world_Moment(1:3,:,i) + J_cmi_world_Moment(4:6,:,i)'*HT_cmi_world_Moment(1:3,1:3,i)*InertiaCOM(:,:,i)*HT_cmi_world_Moment(1:3,1:3,i)'*J_cmi_world_Moment(4:6,:,i);
        end
        M = M ; % Adding actuators inertia
        
        fprintf('Simplifying M... \n');
        M = simplify(M,'Seconds',30);
        
        % Compute the symbolic equation for the centripetal and Coriolis matrix using Christoffel symbol:
        
        fprintf('Computing the centripetal and Coriolis matrix C using Christoffel symbol... \n');
        C = sym(zeros(nbDOF));
        for k = 1:nbDOF
            for j = 1:nbDOF
                for i = 1:nbDOF
                    C(k,j) = C(k,j) + (1/2)*(diff(M(k,j),Q(i)) + diff(M(k,i),Q(j)) - diff(M(i,j),Q(k)))*Qp(i);
                end
            end
        end
        fprintf('Simplifying C... \n');
        C = simplify(C,'Seconds',30);
        
        % Compute the symbolic equation for the gravitational torques vector:
        
        fprintf('Computing the gravitational torques vector G... \n');
        G = sym(zeros(nbDOF,1));
        for i = 1:nbDOF
            G(i) = diff(potentialEnergy,Q(i));    % Partial derrivation of PT wrt q_i
        end
        fprintf('Simplifying G... \n');
        G = simplify(G,'Seconds',30);
        
    case 'newton'
        fprintf('Computing the inertia tensor M, Coriolis matrix C and gravitational torques vector G... \n');
        % Compute the symbolic equation for the inertia tensor, centripetal/Coriolis matrix and gravity torque vector at the same time:
        M_i = sym(zeros(nbDOF,nbDOF,nbDOF));
        C_i = sym(zeros(nbDOF,nbDOF,nbDOF));
        G_i = sym(zeros(nbDOF,nbDOF));
        
        for i = 1:nbDOF
            M_i(:,:,i) = Mass(i)*J_cmi_world_Moment(1:3,:,i)'*J_cmi_world_Moment(1:3,:,i) + J_cmi_world_Moment(4:6,:,i)'*HT_cmi_world_Moment(1:3,1:3,i)*InertiaCOM(:,:,i)*HT_cmi_world_Moment(1:3,1:3,i)'*J_cmi_world_Moment(4:6,:,i);
            C_i(:,:,i) = Mass(i)*J_cmi_world_Moment(1:3,:,i)'*Jd_cmi_world_Moment(1:3,:,i) + J_cmi_world_Moment(4:6,:,i)'*HT_cmi_world_Moment(1:3,1:3,i)*InertiaCOM(:,:,i)*HT_cmi_world_Moment(1:3,1:3,i)'*Jd_cmi_world_Moment(4:6,:,i) +  J_cmi_world_Moment(4:6,:,i)'*Skew(J_cmi_world_Moment(4:6,:,i)*Qp)*HT_cmi_world_Moment(1:3,1:3,i)*InertiaCOM(:,:,i)*HT_cmi_world_Moment(1:3,1:3,i)'*J_cmi_world_Moment(4:6,:,i);
            G_i(:,i) = Mass(i)*J_cmi_world_Moment(1:3,:,i)'*Gravity;
        end
        if nbDOF>1
            fprintf('Simplifying M... \n');
            %         M = simplify(sumnd(M_i,3) + diag(Ia),'Seconds',30);
            M = sumnd(M_i,3) + diag(Ia); % Adding actuators inertia;
            
            fprintf('Simplifying C... \n');
            %         C = simplify(sumnd(C_i,3),'Seconds',30);
            C = sumnd(C_i,3);
            
            fprintf('Simplifying G... \n');
            %         G = simplify(sumnd(G_i,3),'Seconds',30);
            G = sumnd(G_i,2);
        else
            M = M_i + diag(Ia); % Adding actuators inertia;
            C = C_i;
            G = G_i;
        end
        
    otherwise
        error('Robot Dynamic Computation: Unknown Algorithm');
end



% Kinetic Energy:
kineticEnergy = (1/2)*Qp'*M*Qp;

%% Verificaton Routines:

if options.verif == true
    % Verification of dynamic properties:
    fprintf('Verification of Dynamic properties...\n');
    error_M = M'-M;
    error_M = simplify(error_M,'Seconds',30);
    if error_M == zeros(nbDOF,nbDOF)
        fprintf('First dynamic property verified: M is symmetric.\n');
    else
        error('First dynamic property violated: M is NOT symmetric.\n');
    end
    
    M_dot = timeDerivative(M, Q, Qp);
    N = M_dot - 2*C;
    N = simplify(N,'Seconds',30);
    error_N = N'+N;
    error_N = simplify(error_N,'Seconds',30);
    
    if error_N == zeros(nbDOF,nbDOF)
        fprintf('Second dynamic property verified: M_dot - 2*C is skew symmetric.\n');
    else
        error('Second dynamic property violated: M_dot - 2*C is NOT skew symmetric.\n');
    end
end
end


function M = sumnd(M,dim)
s=size(M);
M=permute(M,[setdiff(1:ndims(M),dim),dim]);
M=reshape(M,[],s(dim));
M=sum(M,2);
s(dim)=1;
M=reshape(M,s);
end
function [frictionForce] = computeFrictionModel(robotName, Qp, frictionParameters, Z_m1, dt)



frictionForce = sym(zeros(numel(Qp), 8));

% No friction:
frictionForce(:,1) = 0*Qp ;
% Viscous friction:
frictionForce(:,2) = diag(frictionParameters.Fv)*Qp ;
% Coulomb friction:
frictionForce(:,3) = diag(frictionParameters.Fc)*tanh(100*Qp) ;
% Intagrated Viscous and Coulomb friction:
frictionForce(:,4) = diag(frictionParameters.Fv)*Qp + diag(frictionParameters.Fc)*tanh(100*Qp);



end

function [Y_b, Y_d, Beta, Xhi_b, Xhi_d, qr_P, Kd] = computeIdentificationModel(J_dhi_world, Jd_dhi_world, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, HT_dhi_dhi_1, robot, Tau, Tau_friction_aug, options)

% Authors: Quentin Leboutet, Julien Roux, Alexandre Janot and Gordon Cheng

if nargin<11 || (isequal(robot.dhConvention,'distal') && strcmp(options.method, 'baseParameters'))
    options.method = 'baseParametersNum';
end


switch robot.frictionIdentModel
    % Only Coulomb and viscous frictions are linear and can be identified simultaneously with the other parameters.
    % Nonlinear friction models require state dependant parameter identification
    case 'no'
        Tau = Tau + Tau_friction_aug(:,1);
    case 'Viscous'
        Tau = Tau + Tau_friction_aug(:,2);
    case 'Coulomb'
        Tau = Tau + Tau_friction_aug(:,3);
    case 'ViscousCoulomb'
        Tau = Tau + Tau_friction_aug(:,4);
    
end

% robot.symbolicParameters.Gravity = [sym(zeros(2,1));robot.symbolicParameters.Gravity(3)]; % ASSUMES THAT THE BASE Z AXIS OF THE ROBOT IS COLINEAR WITH GRAVITY. OTHERWISE REMOVE !

fprintf('Computing Identification Model using the Algorithm: %s\n', options.method)

[Y_r, Xhi_r] = blockTriangular(J_dhi_world, Jd_dhi_world, J_cmi_world_Moment, Jd_cmi_world_Moment, HT_cmi_world_Moment, robot);
[Y_b, Y_d, Beta, Xhi_b, Xhi_d, qr_P, Kd] = baseParametersNumericalSimplification(robot, Y_r, Xhi_r, 1e3);
end

%% Compute the regressor in a block upper-triangular form 

function [Y, Xhi] = blockTriangular(J_dhi_world, Jd_dhi_world, J_cmi_world, Jd_cmi_world, HT_cmi_world, robot)

% Generate the parameter vector defined in [jung et al. 2018]:
Xhi = robot.symbolicParameters.Xhi;
nbparam = robot.numericalParameters.numParam;

% Compute the Block-Upper-Triangular Model Regressor:

counter = 0;
Y = sym(zeros(robot.nbDOF,numel(Xhi)));
for i = 1:robot.nbDOF
    Ri = HT_cmi_world(1:3,1:3,i);
    W_cmi_world = J_cmi_world(4:6,:,i)*robot.symbolicParameters.Qp;
    Wp_cmi_world = Jd_cmi_world(4:6,:,i)*robot.symbolicParameters.Qp + J_cmi_world(4:6,:,i)*robot.symbolicParameters.Qpp;
    Vp_dhi_world = Jd_dhi_world(1:3,:,i)*robot.symbolicParameters.Qp + J_dhi_world(1:3,:,i)*robot.symbolicParameters.Qpp;
    
    switch robot.frictionIdentModel
        % Only Coulomb and viscous frictions are linear and can be identified simultaneously with the other parameters.
        % Nonlinear friction models require state dependant parameter identification
        case 'no'
            H = sym(zeros(robot.nbDOF, 2));
            H(i,:) = [robot.symbolicParameters.Qpp(i) 1];
        case 'Viscous'
            H = sym(zeros(robot.nbDOF, 3));
            H(i,:) = [robot.symbolicParameters.Qpp(i) robot.symbolicParameters.Qp(i) 1];
        case 'Coulomb'
            H = sym(zeros(robot.nbDOF, 3));
            H(i,:) = [robot.symbolicParameters.Qpp(i) tanh(100*robot.symbolicParameters.Qp(i)) 1];
        case 'ViscousCoulomb'
            
            if strcmp(robot.name, 'TX40') || strcmp(robot.name, 'RX90') % Take the coupling between joints 5 and 6 into account
                if i == 6
                    H = sym(zeros(robot.nbDOF, 5));
                    H(i-1,:) = [robot.symbolicParameters.Qpp(i) 0 0 robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i))];
                    H(i,:) = [robot.symbolicParameters.Qpp(i)+robot.symbolicParameters.Qpp(i-1) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i)) robot.symbolicParameters.Qp(i-1) tanh(100*robot.symbolicParameters.Qp(i-1))];
                else
                    H = sym(zeros(robot.nbDOF, 2));
                    H(i,:) = [robot.symbolicParameters.Qpp(i) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i))];
                end
            else
                H = sym(zeros(robot.nbDOF, 2));
                H(i,:) = [ robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i))];
            end
            
        case 'ViscousCoulombOff'
            
            if strcmp(robot.name, 'TX40') || strcmp(robot.name, 'RX90') % Take the coupling between joints 5 and 6 into account
                if i == 6
                    H = sym(zeros(robot.nbDOF, 6));
                    H(i-1,:) = [robot.symbolicParameters.Qpp(i) 0 0 robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i))];
                    H(i,:) = [robot.symbolicParameters.Qpp(i)+robot.symbolicParameters.Qpp(i-1) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i)) 1 robot.symbolicParameters.Qp(i-1) tanh(100*robot.symbolicParameters.Qp(i-1))];
                else
                    H = sym(zeros(robot.nbDOF, 4));
                    H(i,:) = [robot.symbolicParameters.Qpp(i) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i)) 1];
                end
            else
                H = sym(zeros(robot.nbDOF, 4));
                H(i,:) = [robot.symbolicParameters.Qpp(i) robot.symbolicParameters.Qp(i) tanh(100*robot.symbolicParameters.Qp(i)) 1];
            end
            
        case 'Stribeck'
            H = sym(zeros(robot.nbDOF, 2));
            H(i,:) = [robot.symbolicParameters.Qpp(i) 1];
        case 'LuGre'
            H = sym(zeros(robot.nbDOF, 2));
            H(i,:) = [robot.symbolicParameters.Qpp(i) 1];
        otherwise
            H = sym(zeros(robot.nbDOF, 2));
            H(i,:) = [robot.symbolicParameters.Qpp(i) 1];
    end

    a1 = J_dhi_world(1:3,:,i)'*(Vp_dhi_world + robot.symbolicParameters.Gravity);
    a2 = (J_dhi_world(1:3,:,i)'*Skew(Wp_cmi_world) + J_dhi_world(1:3,:,i)'*Skew(W_cmi_world)*Skew(W_cmi_world) - J_cmi_world(4:6,:,i)'*Skew(Vp_dhi_world + robot.symbolicParameters.Gravity))*Ri;
    a3 = J_cmi_world(4:6,:,i)'*(Ri*B(Ri'*Wp_cmi_world)+Skew(W_cmi_world)*Ri*B(Ri'*W_cmi_world));
    a4 = H;
    %%
    Y(:,counter+1:counter+nbparam(i)) = [a3, a2, a1,a4];
    counter = counter + nbparam(i);
end
%%
fprintf('Generating funcion handle for the regressor...\n')
matlabFunction(Y,'File',"Y_gg_handle", 'Optimize',false);

end

%% Numerical generation of Base Parameters using a QR decomposition of the regression matrix on multiple random epochs 

function [Y_b, Y_d, Beta, Xhi_b, Xhi_d, qr_P, Kd] = baseParametersNumericalSimplification(robot, Y, Xhi, samples)

disp('Generating the base parameters using QR decomposition...');
Geometry = robot.numericalParameters.Geometry;
Gravity = robot.numericalParameters.Gravity;
Obs = zeros(samples*robot.nbDOF,numel(Xhi));
% Generating an observation matrix:
fprintf('Sampling...')
for i = 1:samples
    if mod(i,50)==0
        fprintf('.')
    end
     gx=0;
    gy=0;
    gz=-9.81;
    q1 = randn(1,1);
    q2 = randn(1,1);
    q3 = randn(1,1);
    q4 = randn(1,1);
    q5 = randn(1,1);
    q6 = randn(1,1);
    qp1 = randn(1,1);
    qp2=randn(1,1);
    qp3=randn(1,1);
    qp4=randn(1,1);
    qp5=randn(1,1);
    qp6=randn(1,1);
    
    qpp1=randn(1,1);
    qpp2=randn(1,1);
    qpp3=randn(1,1);
    qpp4=randn(1,1);
    qpp5=randn(1,1);
    qpp6=randn(1,1);
    
    Obs(robot.nbDOF*(i-1)+1:robot.nbDOF*i,:) = Y_gg_handle(gx,gy,gz,q1,q2,q3,q4,q5,q6,qp1,qp2,qp3,qp4,qp5,qp6,qpp1,qpp2,qpp3,qpp4,qpp5,qpp6);
end
fprintf('\nNullspace base extraction using QR decomposition...\n')

% Evaluate the nulspace of the observation matrix (USE A ROUND FUNCTION TO AVOID THE 10^-18 FACTORS IN THE BASE PARAM EXPRESSION):
rk = rank(Obs,1e-7);
[~,qr_R,qr_P]=qr(Obs);
qr_Rb = qr_R(1:rk,1:rk);
qr_Rd = qr_R(1:rk,rk+1:end);
Kd = round(qr_Rb\qr_Rd,10);

 %Permutation matrices:
qr_Pb = qr_P(:,1:rk);
qr_Pd = qr_P(:,rk+1:end);

% Reordering as [Y_b Y_d]*[Xhi_b;Xhi_d] = Y*Xhi = Tau
Xhi_b = qr_Pb'*Xhi;
Xhi_d = qr_Pd'*Xhi;
Y_b = Y*qr_Pb;
Y_d = Y*qr_Pd;

% Generate a set of base parameters Beta such as Y_b*Beta = Y*Xhi = Tau:
Beta = Xhi_b + Kd*Xhi_d;


end

function [B] = B(v)
B = [[v(1); 0; 0], [v(2); v(1); 0], [v(3); 0; v(1)], [0; v(2); 0], [0; v(3); v(2)], [0; 0; v(3)]];
end



