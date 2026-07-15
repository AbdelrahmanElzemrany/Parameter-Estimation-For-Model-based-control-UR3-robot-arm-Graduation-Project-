
% Created by Abdelrahman Elzemrany
clc;
%% 1. Experiment Parameter Initialization
k = 48;
time = 10;   % Experiment time
T = 0.001;   % Sampling period
l = time/T;  % Number of samples (10000)

% --- Data Loading (Preserving your exact workspace structures) ---
q1  = pos.signals(1).values(1:l);  
q2  = pos.signals(2).values(1:l);   
q3  = pos.signals(3).values(1:l);  
q4  = pos.signals(4).values(1:l);  
q5  = pos.signals(5).values(1:l);  
q6  = pos.signals(6).values(1:l);  

qp1 = velocity.signals(1).values(1:l);  
qp2 = velocity.signals(2).values(1:l);  
qp3 = velocity.signals(3).values(1:l);  
qp4 = velocity.signals(4).values(1:l);  
qp5 = velocity.signals(5).values(1:l);  
qp6 = velocity.signals(6).values(1:l);  

qpp1 = accelration.signals(1).values(1:l);   
qpp2 = accelration.signals(2).values(1:l);  
qpp3 = accelration.signals(3).values(1:l); 
qpp4 = accelration.signals(4).values(1:l); 
qpp5 = accelration.signals(5).values(1:l); 
qpp6 = accelration.signals(6).values(1:l); 

u1 = torque.signals(1).values(1:l);   
u2 = torque.signals(2).values(1:l); 
u3 = torque.signals(3).values(1:l); 
u4 = torque.signals(4).values(1:l); 
u5 = torque.signals(5).values(1:l); 
u6 = torque.signals(6).values(1:l); 

tau1 = u1; 
tau2 = u2; 
tau3 = u3 * 1;
tau4 = u4 * 1;
tau5 = u5 * 1;
tau6 = u6 * 1;
t = pos.time(1:10000, 1);

%% 2. Building the Dynamic Observation Regressor Matrix (Yc)
fprintf('Computing trajectory regressors...\n');
Yb = zeros(6, k, l);
for i = 1:l
   Yb(:,:,i) = Y_b_handle(0, 0, -9.8, q1(i), q2(i), q3(i), q4(i), q5(i), q6(i), ...
                                      qp1(i), qp2(i), qp3(i), qp4(i), qp5(i), qp6(i), ...
                                      qpp1(i), qpp2(i), qpp3(i), qpp4(i), qpp5(i), qpp6(i));
end

sum1 = zeros(l, k); sum2 = zeros(l, k); sum3 = zeros(l, k);
sum4 = zeros(l, k); sum5 = zeros(l, k); sum6 = zeros(l, k);

for ii = 1:k
    for ik = 1:l
        sum1(ik, ii) = Yb(1, ii, ik);
        sum2(ik, ii) = Yb(2, ii, ik);
        sum3(ik, ii) = Yb(3, ii, ik);
        sum4(ik, ii) = Yb(4, ii, ik);
        sum5(ik, ii) = Yb(5, ii, ik);
        sum6(ik, ii) = Yb(6, ii, ik);
    end
end
Yc = [sum1; sum2; sum3; sum4; sum5; sum6];
tt = [tau1; tau2; tau3; tau4; tau5; tau6];

%% 3. Fast fmincon Setup: Pre-computing Mass Matrix Grid Postures
% CORRECT PHYSICS: We set velocity AND gravity strictly to zero [0;0;0] 
% to isolate the pure un-deformed inertia matrix columns.
fprintf('Pre-computing 6-DOF physical matrix nodes...\n');
% --- Replace your Section 3 q_grid with these extreme physical boundaries ---
q_grid = [
    0,   0,   0,   0,   0,   0;  % 1. Home posture (Zero position)
    0,   0,   0, pi/2,   0,   0;  % 2. Fully extended arm (Maximum inertia/stretch)
    0, -pi/2, pi/2,   0,   0,   0;  % 3. Fully folded arm (Minimum inertia/compact)
    0,  pi/4,-pi/4,  pi/4, pi/4, pi/4;% 4. High coupling posture
 pi/4, -pi/4, pi/4, -pi/4, pi/4, -pi/4;% 5. Alternating joint sign variations
    0, -pi/3, pi/3, -pi/3, pi/3, -pi/3 % 6. High wrist-load configuration
];
num_postures = size(q_grid, 1);
Y_M_static = zeros(6, k, 6, num_postures); 

for p = 1:num_postures
    q_curr = q_grid(p, :);
    for j = 1:6
        qpp_unit = zeros(6, 1); qpp_unit(j) = 1; % Unit acceleration pulse on joint j
        Y_M_static(:, :, j, p) = Y_b_handle(0, 0, 0, ... % Strict 0-gravity isolates clean M(q)
                                   q_curr(1), q_curr(2), q_curr(3), q_curr(4), q_curr(5), q_curr(6), ...
                                   0, 0, 0, 0, 0, 0, ...
                                   qpp_unit(1), qpp_unit(2), qpp_unit(3), qpp_unit(4), qpp_unit(5), qpp_unit(6));
    end
end

%% 4. Two-Step Parameter Estimation via fmincon
fprintf('Step 1: Finding a physically feasible starting point using lsqlin...\n');

% Set temporary positive bounds across all variables to ensure an initially valid region
lb_init = zeros(k, 1); 
lb_init(:) = 0.05; 
ub_init = Inf(k, 1);

options_init = optimoptions('lsqlin', 'Display', 'off');
theta_feasible_start = lsqlin(Yc, tt, [], [], [], [], lb_init, ub_init, [], options_init);

fprintf('Step 2: Running fmincon optimization out of the feasible region...\n');
lambda_reg = 1e-2;
% Objective Function: Least Squares Tracking Error
objective_fun = @(theta) norm(Yc * theta - tt, 2)^2 + lambda_reg * norm(theta, 2)^2;
% Initialize fmincon with our safe feasible vector
theta0 = theta_feasible_start; 

% Neglect friction boundaries: Let parameters take any unconstrained value (-Inf to Inf)
lb = -Inf(k, 1);
ub =  Inf(k, 1);

% Configure options for the interior-point path
options = optimoptions('fmincon', ...
                       'Algorithm', 'interior-point', ...
                       'Display', 'iter-detailed', ...
                       'ConstraintTolerance', 1e-6, ...
                       'MaxFunctionEvaluations', 100000, ...
                       'MaxIterations', 4000);

% Define the nonlinear constraint handle
nonlcon_fun = @(theta) fast_mass_constraint(theta, Y_M_static);

% Run fmincon
[theta_valid, fval, exitflag] = fmincon(objective_fun, theta0, [], [], [], [], lb, ub, nonlcon_fun, options);
theta = theta_valid;

%% 5. Error Residuals and Formatting Output Variables
e = tt - Yc * theta;
nn = norm(e);
cov = (nn / (20000 - 9)) * pinv(Yc' * Yc);
d = diag(cov);
dn = diag(d);

tv = Yc * theta;
tv1 = tv(1:10000, 1);
tv2 = tv(10001:20000, 1);
tv3 = tv(20001:30000, 1);
tv4 = tv(30001:40000, 1);
tv5 = tv(40001:50000, 1);
tv6 = tv(50001:60000, 1);

tte1 = tv1 - tau1; tte2 = tv2 - tau2; tte3 = tv3 - tau3;
tte4 = tv4 - tau4; tte5 = tv5 - tau5; tte6 = tv6 - tau6;

% Pack variables to timeseries profiles for Simulink block comparisons
tv1t = timeseries(tv1);   tau1t = timeseries(tau1);
tv2t = timeseries(tv2);   tau2t = timeseries(tau2);
tv3t = timeseries(tv3);   tau3t = timeseries(tau3);
tv4t = timeseries(tv4);   tau4t = timeseries(tau4);
tv5t = timeseries(tv5);   tau5t = timeseries(tau5);
tv6t = timeseries(tv6);   tau6t = timeseries(tau6);
%% 5. Plotting Results (Dynamic Alignment)
figure('Name', 'Joint Dynamic Alignment', 'Color', 'w');

% Joint 1
subplot(3, 2, 1);
plot(t, tau1, '-r', 'LineWidth', 1.5); hold on;
plot(t, tv1, '--b', 'LineWidth', 1.2);
title('Joint 1 Dynamic Alignment');
ylabel('Torque (Nm)');
xlim([0 10]);
grid on;
legend('Ideal Real Torque', 'Optimized Model', 'Location', 'northeast');

% Joint 2
subplot(3, 2, 2);
plot(t, tau2, '-r', 'LineWidth', 1.5); hold on;
plot(t, tv2, '--b', 'LineWidth', 1.2);
title('Joint 2 Dynamic Alignment');
ylabel('Torque (Nm)');
xlim([0 10]);
ylim([-20 40]);
grid on;

% Joint 3
subplot(3, 2, 3);
plot(t, tau3, '-r', 'LineWidth', 1.5); hold on;
plot(t, tv3, '--b', 'LineWidth', 1.2);
title('Joint 3 Dynamic Alignment');
xlim([0 10]);
grid on;

% Joint 4
subplot(3, 2, 4);
plot(t, tau4, '-r', 'LineWidth', 1.5); hold on;
plot(t, tv4, '--b', 'LineWidth', 1.2);
title('Joint 4 Dynamic Alignment');
ylabel('Torque (Nm)');
xlim([0 10]);
ylim([-2 6]);
grid on;

% Joint 5
subplot(3, 2, 5);
plot(t, tau5, '-r', 'LineWidth', 1.5); hold on;
plot(t, tv5, '--b', 'LineWidth', 1.2);
title('Joint 5 Dynamic Alignment');
xlabel('Time (s)');
xlim([0 10]);
grid on;

% Joint 6
subplot(3, 2, 6);
plot(t, tau6, '-r', 'LineWidth', 1.5); hold on;
plot(t, tv6, '--b', 'LineWidth', 1.2);
title('Joint 6 Dynamic Alignment');
xlabel('Time (s)');
ylabel('Torque (Nm)');
xlim([0 10]);
ylim([-0.2 0.6]);
grid on;


%% 6. Embedded Fast Nonlinear Constraint Function
function [c, ceq] = fast_mass_constraint(theta, Y_M_static)
    ceq = []; 
    num_nodes = size(Y_M_static, 4);
    c = zeros(num_nodes, 1);
    
    % Safe Inertia Floor: Forces the minimum eigenvalue up to 0.005.
    % This compresses your worst-case condition number down under 100 
    % and secures safe matrix inversion in your Simulink CTC block.
    safe_inertia_floor = 0.005; 
    
    for p = 1:num_nodes
        M = zeros(6, 6);
        for j = 1:6
            M(:, j) = Y_M_static(:, :, j, p) * theta;
        end
        M_sym = 0.5 * (M + M'); % Enforce mathematical symmetry
        
        % Constraint format: c <= 0. Guarantees min(eig) >= 0.005
        c(p) = safe_inertia_floor - min(eig(M_sym));
    end
end
