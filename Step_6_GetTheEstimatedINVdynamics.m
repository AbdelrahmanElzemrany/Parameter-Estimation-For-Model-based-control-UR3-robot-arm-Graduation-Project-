% Created by Abdelrahman Elzemrany

clc;


%% 1. Initialize a clean temporary Simulink Model window
model_name = 'RRR_Estimated_Dynamics';
new_system(model_name);
open_system(model_name);

%% 2. Generate the Inertia Matrix M Block
% Bypassing 'Optimize' drops generation time down to seconds and prevents hanging
matlabFunctionBlock([model_name, '/Estimated_M'], EstTau_M, ...
                    'Vars', {Q},'Optimize',false);

%% 3. Generate the Coriolis Vector C Block
matlabFunctionBlock([model_name, '/Estimated_C'], EstTau_C, ...
                    'Vars', {Q, Qp},'Optimize',false);

%% 4. Generate the Gravity/Friction Vector G Block
matlabFunctionBlock([model_name, '/Estimated_G'], EstTau_G, ...
                     'Vars', {Q},'Optimize',false);

disp('✅ Done! Copy the 3 blocks from the new "RRR_Estimated_Dynamics" window into your model.');
