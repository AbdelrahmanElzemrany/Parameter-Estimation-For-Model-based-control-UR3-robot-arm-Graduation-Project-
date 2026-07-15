% Created by Abdelrahman Elzemrany

clc;
Ebeta=sym(theta);
EstTau=Y_b*Ebeta ;
syms gx gy gz

%% 1. Change EstTau to isolate the Gravity & Friction Matrix (G_F)
% Assign Qp = 0, Qpp = 0, and bake in numeric gravity. 
% Remaining active variable to pass: Q
EstTau_G = subs(EstTau, [Qp; Qpp; gx; gy; gz], [zeros(12,1); 0; 0; -9.8]);
EstTau_H = subs(EstTau, [Qpp; gx; gy; gz], [zeros(6,1); 0; 0; -9.8]);
EstTau_C = EstTau_H - EstTau_G;
EstTau_M_base = subs(EstTau, [Qp; gx; gy; gz], [zeros(6,1); 0; 0; 0]);

% Reconstruct the 6x6 matrix column-by-column using unit acceleration pulses
EstTau_M = sym(zeros(6,6));
for j = 1:6
    I_col = zeros(6,1);
    I_col(j) = 1; 
    
    % Substitute the symbolic Qpp vector with the numerical unit pulse
    EstTau_M(:,j) = subs(EstTau_M_base, Qpp, I_col);
end
matlabFunction(EstTau_M, 'File', 'get_UR3_M','Optimize',false, 'Vars', {Q});
matlabFunction(EstTau_C, 'File', 'get_UR3_C','Optimize',false, 'Vars', {Q, Qp});
matlabFunction(EstTau_G,  'File', 'get_UR3_G','Optimize',false, 'Vars', {Q}); % Note: Use your variable 'EstTau_G' here if needed