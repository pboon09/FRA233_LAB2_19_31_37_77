%% Pendulum Param
% ค่าที่ตั้งไว้เป้นค่าสูงสุดที่มอเตอร์สามารถหมุนได้โดยไม่เกิน 12V้
L = 0.1;     % [m]  (massless link)
mp = 0.05;   % [kg] (point mass)
g = 9.81;    % [m/s^2]
%% Motor Param 
% นำค่าที่ได้จาก lab1 มาใส่
kt = 5.06e-2;
ke = 5.28e-2;
Lm = 2.84e-3;
R = 3.18;
b = 7.76e-5;
J = 5.86e-5;

% Single Loop Control
kp = 0.063099;
time = Lm/R;
N_2 = mp*L*L*Lm + Lm*J;
N_1 = R*J + Lm*b + mp*L*L*R;
N_0 = R*b + ke*kt;

plant = tf(kt, [N_2 N_1 N_0]);
integrator = tf([1], [1 0]);

closedLoopSystem = feedback(plant * integrator * kp, 1);

controlSystemDesigner('rlocus', closedLoopSystem)