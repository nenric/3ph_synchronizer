clear; clc; close all;

%% 1. PARÁMETROS
Vrms = 230;
V_peak = Vrms * sqrt(2);
fs = 20000;           
Ts = 1/fs;
omega_nom = 2 * pi * 50.0;

% Vector de armónicos
h_vec = [1, 3, 5, 7, 11]; 
num_states = 2 * length(h_vec); % 10 estados

%% 2. MATRICES DEL SISTEMA (A, H, Q, R)
A = zeros(num_states, num_states);
H = [1, 0, 1, 0, 1, 0, 1, 0, 1, 0];

for i = 1:length(h_vec)
    h = h_vec(i);
    idx = 2*i - 1;
    cos_w = cos(h * omega_nom * Ts);
    sin_w = sin(h * omega_nom * Ts);
    A(idx:idx+1, idx:idx+1) = [cos_w, -sin_w; sin_w, cos_w];
end

% Matrices de covarianza de error
Q = eye(num_states) * 0.05 * V_peak^2;  % Varianza proceso
R = 200.0 * V_peak^2;                   % Varianza medida

%% 3. RESOLUCIÓN DE LA ECUACIÓN DE RICCATI CON 'idare' y 'dlqe'
[P_idare, K_idare] = idare(A', H', Q, R, [], []);

K_ss = P_idare * H' / (H * P_idare * H' + R);

G = eye(num_states);
[Kf_dlqe,P_dlqe,~] = dlqe(A,G,H,Q,R);

K_ss1 = P_dlqe * H' / (H * P_dlqe * H' + R);

%% 5. RESULTADO
disp(K_ss);

disp(K_ss1);

