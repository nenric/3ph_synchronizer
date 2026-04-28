clear; clc; close all;

%% 1. PARÁMETROS
fs = 20000;           
Ts = 1/fs;
T_sim = 0.5;            
t = 0:Ts:T_sim-Ts;        
N = length(t);
Vrms = 230;
V_peak = Vrms*sqrt(2);
f0 = 50;
omega_nom = 2 * pi * 50.0;

% Vector de frecuencia de la red
f_real = f0 * ones(1, N);              
f_real(t >= 0.25) = 55;              

% Acumulador de fase para generar la onda
theta_grid = cumsum(2*pi * f_real * Ts);

% Nivel de ruido aleatorio 
nivel_ruido = 0.05*V_peak;

% Generación de red
va = V_peak*1.0 * cos(theta_grid)          + V_peak*0.1 * cos(5*theta_grid)          + nivel_ruido * randn(1, N) + V_peak*0.3 * cos(400*theta_grid)*0;
vb = V_peak*0.6 * cos(theta_grid - 2*pi/3) + V_peak*0.1 * cos(5*theta_grid - 2*pi/3) + nivel_ruido * randn(1, N) + V_peak*0.3 * cos(400*theta_grid - 2*pi/3)*0;
vc = V_peak*1.0 * cos(theta_grid + 2*pi/3) + V_peak*0.1 * cos(5*theta_grid + 2*pi/3) + nivel_ruido * randn(1, N) + V_peak*0.3 * cos(400*theta_grid + 2*pi/3)*0;
vaf = 1.0 * cos(theta_grid);

%% 3. Cálculo de la Ganancia Fija K_ss (Ver Kalman_gain.m)
% Vector de armónicos a modelar en el filtro
h_vec = [1, 3, 5, 7, 11]; 
num_states = 2 * length(h_vec); % 10 estados

H = repmat([1, 0], 1, length(h_vec)); % [1 0 1 0 1 0 1 0 1 0]
Q = eye(num_states) * 0.05 * V_peak^2;  % Varianza proceso
R = 200.0 * V_peak^2;                   % Varianza medida
P = eye(num_states);

K_ss = [0.0212114523557097;
            -0.00135635734998024;
             0.0208354350660645;
            -0.00420119767529548;
             0.0198628676435200;
            -0.00756517714779267;
             0.0170379017504671;
            -0.0127072939778248;
             0.0175215105854129;
            -0.0120317115618000];

%% 4. Inicialización del Bucle en Tiempo Real
xa = zeros(num_states, 1);
xb = zeros(num_states, 1);


% Variables del PLL para estimar la frecuencia
theta_pll = 0;
omega_est = omega_nom;
pll_int = 0;
Kp_pll = 100; % Sintonización del PI del PLL
Ki_pll = 50;

log_va_p = zeros(1, N); log_sync = zeros(1, N); log_f = zeros(1, N);

%% 5. Simulación del Algoritmo
for k = 1:N
    % A. Clarke Transform
    valpha = sqrt(2/3) * (va(k) - 0.5*vb(k) - 0.5*vc(k));
    vbeta  = sqrt(2/3) * (sqrt(3)/2 * vb(k) - sqrt(3)/2 * vc(k));
    
    % B. Matriz Phi dinámica
    Phi_k = zeros(num_states, num_states);
    for i = 1:length(h_vec)
        h = h_vec(i);
        idx = 2*i - 1;
        cos_w = cos(h * omega_est * Ts);
        sin_w = sin(h * omega_est * Ts);
        Phi_k(idx:idx+1, idx:idx+1) = [cos_w, -sin_w; sin_w, cos_w];
    end
    
    % C. Predicción KF
    xa_pred = Phi_k * xa;
    xb_pred = Phi_k * xb;
    
    % D. Corrección KF
    xa = xa_pred + K_ss * (valpha - H * xa_pred);
    xb = xb_pred + K_ss * (vbeta  - H * xb_pred);
    
    % E. Extracción NPSF
    va_pos = 0.5 * (xa(1) - xb(2));
    vb_pos = 0.5 * (xa(2) + xb(1));
    
    % F. Normalización
    norm_v = sqrt(va_pos^2 + vb_pos^2);
    sync_sin = vb_pos / norm_v;
    sync_cos = va_pos / norm_v;
    
    % G. Transfomrada inversa de Clarke 
    va_clean(k) = sync_cos;
    vb_clean(k) = (-0.5 * sync_cos + (sqrt(3)/2) * sync_sin);
    vc_clean(k) = (-0.5 * sync_cos - (sqrt(3)/2) * sync_sin);
    
    % H. Estimador Frecuencia
    % Transformada de Park
    err_pll = sync_sin * cos(theta_pll) - sync_cos * sin(theta_pll);
    
    % Controlador PI del PLL (Intentar Implemntar el PID del TFG)
    delta_w = Kp_pll * err_pll + Ki_pll * pll_int;
    pll_int = pll_int + err_pll * Ts;
    
    % Actualización de frecuencia y ángulo
    omega_est = omega_nom + delta_w;
    theta_pll = theta_pll + omega_est * Ts;
    if theta_pll > 2*pi
        theta_pll = theta_pll - 2*pi;
    end
    
    log_va_p(k) = va_pos;
    log_sync(k) = sync_sin;
    log_f(k)    = omega_est / (2*pi);
    log_valpha(k) = valpha;
end

%% 6. Gráficas
figure;
subplot(3,1,1);
plot(t, va, 'r', t, vb, 'g', t, vc, 'b');
title('1. Tensiones de Red');
ylabel('Amp (pu)'); grid on; xlim([0.2 0.3]);

subplot(3,1,2);
plot(t, log_valpha, 'Color', [0.7 0.7 0.7]); hold on;
plot(t, log_va_p, 'm', 'LineWidth', 1.5);
title('2. Tensión \alpha vs Componente Fundamental Secuencia Positiva (\alpha)');
legend('V_{\alpha}', 'V_{\alpha}^+');
ylabel('Amp (pu)'); grid on; xlim([0.2 0.3]);

subplot(3,1,3);
plot(t, log_f, 'r', 'LineWidth', 1.5); hold on;
plot(t, f_real, 'k--', 'LineWidth', 1.2);
title('3. Seguimiento Adaptativo de Frecuencia');
xlabel('Tiempo (s)'); ylabel('Frec. (Hz)'); grid on; xlim([0 0.5]);

figure;
subplot(2,1,1);
plot(t, va, 'r', t, vb, 'g', t, vc, 'b', 'LineWidth', 1.2);
title('1. Tensiones de Red');
legend('v_a', 'v_b', 'v_c');
ylabel('Amplitud');
grid on;

subplot(2,1,2);
plot(t, va_clean, 'r', t, vb_clean, 'g', t, vc_clean, 'b', 'LineWidth', 1.2);
title('2. Secuencia Positiva Reconstruida');
legend('v_{a+}', 'v_{b+}', 'v_{c+}');
ylabel('Amplitud');
grid on;


figure;
plot(t, va_clean, 'r', t, vaf, 'g', 'LineWidth', 1.2);
legend('v_{a+}', 'v_{af}');
ylabel('Amplitud');
grid on;