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

% Vector de frecuencia de la red
f_real = f0 * ones(1, N);              
f_real(t >= 0.25) = 55.5;              

% Acumulador de fase para generar la onda
theta_grid = cumsum(2*pi * f_real * Ts);

% Nivel de ruido 
nivel_ruido = 0.05*V_peak;

% Generación de red
va = V_peak*1.0 * cos(theta_grid)          + V_peak*0.1 * cos(5*theta_grid)          + nivel_ruido * randn(1, N);
vb = V_peak*0.6 * cos(theta_grid - 2*pi/3) + V_peak*0.1 * cos(5*theta_grid - 2*pi/3) + nivel_ruido * randn(1, N);
vc = V_peak*1.0 * cos(theta_grid + 2*pi/3) + V_peak*0.1 * cos(5*theta_grid + 2*pi/3) + nivel_ruido * randn(1, N);
vaf = 1.0 * cos(theta_grid);

%% 2. Inicialización del Bucle
w_nom = 2*pi*50;      
w_est = w_nom;    

% Ganancia del integrador
Bw = w_nom / 10;
kI = (Bw * w_nom) / 2;
integ_err = 0;             

% Memorias de estado para LPF1 
x_a1 = [0; 0]; y_a1 = [0; 0];
x_b1 = [0; 0]; y_b1 = [0; 0];

% Memorias de estado para LPF2 
x_a2 = [0; 0]; y_a2 = [0; 0];
x_b2 = [0; 0]; y_b2 = [0; 0];

% Memorias de estado para el LPF3
x_sin3 = [0; 0]; y_sin3 = [0; 0];
x_cos3 = [0; 0]; y_cos3 = [0; 0];

zeta = 0.5;


%% 3. Bucle 
for k = 1:N
    % A. Actualización Dinámica de las Matrices
    % Coeficientes (Ver Z.m)
    b0 = Ts^2 * w_est^2;
    b1 = 2 * Ts^2 * w_est^2;
    b2 = Ts^2 * w_est^2;
    
    a0 = Ts^2 * w_est^2 + 4 * zeta * Ts * w_est + 4;
    a1 = 2 * Ts^2 * w_est^2 - 8;
    a2 = Ts^2 * w_est^2 - 4 * zeta * Ts * w_est + 4;

    % B. Transformada de Clarke
    valpha = sqrt(2/3) * (va(k) - 0.5*vb(k) - 0.5*vc(k));
    vbeta  = sqrt(2/3) * (sqrt(3)/2*vb(k) - sqrt(3)/2*vc(k));
    
    % C. Cascada de Filtros LPF1 y LPF2 (Ver Z.m)
    % Alfa LPF1
    valpha_f1 = (b0*valpha + b1*x_a1(1) + b2*x_a1(2) - a1*y_a1(1) - a2*y_a1(2)) / a0;
    x_a1(2) = x_a1(1); x_a1(1) = valpha;
    y_a1(2) = y_a1(1); y_a1(1) = valpha_f1;
    
    % Alfa LPF2
    valpha_f2 = (b0*valpha_f1 + b1*x_a2(1) + b2*x_a2(2) - a1*y_a2(1) - a2*y_a2(2)) / a0;
    x_a2(2) = x_a2(1); x_a2(1) = valpha_f1;
    y_a2(2) = y_a2(1); y_a2(1) = valpha_f2;
    
    % Beta LPF1
    vbeta_f1 = (b0*vbeta + b1*x_b1(1) + b2*x_b1(2) - a1*y_b1(1) - a2*y_b1(2)) / a0;
    x_b1(2) = x_b1(1); x_b1(1) = vbeta;
    y_b1(2) = y_b1(1); y_b1(1) = vbeta_f1;
    
    % Beta LPF2
    vbeta_f2 = (b0*vbeta_f1 + b1*x_b2(1) + b2*x_b2(2) - a1*y_b2(1) - a2*y_b2(2)) / a0;
    x_b2(2) = x_b2(1); x_b2(1) = vbeta_f1;
    y_b2(2) = y_b2(1); y_b2(1) = vbeta_f2;


    % D. Extracción NPSF
    va_pos = 0.5 * (-valpha_f2 - vbeta_f1);
    vb_pos  = 0.5 * (-vbeta_f2  + valpha_f1);
    
    norm_v = sqrt(va_pos^2 + vb_pos^2);
    
    sync_sin = vb_pos / norm_v;
    sync_cos = va_pos / norm_v;

    % E. Transfomrada inversa de Clarke 
    va_clean(k) = sync_cos;
    vb_clean(k) = (-0.5 * sync_cos + (sqrt(3)/2) * sync_sin);
    vc_clean(k) = (-0.5 * sync_cos - (sqrt(3)/2) * sync_sin);
    
    % F. Estimador Frecuencia 
    % Filtro LPF3 para el Seno
    sin_f3 = (b0*sync_sin + b1*x_sin3(1) + b2*x_sin3(2) - a1*y_sin3(1) - a2*y_sin3(2)) / a0;
    x_sin3(2) = x_sin3(1); x_sin3(1) = sync_sin;
    y_sin3(2) = y_sin3(1); y_sin3(1) = sin_f3;
    
    % Filtro LPF3 para el Coseno
    cos_f3 = (b0*sync_cos + b1*x_cos3(1) + b2*x_cos3(2) - a1*y_cos3(1) - a2*y_cos3(2)) / a0;
    x_cos3(2) = x_cos3(1); x_cos3(1) = sync_cos;
    y_cos3(2) = y_cos3(1); y_cos3(1) = cos_f3;

    norma_f3_sq = sin_f3^2 + cos_f3^2;       
    error_w = 1 - norma_f3_sq;               
    integ_err = integ_err + kI * error_w * Ts; 
   
    w_est = w_nom + integ_err;

    log_va_p(k) = va_pos;
    log_sync(k) = sync_sin;
    log_f(k)    = w_est / (2*pi);
    log_valpha(k) = valpha;
end


%% 4. Gráficas
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