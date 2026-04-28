clc, clear, close all;

%% 1. Configuración General
Vp = 230*sqrt(2);
fs = 20e3;             
Ts = 1/fs;             
f0 = 50;   
w = 2*pi*f0;
T_sim = 0.25;       
t = 0:Ts:T_sim;

% Parámetro del modelo
gamma_model = 2*cos(w*Ts) + 1; 

%% 2. Generación de Escenarios de Red
vg_meas = zeros(size(t));

for i = 1:length(t)
    ti = t(i);
    
    % A. Cambio de Frecuencia (Salto a 45Hz en t > 0.15s)
    if ti < 0.15
        f_actual = 50;
    else
        f_actual = 51; % Cambio abrupto de frecuencia
    end
    w_actual = 2*pi*f_actual;
    
    % B. Cambio de Amplitud (Sag de 50% en t entre 0.05 y 0.1s)
    if ti > 0.05 && ti < 0.1
        Amp = 0.5 * Vp;
    else
        Amp = Vp;
    end
    
    % Señal fundamental
    signal = Amp * sin(w_actual * ti);
    
    % C. Armónicos (Añadimos un 5to armónico del 15%)
    harmonic = 0.05 * Amp * sin(5 * w_actual * ti);
    
    % D. Ruido Blanco (Ruido aleatorio de medición)
    noise = 5 * randn(); % Amplitud de ruido de aprox 15V
    
    % Señal Total que entra al algoritmo
    vg_meas(i) = signal + harmonic + noise;
end

%% 3. Implementación del Predictor 
vg_pred = zeros(size(t)); 
mem = [0, 0];

for i = 1:length(t)-1
    vg_pred(i+1) = gamma_model * (vg_meas(i) - mem(1)) + mem(2);
    mem(2) = mem(1);
    mem(1) = vg_meas(i);
end

%% 4. Visualización de Resultados
figure;

% Plot 1: Señal Completa vs Predicción
subplot(2,1,1);
plot(t, vg_meas, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'DisplayName', 'Medida');
hold on;
ylabel('Voltaje (V)');
title('Seguimiento de la Señal');
legend('Location','best');
grid on;
xlim([0 T_sim]);

subplot(2,1,2); 
hold on;
plot(t, vg_pred, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Predicción');
ylabel('Voltaje (V)');
title('Seguimiento de la Señal');
legend('Location','best');
grid on;
xlim([0 T_sim]);

