%% ========================================================================
%  CARACTERIZACIÓN EXPERIMENTAL DEL MOTOR Y DEL SISTEMA PÉNDULO DE FURUTA
%  Y DISEÑO/APLICACIÓN DE LOS CONTROLADORES PID DIGITALES (1 kHz)
%  ========================================================================
%  Proyecto Final — Control Digital (STM32 Nucleo-F446RE + L298N + 2x AS5600)
%
%  Este script en MATLAB:
%    1. Carga los datos CSV capturados desde la STM32 (o generados por el
%       banco experimental):
%         - 'datos_ensayo_escalon_motor.csv'
%         - 'datos_ensayo_oscilacion_libre_pendulo.csv'
%    2. Identifica la Función de Transferencia del Motor en velocidad Gv(s)
%       y posición Gp(s), y discretiza a Ts = 1 ms con ZOH / Tustin.
%    3. Diseña y aplica los Controladores PID del Motor (Velocidad y Posición).
%    4. Identifica la dinámica del Péndulo por Caja Negra (wn, zeta, J1, b1),
%       mapea la planta al punto de equilibrio invertido superior (alpha = pi)
%       y aplica el Controlador PID Dual (PID_alpha + PID_theta) al sistema.
%% ========================================================================
clear; clc; close all;

Ts = 0.001; % Periodo de muestreo del timer TIM6 en la STM32 (1.0 ms = 1 kHz)
s  = tf('s');
z  = tf('z', Ts);

%% ========================================================================
%  PARTE 1: CARACTERIZACIÓN DEL MOTOR NAMIKI 22CL-3501PG
%% ========================================================================
fprintf('==================================================================\n');
fprintf(' 1. CARACTERIZACION DEL MOTOR NAMIKI 22CL + DRIVER L298N\n');
fprintf('==================================================================\n');

base_dir = fileparts(fileparts(mfilename('fullpath')));
data_dir = fullfile(base_dir, 'datos_experimentales');
mot_csv  = fullfile(data_dir, 'datos_ensayo_escalon_motor.csv');
pend_csv = fullfile(data_dir, 'datos_ensayo_oscilacion_libre_pendulo.csv');

if isfile(mot_csv)
    data_mot = readmatrix(mot_csv);
    t_mot    = data_mot(:, 1) / 1000.0;
    u_mot    = data_mot(:, 2);
    theta_m  = data_mot(:, 3);
    omega_m  = data_mot(:, 4);

    idx_step = find(u_mot > 0.5, 1, 'first');
    t_step   = t_mot(idx_step:end) - t_mot(idx_step);
    u_mag    = mean(u_mot(idx_step:end));
    w_step   = omega_m(idx_step:end);

    % Estimación analítica robusta (Método del 63.2% + Estado Estacionario)
    w_ss     = mean(w_step(end-200:end));
    Km_est   = w_ss / u_mag;
    idx_63   = find(w_step >= 0.6321 * w_ss, 1, 'first');
    tau_m    = t_step(idx_63);
else
    % Valores nominales identificados del montaje
    Km_est = 1.2580;
    tau_m  = 0.0885;
end

% Funciones de transferencia continuas y discretas del motor
Gv_s = Km_est / (tau_m * s + 1);
Gp_s = Gv_s / s;
Gv_z = c2d(Gv_s, Ts, 'zoh');
Gp_z = c2d(Gp_s, Ts, 'zoh');

fprintf(' -> Ganancia estatica Km = %.4f (rad/s)/V\n', Km_est);
fprintf(' -> Constante de tiempo tau_m = %.4f s (%.2f ms)\n', tau_m, tau_m*1000);

%% ========================================================================
%  PARTE 2: APLICACIÓN DE LOS CONTROLADORES PID AL MOTOR (VELOCIDAD Y POSICIÓN)
%% ========================================================================
% 2.1. Controlador PID de Velocidad del Motor (idéntico a furuta_control.c)
Kp_v = 0.85;  Ki_v = 12.50;  Kd_v = 0.005;  N_v = 40.0;
C_vel_s = Kp_v + Ki_v/s + (Kd_v * N_v * s)/(s + N_v);
C_vel_z = c2d(C_vel_s, Ts, 'tustin');
T_vel_cl = feedback(C_vel_z * Gv_z, 1);

% 2.2. Controlador PID de Posición del Motor / Brazo
Kp_p = 9.50;  Ki_p = 3.20;   Kd_p = 0.72;   N_p = 35.0;
C_pos_s = Kp_p + Ki_p/s + (Kd_p * N_p * s)/(s + N_p);
C_pos_z = c2d(C_pos_s, Ts, 'tustin');
T_pos_cl = feedback(C_pos_z * Gp_z, 1);

figure('Name', 'Controladores PID Aplicados al Motor Namiki 22CL');
subplot(2,1,1);
step(6.0 * T_vel_cl, 2.0); grid on;
title('Respuesta en Lazo Cerrado: PID Digital de Velocidad del Motor (Referencia = 6 rad/s)');
ylabel('Velocidad \omega (rad/s)');

subplot(2,1,2);
step((pi/3) * T_pos_cl, 2.0); grid on;
title('Respuesta en Lazo Cerrado: PID Digital de Posicion del Motor/Brazo (Referencia = 60°)');
ylabel('Posicion \theta (rad)');

%% ========================================================================
%  PARTE 3: CARACTERIZACIÓN DEL SISTEMA PÉNDULO DE FURUTA (CAJA NEGRA)
%% ========================================================================
fprintf('\n==================================================================\n');
fprintf(' 2. CARACTERIZACION DEL SISTEMA PENDULO DE FURUTA\n');
fprintf('==================================================================\n');

m1 = 0.0420; L0 = 0.0885; l1 = 0.0880; g = 9.80665; J0 = 0.00052; b0 = 0.0048;
tau_k_v = 0.0062;

if isfile(pend_csv)
    data_p = readmatrix(pend_csv);
    t_p    = data_p(:, 1) / 1000.0;
    alpha_p = data_p(:, 2);

    % Detección de picos para extraer periodo T y decremento logarítmico delta
    [pks, locs] = findpeaks(alpha_p, t_p, 'MinPeakDistance', 0.4);
    T_osc = mean(diff(locs));
    wd_id = 2 * pi / T_osc;
    delta_log = log(pks(1) / pks(end)) / (length(pks) - 1);
    zeta_id = delta_log / sqrt(4*pi^2 + delta_log^2);
    wn_id   = wd_id / sqrt(1 - zeta_id^2);
else
    wn_id   = 9.7590;
    zeta_id = 0.0114;
end

J1_piv = (m1 * g * l1) / (wn_id^2);
b1_id  = 2 * zeta_id * wn_id * J1_piv;

fprintf(' -> Frecuencia natural wn = %.4f rad/s\n', wn_id);
fprintf(' -> Amortiguamiento zeta = %.5f\n', zeta_id);
fprintf(' -> Inercia en pivote J1 = %.6e kg*m^2\n', J1_piv);
fprintf(' -> Friccion viscosa b1  = %.6e N*m*s/rad\n', b1_id);

%% ========================================================================
%  PARTE 4: MAPEO AL EQUILIBRIO INVERTIDO Y APLICACIÓN DEL PID DEL SISTEMA
%% ========================================================================
J0_hat = J0 + m1 * L0^2;
M01    = m1 * L0 * l1;
Delta  = J0_hat * J1_piv - M01^2;

% Matrices de Espacio de Estados Linealizadas en alpha = pi (Invertido)
% Estado x = [theta; alpha_err; dtheta; dalpha]
A_up = [0, 0, 1, 0;
        0, 0, 0, 1;
        0, (M01 * m1 * g * l1)/Delta,   -(J1_piv * b0)/Delta,  (M01 * b1_id)/Delta;
        0, (J0_hat * m1 * g * l1)/Delta, (M01 * b0)/Delta,    -(J0_hat * b1_id)/Delta];

B_up = [0; 0; (J1_piv / Delta) * tau_k_v; -(M01 / Delta) * tau_k_v];
C_up = eye(4);
D_up = zeros(4, 1);

sys_furuta_ss = ss(A_up, B_up, C_up, D_up);
tf_matrix     = tf(sys_furuta_ss);
G_alpha_u     = -tf_matrix(2, 1); % Función de transferencia u_volts -> -alpha_err

% Controlador PID del Péndulo Invertido implementado en la STM32
Kp_a = 48.00;  Ki_a = 14.00;  Kd_a = 3.60;  N_a = 25.0;
C_alpha_s = Kp_a + Ki_a/s + (Kd_a * N_a * s)/(s + N_a);

% Lazo cerrado del subsistema péndulo invertido
T_pend_cl = feedback(C_alpha_s * G_alpha_u, 1);
fprintf('\nPolos en lazo cerrado del Pendulo Invertido con PID:\n');
disp(pole(T_pend_cl));

figure('Name', 'Estabilizacion del Sistema Pendulo de Furuta con PID');
impulse(0.20 * T_pend_cl, 2.5); grid on;
title('Recuperacion Vertical del Pendulo de Furuta ante Perturbacion con Controlador PID');
ylabel('Desviacion Angular \alpha_{err} (rad)');
