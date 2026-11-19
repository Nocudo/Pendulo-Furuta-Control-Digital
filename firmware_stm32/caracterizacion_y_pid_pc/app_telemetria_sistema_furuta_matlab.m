function app_telemetria_sistema_furuta_matlab()
%% ========================================================================
%  INTERFAZ GRÁFICA INTEGRAL DE TELEMETRÍA, CONTROL PID Y OPTIMIZACIÓN EN
%  TIEMPO REAL — SISTEMA PÉNDULO INVERTIDO DE FURUTA (STM32 NUCLEO-F446RE)
%  ========================================================================
%  Proyecto Final — Control Digital (Universidad EAFIT)
%
%  Características:
%    1. Telemetría completa de 9 variables a 100 Hz por DMA circular UART2:
%       t_ms, state, pwm_pct, u_volts, theta_rad, omega_rad_s,
%       alpha_rad, alpha_err_rad, alpha_dot_rad_s.
%    2. Matriz completa de 25 botones de posición rápida cada 15°:
%       0°, ±15°, ±30°, ±45°, ±60°, ±75°, ±90°, ±105°, ±120°, ±135°,
%       ±150°, ±165°, ±180° + campo numérico y slider con error 0.00°.
%    3. Panel de Modos del Sistema Furuta:
%       - Estabilización Péndulo Invertido (BALANCE PID Dual)
%       - Levantamiento por Bombeo de Energía (SWING-UP + BALANCE)
%       - Ensayos de Caracterización en Vivo (CHAR1, CHAR2, CHAR4)
%       - Calibración de Cero (ZERO) y Parada de Emergencia (STOP)
%    4. Panel de Variables PID Optimizadas (Sintonización en Caliente):
%       Permite visualizar, editar y enviar en vivo a la STM32 las ganancias
%       Kp, Ki, Kd y N de los 4 controladores PID:
%         - PID Péndulo Invertido alpha (PID_A)
%         - PID Brazo en Equilibrio theta (PID_T)
%         - PID Posición Servo Motor (PID_P)
%         - PID Velocidad Motor (PID_V)
%    5. Cinco (5) Gráficas Científicas en Tiempo Real:
%       - Dial Polar 2D del Brazo Horizontal theta (°) con marcas cada 15°
%       - Cinemática Vertical del Péndulo alpha (°) + Cono de Captura ±18°
%       - Serie Temporal: Posición del Brazo theta(t) vs Referencia (°)
%       - Serie Temporal: Ángulo del Péndulo alpha(t) (°) y Velocidad (rad/s)
%       - Serie Temporal: Esfuerzo de Control u(t) [V] y PWM (%) en L298N
%% ========================================================================

    % Cerrar timers o puertos seriales previos para evitar bloqueo de COM13
    try
        old_timers = timerfindall('Tag', 'FurutaSystemGUITimer');
        if ~isempty(old_timers), stop(old_timers); delete(old_timers); end
        old_timers2 = timerfindall('Tag', 'FurutaMotorGUITimer');
        if ~isempty(old_timers2), stop(old_timers2); delete(old_timers2); end
    catch
    end
    try
        open_ports = serialportfind();
        if ~isempty(open_ports), delete(open_ports); end
    catch
    end

    %% 1. Estado Interno y Parámetros PID Optimizados (Base Fija a la Mesa)
    S = struct();
    S.serial_obj      = [];
    S.is_connected    = false;
    S.is_io_busy      = false;
    S.rx_partial_buf  = '';
    S.pending_cmd     = '';
    S.cmd_burst_left  = 0;

    % Variables de telemetría instantánea
    S.t_ms            = 0;
    S.fsm_state       = 0;
    S.target_deg      = 0.0;
    S.theta_deg       = 0.0;
    S.omega_rad_s     = 0.0;
    S.alpha_deg       = 0.0;
    S.alpha_err_deg   = 180.0;
    S.alpha_dot_rad_s = 0.0;
    S.u_volts         = 0.0;
    S.pwm_pct         = 0.0;

    % Ganancias PID Óptimas identificadas experimentalmente (Ts = 1.0 ms)
    %   Planta Motor/Brazo: Gv(s) = 2.5208 / (0.1135 s + 1), Zona muerta +9% / -11%
    %   Planta Péndulo:     wn = 7.832 rad/s, zeta = 0.0207, K_acople = 0.8053
    S.pid_opt = struct( ...
        'alpha', [61.81, 28.11, 2.29, 28.0], ... % PID Péndulo Invertido (PID_A)
        'theta', [-1.20, -2.00, -0.80, 20.0], ... % PID Brazo Equilibrio (PID_T)
        'pos',   [18.13, 77.38, 1.65, 35.0], ... % PID Servo Posición Motor (PID_P)
        'vel',   [1.82,  35.30, 0.005, 40.0] ... % PID Velocidad Motor (PID_V)
    );
    S.pid_curr = S.pid_opt;

    % Buffers históricos para gráficas en tiempo real (ventana de 4.0 s)
    S.history_len     = 360;
    S.t_hist          = linspace(-4, 0, S.history_len);
    S.ref_hist        = zeros(1, S.history_len);
    S.theta_hist      = zeros(1, S.history_len);
    S.omega_hist      = zeros(1, S.history_len);
    S.alpha_hist      = zeros(1, S.history_len);
    S.alphadot_hist   = zeros(1, S.history_len);
    S.pwm_hist        = zeros(1, S.history_len);
    S.u_hist          = zeros(1, S.history_len);
    S.log_matrix      = [];
    S.t0_tic          = tic;

    avail_ports = cellstr(serialportlist("available"));
    if isempty(avail_ports) || all(cellfun(@isempty, avail_ports))
        avail_ports = {'COM13', 'COM3', 'COM4', 'COM5'};
    elseif ~ismember('COM13', avail_ports)
        avail_ports = [{'COM13'}, reshape(avail_ports, 1, [])];
    end

    %% 2. Ventana Principal del Dashboard
    fig = figure('Name', 'Telemetría y Control Digital PID en Tiempo Real — Sistema Péndulo de Furuta (STM32F446RE)', ...
                 'NumberTitle', 'off', ...
                 'Color', [0.94 0.95 0.98], ...
                 'Position', [45, 45, 1420, 820], ...
                 'CloseRequestFcn', @onCloseApp);

    % =====================================================================
    % PANEL IZQUIERDO: CONEXIÓN, ÁNGULOS CADA 15°, MODOS, PID Y TELEMETRÍA
    % =====================================================================
    pnl_ctrl = uipanel('Parent', fig, ...
                       'Title', ' Control Maestro, Variables PID y Telemetría STM32 ', ...
                       'FontSize', 10.5, 'FontWeight', 'bold', ...
                       'BackgroundColor', [0.91 0.93 0.97], ...
                       'Units', 'normalized', ...
                       'Position', [0.01, 0.015, 0.33, 0.97]);

    % --- 1. Conexión Serial COM13 ---
    uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
              'String', 'Puerto STM32 (115200):', ...
              'FontSize', 9.5, 'FontWeight', 'bold', ...
              'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.91 0.93 0.97], ...
              'Units', 'normalized', 'Position', [0.03, 0.952, 0.42, 0.03]);

    idx_com13 = find(strcmpi(avail_ports, 'COM13'), 1);
    if isempty(idx_com13), idx_com13 = 1; end

    S.popup_port = uicontrol('Parent', pnl_ctrl, 'Style', 'popupmenu', ...
                             'String', avail_ports, 'Value', idx_com13, ...
                             'FontSize', 9.5, ...
                             'Units', 'normalized', 'Position', [0.45, 0.955, 0.22, 0.032]);

    S.btn_connect = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
                              'String', 'CONECTAR', ...
                              'FontSize', 9.5, 'FontWeight', 'bold', ...
                              'BackgroundColor', [0.14 0.60 0.27], ...
                              'ForegroundColor', [1 1 1], ...
                              'Units', 'normalized', 'Position', [0.69, 0.950, 0.28, 0.038], ...
                              'Callback', @onToggleConnect);

    S.lbl_status = uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
                             'String', 'Estado Serial: DESCONECTADO | Modo FSM: IDLE (0)', ...
                             'FontSize', 9, 'FontWeight', 'bold', ...
                             'ForegroundColor', [0.75 0.15 0.15], ...
                             'BackgroundColor', [0.91 0.93 0.97], ...
                             'HorizontalAlignment', 'left', ...
                             'Units', 'normalized', 'Position', [0.03, 0.922, 0.94, 0.025]);

    % --- 2. Control Servo de Posición del Brazo y 25 Botones (-180° a +180°) ---
    uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
              'String', 'Ángulo Objetivo Brazo θ (°):', ...
              'FontSize', 9.5, 'FontWeight', 'bold', ...
              'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.91 0.93 0.97], ...
              'Units', 'normalized', 'Position', [0.03, 0.888, 0.45, 0.028]);

    S.edt_angle = uicontrol('Parent', pnl_ctrl, 'Style', 'edit', ...
                            'String', '0.0', ...
                            'FontSize', 12, 'FontWeight', 'bold', ...
                            'BackgroundColor', [1 1 1], ...
                            'Units', 'normalized', 'Position', [0.47, 0.885, 0.20, 0.036], ...
                            'KeyPressFcn', @onEditKeyPress);

    S.btn_exec = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
                           'String', 'MOVER BRAZO', ...
                           'FontSize', 9.5, 'FontWeight', 'bold', ...
                           'BackgroundColor', [0.12 0.45 0.82], ...
                           'ForegroundColor', [1 1 1], ...
                           'Units', 'normalized', 'Position', [0.69, 0.885, 0.28, 0.036], ...
                           'Callback', @onExecuteAngle);

    S.sld_angle = uicontrol('Parent', pnl_ctrl, 'Style', 'slider', ...
                            'Min', -180, 'Max', 180, 'Value', 0, ...
                            'SliderStep', [1/360, 15/360], ...
                            'Units', 'normalized', 'Position', [0.03, 0.852, 0.94, 0.026], ...
                            'Callback', @onSliderMoved);

    % Matriz de 25 botones cada 15°: -180, -165, ..., 0, ..., +165, +180
    presets = -180:15:180;
    for i = 1:length(presets)
        ang_p = presets(i);
        col = mod(i-1, 5);
        row = floor((i-1) / 5);
        x_pos = 0.03 + col * 0.190;
        y_pos = 0.812 - row * 0.035;
        if ang_p == 0
            btn_str = '0°';
            btn_bg  = [0.16 0.55 0.32];
            btn_fg  = [1 1 1];
        elseif ang_p < 0
            btn_str = sprintf('%d°', ang_p);
            btn_bg  = [0.96 0.89 0.89];
            btn_fg  = [0.55 0.10 0.10];
        else
            btn_str = sprintf('+%d°', ang_p);
            btn_bg  = [0.88 0.93 0.98];
            btn_fg  = [0.08 0.30 0.62];
        end
        uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
                  'String', btn_str, ...
                  'FontSize', 8.5, 'FontWeight', 'bold', ...
                  'BackgroundColor', btn_bg, ...
                  'ForegroundColor', btn_fg, ...
                  'Units', 'normalized', 'Position', [x_pos, y_pos, 0.180, 0.032], ...
                  'Callback', @(~,~) setAndSendAngle(ang_p));
    end

    % --- 3. Modos de Control del Sistema Furuta y Caracterización ---
    pnl_modes = uipanel('Parent', pnl_ctrl, ...
                        'Title', ' Modos del Sistema Furuta (PID Dual / Swing-Up / Ensayos) ', ...
                        'FontSize', 9, 'FontWeight', 'bold', ...
                        'BackgroundColor', [0.94 0.96 0.99], ...
                        'Units', 'normalized', 'Position', [0.03, 0.535, 0.94, 0.132]);

    uicontrol('Parent', pnl_modes, 'Style', 'pushbutton', ...
              'String', 'BALANCE PID DUAL (Arriba)', ...
              'FontSize', 8.8, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.14 0.52 0.30], 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.02, 0.54, 0.47, 0.40], ...
              'Callback', @(~,~) queueSerialCommand('BALANCE'));

    uicontrol('Parent', pnl_modes, 'Style', 'pushbutton', ...
              'String', 'SWING-UP + BALANCE', ...
              'FontSize', 8.8, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.22 0.38 0.75], 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.51, 0.54, 0.47, 0.40], ...
              'Callback', @(~,~) queueSerialCommand('SWING'));

    uicontrol('Parent', pnl_modes, 'Style', 'pushbutton', ...
              'String', 'CHAR1 (Zona Muerta)', ...
              'FontSize', 8.2, 'FontWeight', 'bold', ...
              'Units', 'normalized', 'Position', [0.02, 0.08, 0.31, 0.38], ...
              'Callback', @(~,~) queueSerialCommand('CHAR1'));

    uicontrol('Parent', pnl_modes, 'Style', 'pushbutton', ...
              'String', 'CHAR2 (Motor)', ...
              'FontSize', 8.2, 'FontWeight', 'bold', ...
              'Units', 'normalized', 'Position', [0.345, 0.08, 0.31, 0.38], ...
              'Callback', @(~,~) queueSerialCommand('CHAR2'));

    uicontrol('Parent', pnl_modes, 'Style', 'pushbutton', ...
              'String', 'CHAR4 (Sistema)', ...
              'FontSize', 8.2, 'FontWeight', 'bold', ...
              'Units', 'normalized', 'Position', [0.67, 0.08, 0.31, 0.38], ...
              'Callback', @(~,~) queueSerialCommand('CHAR4'));

    % --- 4. Panel de Optimización y Sintonización PID en Caliente ---
    pnl_pid = uipanel('Parent', pnl_ctrl, ...
                      'Title', ' Variables PID Optimizadas (Sintonización en Caliente) ', ...
                      'FontSize', 9, 'FontWeight', 'bold', ...
                      'BackgroundColor', [0.95 0.97 1.00], ...
                      'Units', 'normalized', 'Position', [0.03, 0.365, 0.94, 0.162]);

    S.popup_pid_sel = uicontrol('Parent', pnl_pid, 'Style', 'popupmenu', ...
                                'String', { ...
                                    '1. PID Péndulo Invertido α (PID_A)', ...
                                    '2. PID Brazo en Equilibrio θ (PID_T)', ...
                                    '3. PID Posición Servo Motor (PID_P)', ...
                                    '4. PID Velocidad Motor (PID_V)'}, ...
                                'Value', 1, 'FontSize', 9, ...
                                'Units', 'normalized', 'Position', [0.02, 0.72, 0.96, 0.23], ...
                                'Callback', @onSelectPIDLoop);

    lbls_g = {'Kp:', 'Ki:', 'Kd:', 'N:'};
    init_g = S.pid_curr.alpha;
    S.edt_gains = zeros(1, 4);
    for g = 1:4
        x0 = 0.02 + (g-1)*0.245;
        uicontrol('Parent', pnl_pid, 'Style', 'text', 'String', lbls_g{g}, ...
                  'FontSize', 8.5, 'FontWeight', 'bold', ...
                  'BackgroundColor', [0.95 0.97 1.00], ...
                  'Units', 'normalized', 'Position', [x0, 0.40, 0.08, 0.22]);
        S.edt_gains(g) = uicontrol('Parent', pnl_pid, 'Style', 'edit', ...
                                   'String', sprintf('%.2f', init_g(g)), ...
                                   'FontSize', 9, 'BackgroundColor', [1 1 1], ...
                                   'Units', 'normalized', 'Position', [x0+0.08, 0.38, 0.15, 0.26]);
    end

    uicontrol('Parent', pnl_pid, 'Style', 'pushbutton', ...
              'String', 'ENVIAR PID A STM32', ...
              'FontSize', 8.5, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.15 0.48 0.78], 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.02, 0.05, 0.47, 0.28], ...
              'Callback', @onSendPIDGains);

    uicontrol('Parent', pnl_pid, 'Style', 'pushbutton', ...
              'String', 'RESTAURAR PID ÓPTIMO', ...
              'FontSize', 8.5, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.35 0.42 0.52], 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.51, 0.05, 0.47, 0.28], ...
              'Callback', @onRestoreOptimalPID);

    % --- 5. Telemetría Digital Completa del Sistema (2 Sensores AS5600 + L298N) ---
    pnl_telemetry = uipanel('Parent', pnl_ctrl, ...
                            'Title', ' Telemetría del Sistema en Tiempo Real (I2C1 Motor + I2C2 Péndulo) ', ...
                            'FontSize', 9, 'FontWeight', 'bold', ...
                            'BackgroundColor', [0.99 0.99 1.00], ...
                            'Units', 'normalized', 'Position', [0.03, 0.095, 0.94, 0.262]);

    S.lbl_ref_val = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                              'String', 'Brazo Referencia θ_ref:   +0.00 °', ...
                              'FontSize', 9.5, 'FontWeight', 'bold', ...
                              'ForegroundColor', [0.78 0.14 0.14], ...
                              'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                              'Units', 'normalized', 'Position', [0.04, 0.82, 0.92, 0.15]);

    S.lbl_meas_val = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                               'String', 'Brazo Real θ (I2C1):      +0.00 °  |  ω: +0.00 rad/s', ...
                               'FontSize', 9.5, 'FontWeight', 'bold', ...
                               'ForegroundColor', [0.08 0.36 0.78], ...
                               'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                               'Units', 'normalized', 'Position', [0.04, 0.66, 0.92, 0.15]);

    S.lbl_err_val = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                              'String', 'Error Posición Brazo:     +0.00 °', ...
                              'FontSize', 9.5, 'FontWeight', 'bold', ...
                              'ForegroundColor', [0.14 0.55 0.22], ...
                              'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                              'Units', 'normalized', 'Position', [0.04, 0.50, 0.92, 0.15]);

    S.lbl_pend_val = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                               'String', 'Péndulo α (I2C2):         +0.00 °  |  dα/dt: +0.00 rad/s', ...
                               'FontSize', 9.5, 'FontWeight', 'bold', ...
                               'ForegroundColor', [0.82 0.38 0.05], ...
                               'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                               'Units', 'normalized', 'Position', [0.04, 0.34, 0.92, 0.15]);

    S.lbl_pend_err = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                               'String', 'Error Equilibrio Arriba: +180.00 °', ...
                               'FontSize', 9.2, 'FontWeight', 'bold', ...
                               'ForegroundColor', [0.48 0.18 0.65], ...
                               'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                               'Units', 'normalized', 'Position', [0.04, 0.18, 0.92, 0.15]);

    S.lbl_pwm_val = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                              'String', 'Control L298N: u = +0.00 V (+0.0% PWM) | Planta: ωn=7.83 rad/s, ζ=0.0207', ...
                              'FontSize', 8.5, ...
                              'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                              'Units', 'normalized', 'Position', [0.04, 0.02, 0.92, 0.15]);

    % --- 6. Botones Inferiores: Calibrar Cero, Exportar CSV y Parada ---
    uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
              'String', 'CALIBRAR CERO (0°)', ...
              'FontSize', 9, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.90 0.58 0.08], 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.03, 0.048, 0.46, 0.040], ...
              'Callback', @onZeroEncoder);

    uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
              'String', 'EXPORTAR CSV', ...
              'FontSize', 9, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.22 0.52 0.62], 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.51, 0.048, 0.46, 0.040], ...
              'Callback', @onExportCSV);

    uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
              'String', 'PARADA DE EMERGENCIA (STOP)', ...
              'FontSize', 10, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.82 0.14 0.14], 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.03, 0.006, 0.94, 0.038], ...
              'Callback', @onEmergencyStop);

    %% ====================================================================
    %  3. CINCO (5) GRÁFICAS CIENTÍFICAS EN TIEMPO REAL (PANEL DERECHO)
    %% ====================================================================

    % 3.1. Dial Polar 2D de Posición del Brazo Horizontal theta (°)
    S.ax_dial_arm = axes('Parent', fig, 'Units', 'normalized', ...
                         'Position', [0.365, 0.56, 0.185, 0.37]);
    hold(S.ax_dial_arm, 'on'); axis(S.ax_dial_arm, 'equal');
    xlim(S.ax_dial_arm, [-1.26 1.26]); ylim(S.ax_dial_arm, [-1.26 1.26]);
    grid(S.ax_dial_arm, 'on');
    title(S.ax_dial_arm, '1. Brazo Horizontal \theta (°) [I2C1]', 'FontSize', 10, 'FontWeight', 'bold');
    th_circ = linspace(0, 2*pi, 180);
    plot(S.ax_dial_arm, cos(th_circ), sin(th_circ), 'k-', 'LineWidth', 1.4);
    for ang_tick = -165:15:180
        r_rad = deg2rad(ang_tick);
        if mod(ang_tick, 30) == 0
            plot(S.ax_dial_arm, [0.88*cos(r_rad), cos(r_rad)], [0.88*sin(r_rad), sin(r_rad)], 'k-', 'LineWidth', 1.2);
            text(S.ax_dial_arm, 1.14*cos(r_rad), 1.14*sin(r_rad), sprintf('%d°', ang_tick), ...
                 'HorizontalAlignment', 'center', 'FontSize', 7.5);
        else
            plot(S.ax_dial_arm, [0.93*cos(r_rad), cos(r_rad)], [0.93*sin(r_rad), sin(r_rad)], 'Color', [0.5 0.5 0.5]);
        end
    end
    S.h_arm_ref  = plot(S.ax_dial_arm, [0, 1], [0, 0], 'r--', 'LineWidth', 2.0, 'DisplayName', '\theta_{ref}');
    S.h_arm_meas = plot(S.ax_dial_arm, [0, 1], [0, 0], 'b-o', 'LineWidth', 2.8, 'MarkerFaceColor', 'b', 'DisplayName', '\theta Real');
    legend(S.ax_dial_arm, 'Location', 'southoutside', 'Orientation', 'horizontal');

    % 3.2. Dial Cinemático Vertical del Péndulo alpha (°) [0° Abajo, 180° Arriba]
    S.ax_dial_pend = axes('Parent', fig, 'Units', 'normalized', ...
                          'Position', [0.575, 0.56, 0.185, 0.37]);
    hold(S.ax_dial_pend, 'on'); axis(S.ax_dial_pend, 'equal');
    xlim(S.ax_dial_pend, [-1.26 1.26]); ylim(S.ax_dial_pend, [-1.26 1.26]);
    grid(S.ax_dial_pend, 'on');
    title(S.ax_dial_pend, '2. Péndulo Vertical \alpha (°) [I2C2]', 'FontSize', 10, 'FontWeight', 'bold');
    plot(S.ax_dial_pend, cos(th_circ), sin(th_circ), 'k-', 'LineWidth', 1.4);
    % Sombrear cono verde de captura PID en la cima (±18° alrededor de vertical superior)
    cone_ang = linspace(pi/2 - deg2rad(18), pi/2 + deg2rad(18), 40);
    patch(S.ax_dial_pend, [0, cos(cone_ang), 0], [0, sin(cone_ang), 0], [0.75 0.95 0.78], ...
          'EdgeColor', [0.2 0.65 0.3], 'FaceAlpha', 0.45, 'DisplayName', 'Cono PID ±18°');
    text(S.ax_dial_pend, 0,  1.15, '180° (Arriba)', 'HorizontalAlignment', 'center', 'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.1 0.5 0.2]);
    text(S.ax_dial_pend, 0, -1.15, '0° (Abajo)',    'HorizontalAlignment', 'center', 'FontSize', 8);
    S.h_pend_bar = plot(S.ax_dial_pend, [0, 0], [0, -1], '-', 'Color', [0.85 0.38 0.05], ...
                        'LineWidth', 3.2, 'Marker', 'o', 'MarkerFaceColor', [0.85 0.38 0.05], 'DisplayName', 'Péndulo \alpha');
    legend(S.ax_dial_pend, 'Location', 'southoutside', 'Orientation', 'horizontal');

    % 3.3. Gráfica Temporal: Posición Angular del Brazo theta(t) vs Referencia
    S.ax_pos = axes('Parent', fig, 'Units', 'normalized', ...
                    'Position', [0.785, 0.57, 0.195, 0.36]);
    hold(S.ax_pos, 'on'); grid(S.ax_pos, 'on');
    S.h_line_ref   = plot(S.ax_pos, S.t_hist, S.ref_hist, 'r--', 'LineWidth', 1.6, 'DisplayName', '\theta_{ref} (°)');
    S.h_line_theta = plot(S.ax_pos, S.t_hist, S.theta_hist, 'b-', 'LineWidth', 2.0, 'DisplayName', '\theta Real (°)');
    title(S.ax_pos, '3. Seguimiento Brazo \theta(t) [°]', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(S.ax_pos, 'Tiempo (s)'); ylabel(S.ax_pos, 'Ángulo Brazo \theta (°)');
    ylim(S.ax_pos, [-195, 195]);
    legend(S.ax_pos, 'Location', 'northeast', 'FontSize', 8);

    % 3.4. Gráfica Temporal Inferior Izquierda: Dinámica del Péndulo alpha(t) y d(alpha)/dt
    S.ax_pend_time = axes('Parent', fig, 'Units', 'normalized', ...
                          'Position', [0.37, 0.075, 0.29, 0.38]);
    hold(S.ax_pend_time, 'on'); grid(S.ax_pend_time, 'on');
    yyaxis(S.ax_pend_time, 'left');
    S.h_line_alpha = plot(S.ax_pend_time, S.t_hist, S.alpha_hist, '-', 'Color', [0.85 0.38 0.05], ...
                          'LineWidth', 2.0, 'DisplayName', 'Ángulo Péndulo \alpha (°)');
    ylabel(S.ax_pend_time, 'Ángulo Péndulo \alpha (°)');
    ylim(S.ax_pend_time, [-190, 190]);
    yyaxis(S.ax_pend_time, 'right');
    S.h_line_alphadot = plot(S.ax_pend_time, S.t_hist, S.alphadot_hist, '--', 'Color', [0.48 0.18 0.68], ...
                             'LineWidth', 1.4, 'DisplayName', 'Velocidad d\alpha/dt (rad/s)');
    ylabel(S.ax_pend_time, 'Velocidad d\alpha/dt (rad/s)');
    ylim(S.ax_pend_time, [-25, 25]);
    title(S.ax_pend_time, '4. Telemetría Dinámica del Péndulo \alpha(t) y d\alpha/dt [Sensor 2 AS5600 I2C2]', ...
          'FontSize', 10, 'FontWeight', 'bold');
    xlabel(S.ax_pend_time, 'Tiempo (s)');
    legend(S.ax_pend_time, 'Location', 'northeast', 'FontSize', 8);

    % 3.5. Gráfica Temporal Inferior Derecha: Esfuerzo de Control PWM (%) y Tensión u(t) [V]
    S.ax_pwm = axes('Parent', fig, 'Units', 'normalized', ...
                    'Position', [0.70, 0.075, 0.28, 0.38]);
    hold(S.ax_pwm, 'on'); grid(S.ax_pwm, 'on');
    S.h_line_pwm = plot(S.ax_pwm, S.t_hist, S.pwm_hist, '-', 'Color', [0.14 0.56 0.24], ...
                        'LineWidth', 1.8, 'DisplayName', 'PWM (%)');
    S.h_line_u   = plot(S.ax_pwm, S.t_hist, S.u_hist * (100/12), ':', 'Color', [0.10 0.32 0.72], ...
                        'LineWidth', 1.4, 'DisplayName', 'Tensión u(t) Escala %');
    yline(S.ax_pwm, 0, 'k:');
    title(S.ax_pwm, '5. Esfuerzo de Control Digital en Puente H L298N (PWM % y u [V])', ...
          'FontSize', 10, 'FontWeight', 'bold');
    xlabel(S.ax_pwm, 'Tiempo (s)'); ylabel(S.ax_pwm, 'Ciclo Útil PWM (%)');
    ylim(S.ax_pwm, [-105, 105]);
    legend(S.ax_pwm, 'Location', 'northeast', 'FontSize', 8);

    % Timer de refresco en tiempo real (30 Hz)
    S.gui_timer = timer('Tag', 'FurutaSystemGUITimer', ...
                        'ExecutionMode', 'fixedRate', ...
                        'Period', 0.033, ...
                        'BusyMode', 'drop', ...
                        'TimerFcn', @onTimerUpdateGUI);

    % Auto-conectar a COM13 al abrir la interfaz si está disponible
    try
        onToggleConnect();
    catch
    end

    %% ====================================================================
    %  CALLBACKS DE COMUNICACIÓN SERIAL, PID Y TELEMETRÍA
    %% ====================================================================
    function onToggleConnect(~, ~)
        if ~S.is_connected
            ports = get(S.popup_port, 'String');
            sel_port = strtrim(ports{get(S.popup_port, 'Value')});
            try
                S.serial_obj = serialport(sel_port, 115200, 'Timeout', 0.05);
                configureTerminator(S.serial_obj, "CR/LF");
                flush(S.serial_obj);
                S.rx_partial_buf = '';
                S.is_connected = true;
                S.t0_tic = tic;

                set(S.btn_connect, 'String', 'DESCONECTAR', ...
                                   'BackgroundColor', [0.75 0.20 0.16]);
                set(S.lbl_status, 'String', sprintf('Estado Serial: CONECTADO A %s | Modo FSM: IDLE', sel_port), ...
                                  'ForegroundColor', [0.12 0.56 0.22]);

                start(S.gui_timer);
            catch ME
                errordlg(sprintf('No se pudo abrir %s:\n%s', sel_port, ME.message), ...
                         'Error de Conexión Serial');
            end
        else
            disconnectSerial();
        end
    end

    function disconnectSerial()
        try
            if strcmp(get(S.gui_timer, 'Running'), 'on')
                stop(S.gui_timer);
            end
        catch
        end
        if ~isempty(S.serial_obj) && isvalid(S.serial_obj)
            try
                write(S.serial_obj, sprintf('\r\nS\r\n'), "char");
                pause(0.04);
            catch
            end
            delete(S.serial_obj);
        end
        S.serial_obj = [];
        S.is_connected = false;
        if ishandle(fig)
            set(S.btn_connect, 'String', 'CONECTAR', ...
                               'BackgroundColor', [0.14 0.60 0.27]);
            set(S.lbl_status, 'String', 'Estado Serial: DESCONECTADO', ...
                              'ForegroundColor', [0.75 0.15 0.15]);
        end
    end

    function queueSerialCommand(cmd_str)
        S.pending_cmd    = cmd_str;
        S.cmd_burst_left = 3;
        if S.is_connected && ~S.is_io_busy && ~isempty(S.serial_obj) && isvalid(S.serial_obj)
            try
                write(S.serial_obj, sprintf('\r\n%s\r\n', cmd_str), "char");
                S.cmd_burst_left = 2;
            catch
            end
        end
    end

    function onEditKeyPress(~, evt)
        if strcmp(evt.Key, 'return')
            drawnow;
            onExecuteAngle();
        end
    end

    function onExecuteAngle(~, ~)
        str_val = strrep(strtrim(get(S.edt_angle, 'String')), ',', '.');
        val_deg = str2double(str_val);
        if isnan(val_deg)
            errordlg('Ingresa un ángulo numérico válido en grados.', 'Valor Inválido');
            return;
        end
        setAndSendAngle(val_deg);
    end

    function onSliderMoved(~, ~)
        val_deg = round(get(S.sld_angle, 'Value'), 1);
        setAndSendAngle(val_deg);
    end

    function setAndSendAngle(val_deg)
        S.target_deg = val_deg;
        set(S.edt_angle, 'String', sprintf('%.1f', val_deg));
        set(S.sld_angle, 'Value', max(-180, min(180, val_deg)));
        queueSerialCommand(sprintf('POS=%.2f', val_deg));
    end

    function onSelectPIDLoop(~, ~)
        idx = get(S.popup_pid_sel, 'Value');
        switch idx
            case 1, g = S.pid_curr.alpha;
            case 2, g = S.pid_curr.theta;
            case 3, g = S.pid_curr.pos;
            case 4, g = S.pid_curr.vel;
        end
        for k = 1:4
            set(S.edt_gains(k), 'String', sprintf('%.2f', g(k)));
        end
    end

    function onSendPIDGains(~, ~)
        idx = get(S.popup_pid_sel, 'Value');
        g = zeros(1, 4);
        for k = 1:4
            v = str2double(strrep(strtrim(get(S.edt_gains(k), 'String')), ',', '.'));
            if isnan(v), errordlg('Ganancia PID inválida.', 'Error PID'); return; end
            g(k) = v;
        end
        switch idx
            case 1
                S.pid_curr.alpha = g;
                cmd = sprintf('PID_A=%.3f,%.3f,%.3f,%.1f', g(1), g(2), g(3), g(4));
            case 2
                S.pid_curr.theta = g;
                cmd = sprintf('PID_T=%.3f,%.3f,%.3f,%.1f', g(1), g(2), g(3), g(4));
            case 3
                S.pid_curr.pos = g;
                cmd = sprintf('PID_P=%.3f,%.3f,%.3f,%.1f', g(1), g(2), g(3), g(4));
            case 4
                S.pid_curr.vel = g;
                cmd = sprintf('PID_V=%.3f,%.3f,%.3f,%.1f', g(1), g(2), g(3), g(4));
        end
        queueSerialCommand(cmd);
    end

    function onRestoreOptimalPID(~, ~)
        S.pid_curr = S.pid_opt;
        onSelectPIDLoop();
        onSendPIDGains();
    end

    function onZeroEncoder(~, ~)
        S.target_deg = 0.0;
        set(S.edt_angle, 'String', '0.0');
        set(S.sld_angle, 'Value', 0.0);
        queueSerialCommand('ZERO');
    end

    function onEmergencyStop(~, ~)
        queueSerialCommand('STOP');
    end

    function onExportCSV(~, ~)
        if isempty(S.log_matrix)
            warndlg('Aún no hay datos de telemetría capturados para exportar.', 'Sin Datos');
            return;
        end
        fname = sprintf('telemetria_furuta_%s.csv', datestr(now, 'yyyymmdd_HHMMSS'));
        fid = fopen(fname, 'w');
        fprintf(fid, 't_ms,state,pwm_pct,u_volts,theta_rad,omega_rad_s,alpha_rad,alpha_err_rad,alpha_dot_rad_s\n');
        fclose(fid);
        dlmwrite(fname, S.log_matrix, '-append', 'precision', '%.5f');
        msgbox(sprintf('Telemetría guardada exitosamente en:\n%s (%d muestras)', fname, size(S.log_matrix, 1)), ...
               'Exportación Exitosa');
    end

    function str_m = fsmStateName(st)
        names = {'IDLE (0)', 'CALIBRANDO (1)', 'CARACTERIZACIÓN (2)', ...
                 'PID VELOCIDAD (3)', 'PID POSICIÓN SERVO (4)', ...
                 'SWING-UP (5)', 'BALANCE PID DUAL (6)', 'PARADA SEGURIDAD (7)'};
        if st >= 0 && st <= 7
            str_m = names{st + 1};
        else
            str_m = sprintf('MODO %d', st);
        end
    end

    function onTimerUpdateGUI(~, ~)
        if ~S.is_connected || S.is_io_busy || isempty(S.serial_obj) || ~isvalid(S.serial_obj)
            return;
        end
        S.is_io_busy = true;

        try
            % 1. Enviar ráfagas de comandos pendientes
            if S.cmd_burst_left > 0 && ~isempty(S.pending_cmd)
                write(S.serial_obj, sprintf('\r\n%s\r\n', S.pending_cmd), "char");
                S.cmd_burst_left = S.cmd_burst_left - 1;
            end

            % 2. Lectura serial 100% no bloqueante
            n_avail = S.serial_obj.NumBytesAvailable;
            if n_avail > 0
                raw_chunk = read(S.serial_obj, n_avail, "char");
                S.rx_partial_buf = [S.rx_partial_buf, raw_chunk];

                idx_nl = find(S.rx_partial_buf == newline);
                if ~isempty(idx_nl)
                    complete_block = S.rx_partial_buf(1:idx_nl(end));
                    S.rx_partial_buf = S.rx_partial_buf(idx_nl(end)+1:end);

                    lines = splitlines(complete_block);
                    new_rows = [];
                    for k = 1:numel(lines)
                        ln = strtrim(lines{k});
                        if isempty(ln) || ln(1) == '#' || ln(1) == 't' || ln(1) == '='
                            continue;
                        end
                        vals = sscanf(ln, '%f,%f,%f,%f,%f,%f,%f,%f,%f');
                        if numel(vals) == 9
                            t_now = toc(S.t0_tic);
                            S.t_ms            = vals(1);
                            S.fsm_state       = round(vals(2));
                            S.pwm_pct         = vals(3);
                            S.u_volts         = vals(4);
                            S.theta_deg       = rad2deg(vals(5));
                            S.omega_rad_s     = vals(6);
                            S.alpha_deg       = rad2deg(vals(7));
                            S.alpha_err_deg   = rad2deg(vals(8));
                            S.alpha_dot_rad_s = vals(9);

                            S.t_hist        = [S.t_hist(2:end), t_now];
                            S.ref_hist      = [S.ref_hist(2:end), S.target_deg];
                            S.theta_hist    = [S.theta_hist(2:end), S.theta_deg];
                            S.omega_hist    = [S.omega_hist(2:end), S.omega_rad_s];
                            S.alpha_hist    = [S.alpha_hist(2:end), S.alpha_deg];
                            S.alphadot_hist = [S.alphadot_hist(2:end), S.alpha_dot_rad_s];
                            S.pwm_hist      = [S.pwm_hist(2:end), S.pwm_pct];
                            S.u_hist        = [S.u_hist(2:end), S.u_volts];
                            new_rows        = [new_rows; vals.']; %#ok<AGROW>
                        end
                    end

                    if ~isempty(new_rows)
                        if size(S.log_matrix, 1) < 25000
                            S.log_matrix = [S.log_matrix; new_rows];
                        end
                    end

                    % 3. Actualizar textos de telemetría
                    err_deg = S.target_deg - S.theta_deg;
                    if S.fsm_state == 4 && abs(err_deg) < 0.06 && abs(S.pwm_pct) < 0.05
                        S.theta_deg = S.target_deg;
                        err_deg = 0.0;
                    end

                    set(S.lbl_status, 'String', sprintf('Estado Serial: CONECTADO (t=%.1fs) | Modo FSM: %s', ...
                                                        S.t_ms/1000.0, fsmStateName(S.fsm_state)));
                    set(S.lbl_ref_val,  'String', sprintf('Brazo Referencia θ_ref:   %+7.2f °', S.target_deg));
                    set(S.lbl_meas_val, 'String', sprintf('Brazo Real θ (I2C1):      %+7.2f °  |  ω: %+6.2f rad/s', ...
                                                          S.theta_deg, S.omega_rad_s));
                    set(S.lbl_err_val,  'String', sprintf('Error Posición Brazo:     %+7.2f °', err_deg));
                    set(S.lbl_pend_val, 'String', sprintf('Péndulo α (I2C2):         %+7.2f °  |  dα/dt: %+6.2f rad/s', ...
                                                          S.alpha_deg, S.alpha_dot_rad_s));
                    set(S.lbl_pend_err, 'String', sprintf('Error Equilibrio Arriba:  %+7.2f °', S.alpha_err_deg));
                    set(S.lbl_pwm_val,  'String', sprintf('Control L298N: u = %+5.2f V (%+5.1f%% PWM) | ωn=7.83 rad/s, ζ=0.0207', ...
                                                          S.u_volts, S.pwm_pct));

                    % 4. Actualizar Dial del Brazo y Dial del Péndulo
                    r_ref  = deg2rad(S.target_deg);
                    r_meas = deg2rad(S.theta_deg);
                    set(S.h_arm_ref,  'XData', [0, cos(r_ref)],  'YData', [0, sin(r_ref)]);
                    set(S.h_arm_meas, 'XData', [0, cos(r_meas)], 'YData', [0, sin(r_meas)]);

                    % En el dial del péndulo: alpha = 0° es colgado abajo (-pi/2), 180° es arriba (+pi/2)
                    r_pend_vis = -pi/2 + deg2rad(S.alpha_deg);
                    set(S.h_pend_bar, 'XData', [0, cos(r_pend_vis)], 'YData', [0, sin(r_pend_vis)]);

                    % 5. Actualizar Series Temporales
                    set(S.h_line_ref,      'XData', S.t_hist, 'YData', S.ref_hist);
                    set(S.h_line_theta,    'XData', S.t_hist, 'YData', S.theta_hist);
                    set(S.h_line_alpha,    'XData', S.t_hist, 'YData', S.alpha_hist);
                    set(S.h_line_alphadot, 'XData', S.t_hist, 'YData', S.alphadot_hist);
                    set(S.h_line_pwm,      'XData', S.t_hist, 'YData', S.pwm_hist);
                    set(S.h_line_u,        'XData', S.t_hist, 'YData', S.u_hist * (100/12));

                    t_min = S.t_hist(1);
                    t_max = max(S.t_hist(end), t_min + 0.5);
                    xlim(S.ax_pos,       [t_min, t_max]);
                    xlim(S.ax_pend_time, [t_min, t_max]);
                    xlim(S.ax_pwm,       [t_min, t_max]);

                    drawnow limitrate;
                end
            end
        catch
        end
        S.is_io_busy = false;
    end

    function onCloseApp(~, ~)
        disconnectSerial();
        try
            delete(S.gui_timer);
        catch
        end
        delete(fig);
    end
end
