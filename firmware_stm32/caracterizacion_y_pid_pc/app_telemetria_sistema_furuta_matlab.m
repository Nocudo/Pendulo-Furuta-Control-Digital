function app_telemetria_sistema_furuta_matlab()
%% ========================================================================
%  INTERFAZ DE TELEMETRÍA Y CONTROL AUTÓNOMO EN TIEMPO REAL
%  SISTEMA PÉNDULO INVERTIDO DE FURUTA (STM32 NUCLEO-F446RE)
%  ========================================================================
%  Proyecto Final — Control Digital (Universidad EAFIT)
%
%  Características:
%    1. Operación con un único botón maestro (Toggle):
%       - "INICIAR PÉNDULO (SUBIR Y EQUILIBRAR)": arranca siempre desde 0°,
%         levanta el péndulo, lo mantiene equilibrado sin límite angular en
%         el brazo ante perturbaciones y, si cae, regresa automáticamente a
%         0° y vuelve a levantarlo de forma indefinida.
%       - "DETENER Y VOLVER A 0°": detiene el ciclo autónomo, devuelve el
%         brazo suavemente a 0° y apaga el motor en espera (IDLE).
%    2. Telemetría completa en tiempo real (I2C1 Motor + I2C2 Péndulo + L298N)
%       con exportación a CSV y calibración rápida de cero (0°).
%    3. Cinco (5) gráficas científicas limpias en tiempo real sin etiquetas
%       espurias ("data X") en los diales circulares.
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

    %% 1. Estado Interno
    S = struct();
    S.serial_obj      = [];
    S.is_connected    = false;
    S.is_io_busy      = false;
    S.is_running      = false;   % Estado del botón único Iniciar / Detener y Volver a 0°
    S.rx_partial_buf  = '';
    S.pending_cmd     = '';
    S.cmd_burst_left  = 0;

    % Variables de telemetría instantánea
    S.t_ms            = 0;
    S.fsm_state       = 0;
    S.theta_deg       = 0.0;
    S.omega_rad_s     = 0.0;
    S.alpha_deg       = 0.0;
    S.alpha_err_deg   = 180.0;
    S.alpha_dot_rad_s = 0.0;
    S.u_volts         = 0.0;
    S.pwm_pct         = 0.0;

    % Buffers históricos para gráficas en tiempo real (ventana de 5.0 s)
    S.history_len     = 400;
    S.t_hist          = linspace(-5, 0, S.history_len);
    S.theta_hist      = zeros(1, S.history_len);
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
    fig = figure('Name', 'Péndulo Invertido de Furuta — Control Autónomo y Telemetría (STM32F446RE)', ...
                 'NumberTitle', 'off', ...
                 'Color', [0.94 0.95 0.98], ...
                 'Position', [45, 45, 1420, 810], ...
                 'CloseRequestFcn', @onCloseApp);

    % =====================================================================
    % PANEL IZQUIERDO LIMPIO: CONEXIÓN, BOTÓN ÚNICO Y TELEMETRÍA
    % =====================================================================
    pnl_ctrl = uipanel('Parent', fig, ...
                       'Title', ' Control Autónomo y Telemetría STM32 ', ...
                       'FontSize', 10.5, 'FontWeight', 'bold', ...
                       'BackgroundColor', [0.91 0.93 0.97], ...
                       'Units', 'normalized', ...
                       'Position', [0.012, 0.018, 0.275, 0.964]);

    % --- 1. Conexión Serial COM13 ---
    uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
              'String', 'Puerto STM32:', ...
              'FontSize', 9.5, 'FontWeight', 'bold', ...
              'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.91 0.93 0.97], ...
              'Units', 'normalized', 'Position', [0.04, 0.935, 0.34, 0.032]);

    idx_com13 = find(strcmpi(avail_ports, 'COM13'), 1);
    if isempty(idx_com13), idx_com13 = 1; end

    S.popup_port = uicontrol('Parent', pnl_ctrl, 'Style', 'popupmenu', ...
                             'String', avail_ports, 'Value', idx_com13, ...
                             'FontSize', 9.5, ...
                             'Units', 'normalized', 'Position', [0.38, 0.938, 0.25, 0.034]);

    S.btn_connect = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
                              'String', 'CONECTAR', ...
                              'FontSize', 9.2, 'FontWeight', 'bold', ...
                              'BackgroundColor', [0.14 0.60 0.27], ...
                              'ForegroundColor', [1 1 1], ...
                              'Units', 'normalized', 'Position', [0.65, 0.933, 0.31, 0.042], ...
                              'Callback', @onToggleConnect);

    S.lbl_status = uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
                             'String', 'Serial: DESCONECTADO | Estado: IDLE', ...
                             'FontSize', 9.0, 'FontWeight', 'bold', ...
                             'ForegroundColor', [0.75 0.15 0.15], ...
                             'BackgroundColor', [0.91 0.93 0.97], ...
                             'HorizontalAlignment', 'left', ...
                             'Units', 'normalized', 'Position', [0.04, 0.892, 0.92, 0.032]);

    % --- 2. Panel de Control Maestro (Botón Único Toggle) ---
    pnl_action = uipanel('Parent', pnl_ctrl, ...
                         'Title', ' Operación Autónoma Continua ', ...
                         'FontSize', 9.5, 'FontWeight', 'bold', ...
                         'BackgroundColor', [0.95 0.97 1.00], ...
                         'Units', 'normalized', 'Position', [0.04, 0.615, 0.92, 0.265]);

    S.btn_master = uicontrol('Parent', pnl_action, 'Style', 'pushbutton', ...
                             'String', '▶ INICIAR PÉNDULO (SUBIR Y EQUILIBRAR)', ...
                             'FontSize', 10.5, 'FontWeight', 'bold', ...
                             'BackgroundColor', [0.12 0.54 0.28], ...
                             'ForegroundColor', [1 1 1], ...
                             'Units', 'normalized', 'Position', [0.05, 0.56, 0.90, 0.35], ...
                             'Callback', @onToggleMasterRun);

    S.lbl_mode_hint = uicontrol('Parent', pnl_action, 'Style', 'text', ...
                                'String', sprintf(['Listo en 0°. Al iniciar:\n' ...
                                                   '• Sube siempre desde 0° y estabiliza arriba.\n' ...
                                                   '• Sin límite de ángulo ante perturbaciones.\n' ...
                                                   '• Si cae, vuelve a 0° y reintenta solo.']), ...
                                'FontSize', 8.6, ...
                                'ForegroundColor', [0.22 0.28 0.38], ...
                                'BackgroundColor', [0.95 0.97 1.00], ...
                                'HorizontalAlignment', 'left', ...
                                'Units', 'normalized', 'Position', [0.05, 0.06, 0.90, 0.45]);

    % --- 3. Telemetría Digital en Tiempo Real ---
    pnl_telemetry = uipanel('Parent', pnl_ctrl, ...
                            'Title', ' Telemetría del Sistema (I2C1 + I2C2 + L298N) ', ...
                            'FontSize', 9.5, 'FontWeight', 'bold', ...
                            'BackgroundColor', [0.99 0.99 1.00], ...
                            'Units', 'normalized', 'Position', [0.04, 0.145, 0.92, 0.450]);

    S.lbl_fsm_big = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                              'String', 'ESTADO: DETENIDO EN 0° (IDLE)', ...
                              'FontSize', 10.0, 'FontWeight', 'bold', ...
                              'ForegroundColor', [0.16 0.34 0.68], ...
                              'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                              'Units', 'normalized', 'Position', [0.05, 0.84, 0.90, 0.12]);

    S.lbl_arm_pos = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                              'String', 'Brazo θ (I2C1):         +0.00 °', ...
                              'FontSize', 10.0, 'FontWeight', 'bold', ...
                              'ForegroundColor', [0.08 0.36 0.78], ...
                              'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                              'Units', 'normalized', 'Position', [0.05, 0.69, 0.90, 0.12]);

    S.lbl_arm_vel = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                              'String', 'Velocidad Brazo ω:      +0.00 rad/s', ...
                              'FontSize', 9.5, ...
                              'ForegroundColor', [0.18 0.42 0.72], ...
                              'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                              'Units', 'normalized', 'Position', [0.05, 0.56, 0.90, 0.11]);

    S.lbl_pend_pos = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                               'String', 'Péndulo α (I2C2):       +0.00 °', ...
                               'FontSize', 10.0, 'FontWeight', 'bold', ...
                               'ForegroundColor', [0.82 0.38 0.05], ...
                               'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                               'Units', 'normalized', 'Position', [0.05, 0.41, 0.90, 0.12]);

    S.lbl_pend_err = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                               'String', 'Error Vertical α_err: +180.00 °', ...
                               'FontSize', 10.0, 'FontWeight', 'bold', ...
                               'ForegroundColor', [0.55 0.16 0.62], ...
                               'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                               'Units', 'normalized', 'Position', [0.05, 0.27, 0.90, 0.12]);

    S.lbl_pend_vel = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                               'String', 'Velocidad dα/dt:        +0.00 rad/s', ...
                               'FontSize', 9.5, ...
                               'ForegroundColor', [0.35 0.35 0.42], ...
                               'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                               'Units', 'normalized', 'Position', [0.05, 0.14, 0.90, 0.11]);

    S.lbl_pwm_val = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                              'String', 'Motor L298N: u = +0.00 V (+0.0% PWM)', ...
                              'FontSize', 9.2, 'FontWeight', 'bold', ...
                              'ForegroundColor', [0.12 0.50 0.24], ...
                              'HorizontalAlignment', 'left', 'BackgroundColor', [0.99 0.99 1.00], ...
                              'Units', 'normalized', 'Position', [0.05, 0.02, 0.90, 0.11]);

    % --- 4. Utilidades Inferiores: Calibrar Cero y Exportar CSV ---
    uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
              'String', 'CALIBRAR CERO (0°)', ...
              'FontSize', 9.2, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.88 0.56 0.08], 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.04, 0.075, 0.92, 0.052], ...
              'Callback', @onZeroEncoder);

    uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
              'String', 'EXPORTAR TELEMETRÍA CSV', ...
              'FontSize', 9.2, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.22 0.50 0.62], 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.04, 0.015, 0.92, 0.050], ...
              'Callback', @onExportCSV);

    %% ====================================================================
    %  3. CINCO (5) GRÁFICAS CIENTÍFICAS EN TIEMPO REAL (SIN "data X")
    %% ====================================================================

    % 3.1. Dial Polar 2D del Brazo Horizontal theta (°)
    S.ax_dial_arm = axes('Parent', fig, 'Units', 'normalized', ...
                         'Position', [0.315, 0.555, 0.200, 0.375]);
    hold(S.ax_dial_arm, 'on'); axis(S.ax_dial_arm, 'equal');
    xlim(S.ax_dial_arm, [-1.28 1.28]); ylim(S.ax_dial_arm, [-1.28 1.28]);
    grid(S.ax_dial_arm, 'on');
    title(S.ax_dial_arm, '1. Brazo Horizontal \theta (°) [I2C1]', 'FontSize', 10, 'FontWeight', 'bold');
    th_circ = linspace(0, 2*pi, 180);
    plot(S.ax_dial_arm, cos(th_circ), sin(th_circ), 'k-', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    for ang_tick = -165:15:180
        r_rad = deg2rad(ang_tick);
        if mod(ang_tick, 30) == 0
            plot(S.ax_dial_arm, [0.88*cos(r_rad), cos(r_rad)], [0.88*sin(r_rad), sin(r_rad)], ...
                 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off');
            text(S.ax_dial_arm, 1.15*cos(r_rad), 1.15*sin(r_rad), sprintf('%d°', ang_tick), ...
                 'HorizontalAlignment', 'center', 'FontSize', 7.5);
        else
            plot(S.ax_dial_arm, [0.93*cos(r_rad), cos(r_rad)], [0.93*sin(r_rad), sin(r_rad)], ...
                 'Color', [0.55 0.55 0.55], 'HandleVisibility', 'off');
        end
    end
    plot(S.ax_dial_arm, [0, 1], [0, 0], ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.1, 'HandleVisibility', 'off');
    S.h_arm_meas = plot(S.ax_dial_arm, [0, 1], [0, 0], 'b-o', 'LineWidth', 2.8, ...
                        'MarkerFaceColor', 'b', 'DisplayName', 'Brazo \theta Real');
    legend(S.ax_dial_arm, S.h_arm_meas, {'Brazo \theta Real'}, ...
           'Location', 'southoutside', 'Orientation', 'horizontal');

    % 3.2. Dial Cinemático Vertical del Péndulo alpha (°) [0° Abajo, 180° Arriba]
    S.ax_dial_pend = axes('Parent', fig, 'Units', 'normalized', ...
                          'Position', [0.540, 0.555, 0.200, 0.375]);
    hold(S.ax_dial_pend, 'on'); axis(S.ax_dial_pend, 'equal');
    xlim(S.ax_dial_pend, [-1.28 1.28]); ylim(S.ax_dial_pend, [-1.28 1.28]);
    grid(S.ax_dial_pend, 'on');
    title(S.ax_dial_pend, '2. Péndulo Vertical \alpha (°) [I2C2]', 'FontSize', 10, 'FontWeight', 'bold');
    plot(S.ax_dial_pend, cos(th_circ), sin(th_circ), 'k-', 'LineWidth', 1.4, 'HandleVisibility', 'off');
    cone_ang = linspace(pi/2 - deg2rad(20), pi/2 + deg2rad(20), 40);
    h_cone = patch(S.ax_dial_pend, [0, cos(cone_ang), 0], [0, sin(cone_ang), 0], [0.75 0.95 0.78], ...
                   'EdgeColor', [0.20 0.65 0.30], 'FaceAlpha', 0.45, 'DisplayName', 'Cono PID ±20°');
    text(S.ax_dial_pend, 0,  1.16, '180° (Arriba)', 'HorizontalAlignment', 'center', ...
         'FontSize', 8, 'FontWeight', 'bold', 'Color', [0.10 0.50 0.20]);
    text(S.ax_dial_pend, 0, -1.16, '0° (Abajo)', 'HorizontalAlignment', 'center', 'FontSize', 8);
    S.h_pend_bar = plot(S.ax_dial_pend, [0, 0], [0, -1], '-', 'Color', [0.85 0.38 0.05], ...
                        'LineWidth', 3.2, 'Marker', 'o', 'MarkerFaceColor', [0.85 0.38 0.05], ...
                        'DisplayName', 'Péndulo \alpha');
    legend(S.ax_dial_pend, [h_cone, S.h_pend_bar], {'Cono PID ±20°', 'Péndulo \alpha'}, ...
           'Location', 'southoutside', 'Orientation', 'horizontal');

    % 3.3. Gráfica Temporal: Posición Angular del Brazo theta(t)
    S.ax_pos = axes('Parent', fig, 'Units', 'normalized', ...
                    'Position', [0.770, 0.565, 0.212, 0.365]);
    hold(S.ax_pos, 'on'); grid(S.ax_pos, 'on');
    yline(S.ax_pos, 0, 'k:', 'HandleVisibility', 'off');
    S.h_line_theta = plot(S.ax_pos, S.t_hist, S.theta_hist, 'b-', 'LineWidth', 2.0, ...
                          'DisplayName', 'Ángulo Brazo \theta (°)');
    title(S.ax_pos, '3. Posición del Brazo \theta(t) [°]', 'FontSize', 10, 'FontWeight', 'bold');
    xlabel(S.ax_pos, 'Tiempo (s)'); ylabel(S.ax_pos, 'Ángulo Brazo \theta (°)');
    ylim(S.ax_pos, [-45, 45]);
    legend(S.ax_pos, S.h_line_theta, {'Ángulo Brazo \theta (°)'}, 'Location', 'northeast', 'FontSize', 8);

    % 3.4. Gráfica Temporal Inferior Izquierda: Dinámica del Péndulo alpha(t) y d(alpha)/dt
    S.ax_pend_time = axes('Parent', fig, 'Units', 'normalized', ...
                          'Position', [0.325, 0.078, 0.315, 0.385]);
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
    ylim(S.ax_pend_time, [-20, 20]);
    title(S.ax_pend_time, '4. Dinámica del Péndulo \alpha(t) y d\alpha/dt [I2C2 AS5600]', ...
          'FontSize', 10, 'FontWeight', 'bold');
    xlabel(S.ax_pend_time, 'Tiempo (s)');
    legend(S.ax_pend_time, [S.h_line_alpha, S.h_line_alphadot], ...
           {'Ángulo \alpha (°)', 'd\alpha/dt (rad/s)'}, 'Location', 'northeast', 'FontSize', 8);

    % 3.5. Gráfica Temporal Inferior Derecha: Esfuerzo de Control u(t) [V] y PWM (%)
    S.ax_pwm = axes('Parent', fig, 'Units', 'normalized', ...
                    'Position', [0.685, 0.078, 0.297, 0.385]);
    hold(S.ax_pwm, 'on'); grid(S.ax_pwm, 'on');
    yline(S.ax_pwm, 0, 'k:', 'HandleVisibility', 'off');
    S.h_line_pwm = plot(S.ax_pwm, S.t_hist, S.pwm_hist, '-', 'Color', [0.14 0.56 0.24], ...
                        'LineWidth', 1.8, 'DisplayName', 'PWM (%)');
    S.h_line_u   = plot(S.ax_pwm, S.t_hist, S.u_hist * (100/12), ':', 'Color', [0.10 0.32 0.72], ...
                        'LineWidth', 1.5, 'DisplayName', 'Tensión u(t) Escala %');
    title(S.ax_pwm, '5. Esfuerzo de Control en Puente H L298N (PWM % y u [V])', ...
          'FontSize', 10, 'FontWeight', 'bold');
    xlabel(S.ax_pwm, 'Tiempo (s)'); ylabel(S.ax_pwm, 'Ciclo Útil PWM (%)');
    ylim(S.ax_pwm, [-80, 80]);
    legend(S.ax_pwm, [S.h_line_pwm, S.h_line_u], {'PWM (%)', 'Tensión u(t) Escala %'}, ...
           'Location', 'northeast', 'FontSize', 8);

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
    %  CALLBACKS DE COMUNICACIÓN SERIAL, BOTÓN ÚNICO Y TELEMETRÍA
    %% ====================================================================
    function updateMasterButtonUI()
        if S.is_running
            set(S.btn_master, 'String', '■ DETENER Y VOLVER A 0°', ...
                              'BackgroundColor', [0.80 0.16 0.16]);
            set(S.lbl_mode_hint, 'String', sprintf(['Sistema ACTIVO en modo autónomo continuo:\n' ...
                                                    '• Equilibrio suave sin límite angular del brazo.\n' ...
                                                    '• Auto-recuperación desde 0° si cae el péndulo.\n' ...
                                                    '• Pulsa el botón rojo para detener en 0°.']));
        else
            set(S.btn_master, 'String', '▶ INICIAR PÉNDULO (SUBIR Y EQUILIBRAR)', ...
                              'BackgroundColor', [0.12 0.54 0.28]);
            set(S.lbl_mode_hint, 'String', sprintf(['Detenido en 0°. Al iniciar:\n' ...
                                                    '• Sube siempre desde 0° y estabiliza arriba.\n' ...
                                                    '• Sin límite de ángulo ante perturbaciones.\n' ...
                                                    '• Si cae, vuelve a 0° y reintenta solo.']));
        end
    end

    function onToggleMasterRun(~, ~)
        if ~S.is_connected
            warndlg('Conecta primero el puerto serial de la STM32.', 'Sin Conexión');
            return;
        end
        if ~S.is_running
            S.is_running = true;
            updateMasterButtonUI();
            queueSerialCommand('START');
        else
            S.is_running = false;
            updateMasterButtonUI();
            queueSerialCommand('STOPHOME');
        end
    end

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
                S.is_running   = false;
                S.t0_tic       = tic;

                set(S.btn_connect, 'String', 'DESCONECTAR', ...
                                   'BackgroundColor', [0.75 0.20 0.16]);
                set(S.lbl_status, 'String', sprintf('Serial: CONECTADO A %s | Modo: IDLE', sel_port), ...
                                  'ForegroundColor', [0.12 0.56 0.22]);
                updateMasterButtonUI();

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
                write(S.serial_obj, sprintf('\r\nSTOPHOME\r\n'), "char");
                pause(0.05);
            catch
            end
            delete(S.serial_obj);
        end
        S.serial_obj   = [];
        S.is_connected = false;
        S.is_running   = false;
        if ishandle(fig)
            set(S.btn_connect, 'String', 'CONECTAR', ...
                               'BackgroundColor', [0.14 0.60 0.27]);
            set(S.lbl_status, 'String', 'Serial: DESCONECTADO', ...
                              'ForegroundColor', [0.75 0.15 0.15]);
            updateMasterButtonUI();
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

    function onZeroEncoder(~, ~)
        S.is_running = false;
        updateMasterButtonUI();
        queueSerialCommand('ZERO');
    end

    function onExportCSV(~, ~)
        if isempty(S.log_matrix)
            warndlg('Aún no hay datos de telemetría capturados para exportar.', 'Sin Datos');
            return;
        end
        fname = sprintf('telemetria_furuta_%s.csv', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
        fid = fopen(fname, 'w');
        fprintf(fid, 't_ms,state,pwm_pct,u_volts,theta_rad,omega_rad_s,alpha_rad,alpha_err_rad,alpha_dot_rad_s\n');
        fclose(fid);
        writematrix(S.log_matrix, fname, 'WriteMode', 'append');
        msgbox(sprintf('Telemetría guardada exitosamente en:\n%s (%d muestras)', fname, size(S.log_matrix, 1)), ...
               'Exportación Exitosa');
    end

    function str_m = fsmStateName(st)
        names = { ...
            'DETENIDO EN 0° (IDLE)', ...
            'CALIBRANDO CERO (1)', ...
            'CARACTERIZACIÓN (2)', ...
            'PID VELOCIDAD (3)', ...
            'POSICIONANDO (4)', ...
            'LEVANTANDO PÉNDULO (SWING-UP)', ...
            'EQUILIBRIO INVERTIDO ACTIVO', ...
            'PARADA SEGURIDAD (7)', ...
            'AUTO-RECUPERANDO EN 0°', ...
            'DETENIENDO Y VOLVIENDO A 0°'};
        if st >= 0 && st <= 9
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
                            S.theta_hist    = [S.theta_hist(2:end), S.theta_deg];
                            S.alpha_hist    = [S.alpha_hist(2:end), S.alpha_deg];
                            S.alphadot_hist = [S.alphadot_hist(2:end), S.alpha_dot_rad_s];
                            S.pwm_hist      = [S.pwm_hist(2:end), S.pwm_pct];
                            S.u_hist        = [S.u_hist(2:end), S.u_volts];
                            new_rows        = [new_rows; vals.']; %#ok<AGROW>
                        end
                    end

                    if ~isempty(new_rows)
                        if size(S.log_matrix, 1) < 30000
                            S.log_matrix = [S.log_matrix; new_rows];
                        end
                    end

                    % 3. Actualizar textos de telemetría
                    st_str = fsmStateName(S.fsm_state);
                    set(S.lbl_status,   'String', sprintf('Serial: CONECTADO (t=%.1fs) | %s', S.t_ms/1000.0, st_str));
                    set(S.lbl_fsm_big,  'String', sprintf('ESTADO: %s', st_str));
                    if S.fsm_state == 6
                        set(S.lbl_fsm_big, 'ForegroundColor', [0.10 0.56 0.22]);
                    elseif S.fsm_state == 5 || S.fsm_state == 8
                        set(S.lbl_fsm_big, 'ForegroundColor', [0.82 0.44 0.04]);
                    else
                        set(S.lbl_fsm_big, 'ForegroundColor', [0.16 0.34 0.68]);
                    end

                    set(S.lbl_arm_pos,  'String', sprintf('Brazo θ (I2C1):         %+7.2f °', S.theta_deg));
                    set(S.lbl_arm_vel,  'String', sprintf('Velocidad Brazo ω:      %+6.2f rad/s', S.omega_rad_s));
                    set(S.lbl_pend_pos, 'String', sprintf('Péndulo α (I2C2):       %+7.2f °', S.alpha_deg));
                    set(S.lbl_pend_err, 'String', sprintf('Error Vertical α_err:   %+7.2f °', S.alpha_err_deg));
                    set(S.lbl_pend_vel, 'String', sprintf('Velocidad dα/dt:        %+6.2f rad/s', S.alpha_dot_rad_s));
                    set(S.lbl_pwm_val,  'String', sprintf('Motor L298N: u = %+5.2f V (%+5.1f%% PWM)', ...
                                                          S.u_volts, S.pwm_pct));

                    % 4. Actualizar Diales Circulares (Brazo y Péndulo)
                    r_meas = deg2rad(S.theta_deg);
                    set(S.h_arm_meas, 'XData', [0, cos(r_meas)], 'YData', [0, sin(r_meas)]);

                    r_pend_vis = -pi/2 + deg2rad(S.alpha_deg);
                    set(S.h_pend_bar, 'XData', [0, cos(r_pend_vis)], 'YData', [0, sin(r_pend_vis)]);

                    % 5. Actualizar Series Temporales
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

                    % Escalado automático suave del eje Y del brazo para ver bien tanto ±10° como perturbaciones grandes
                    th_span = max(45.0, ceil((max(abs(S.theta_hist)) + 15.0) / 15.0) * 15.0);
                    ylim(S.ax_pos, [-th_span, th_span]);

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
