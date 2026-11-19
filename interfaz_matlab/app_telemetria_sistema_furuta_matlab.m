function fig_out = app_telemetria_sistema_furuta_matlab(auto_connect)
%% ========================================================================
%  INTERFAZ DE CONTROL AUTÓNOMO Y TELEMETRÍA — PÉNDULO DE FURUTA
%  STM32 Nucleo-F446RE (180 MHz, Ts = 1 ms) + L298N (20 kHz) + 2x AS5600
%  Proyecto Final — Control Digital (Universidad EAFIT)
%% ========================================================================

    if nargin < 1
        auto_connect = true;
    end

    % Limpiar timers previos; liberar puertos solo si vamos a conectar
    try
        old_timers = timerfindall('Tag', 'FurutaSystemGUITimer');
        if ~isempty(old_timers), stop(old_timers); delete(old_timers); end
        old_timers2 = timerfindall('Tag', 'FurutaMotorGUITimer');
        if ~isempty(old_timers2), stop(old_timers2); delete(old_timers2); end
    catch
    end
    if auto_connect
        try
            open_ports = serialportfind();
            if ~isempty(open_ports), delete(open_ports); end
        catch
        end
    end

    %% 1. Estado Interno
    S = struct();
    S.serial_obj      = [];
    S.is_connected    = false;
    S.is_io_busy      = false;
    S.is_running      = false;
    S.rx_partial_buf  = '';
    S.pending_cmd     = '';
    S.cmd_burst_left  = 0;

    S.t_ms            = 0;
    S.fsm_state       = 0;
    S.theta_deg       = 0.0;
    S.omega_rad_s     = 0.0;
    S.alpha_deg       = 0.0;
    S.alpha_err_deg   = 180.0;
    S.alpha_dot_rad_s = 0.0;
    S.u_volts         = 0.0;
    S.pwm_pct         = 0.0;

    S.history_len     = 400;
    S.t_hist          = linspace(-5, 0, S.history_len);
    S.theta_hist      = zeros(1, S.history_len);
    S.alpha_hist      = zeros(1, S.history_len);
    S.alphadot_hist   = zeros(1, S.history_len);
    S.pwm_hist        = zeros(1, S.history_len);
    S.u_hist          = zeros(1, S.history_len);
    S.log_matrix      = [];
    S.t0_tic          = tic;

    % Puertos seriales disponibles
    live_ports = cellstr(serialportlist("available"));
    live_ports = live_ports(~cellfun(@isempty, live_ports));
    if isempty(live_ports)
        avail_ports = {'COM13', 'COM3', 'COM4', 'COM5'};
    elseif ~ismember('COM13', live_ports)
        avail_ports = [{'COM13'}, reshape(live_ports, 1, [])];
    else
        avail_ports = reshape(live_ports, 1, []);
    end

    %% 2. Paleta Limpia y Ventana Principal Maximizada
    C_BG     = [0.945 0.957 0.976];
    C_CARD   = [1.000 1.000 1.000];
    C_TILE   = [0.965 0.974 0.988];
    C_ROWALT = [0.968 0.977 0.990];
    C_BORDER = [0.820 0.855 0.905];
    C_INK    = [0.100 0.145 0.220];
    C_MUTED  = [0.420 0.480 0.570];
    C_BLUE   = [0.080 0.390 0.780];
    C_ORANGE = [0.855 0.365 0.055];
    C_PURPLE = [0.480 0.180 0.680];
    C_GREEN  = [0.090 0.545 0.275];
    C_RED    = [0.795 0.160 0.180];
    C_GOLD   = [0.835 0.520 0.065];
    C_TEAL   = [0.155 0.470 0.615];

    fig = figure('Name', 'Péndulo de Furuta — Control Digital (STM32F446RE)', ...
                 'NumberTitle', 'off', ...
                 'MenuBar', 'none', ...
                 'ToolBar', 'none', ...
                 'Color', C_BG, ...
                 'Units', 'normalized', ...
                 'OuterPosition', [0 0 1 1], ...
                 'WindowState', 'maximized', ...
                 'CloseRequestFcn', @onCloseApp);

    %% ====================================================================
    %  COLUMNA IZQUIERDA: CONTROL, TELEMETRÍA Y CONSTANTES (SIN TEXTO EXTRA)
    %% ====================================================================

    % --- Tarjeta 1: Conexión y Control Maestro ---
    pnl_ctrl = uipanel('Parent', fig, ...
                       'Title', ' Control STM32 ', ...
                       'FontName', 'Segoe UI', 'FontSize', 9.5, 'FontWeight', 'bold', ...
                       'ForegroundColor', C_INK, ...
                       'BackgroundColor', C_CARD, ...
                       'BorderType', 'line', 'HighlightColor', C_BORDER, ...
                       'Units', 'normalized', ...
                       'Position', [0.010, 0.785, 0.270, 0.197]);

    idx_com13 = find(strcmpi(avail_ports, 'COM13'), 1);
    if isempty(idx_com13), idx_com13 = 1; end

    S.popup_port = uicontrol('Parent', pnl_ctrl, 'Style', 'popupmenu', ...
                             'String', avail_ports, 'Value', idx_com13, ...
                             'FontName', 'Segoe UI', 'FontSize', 9.5, ...
                             'BackgroundColor', C_CARD, ...
                             'Units', 'normalized', 'Position', [0.035, 0.695, 0.275, 0.210]);

    S.btn_connect = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
                              'String', 'CONECTAR', ...
                              'FontName', 'Segoe UI', 'FontSize', 9.0, 'FontWeight', 'bold', ...
                              'BackgroundColor', C_GREEN, 'ForegroundColor', [1 1 1], ...
                              'Units', 'normalized', 'Position', [0.330, 0.680, 0.370, 0.235], ...
                              'Callback', @onToggleConnect);

    S.lbl_conn = uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
                           'String', '● OFF', ...
                           'FontName', 'Segoe UI', 'FontSize', 9.5, 'FontWeight', 'bold', ...
                           'ForegroundColor', C_RED, 'BackgroundColor', C_CARD, ...
                           'HorizontalAlignment', 'center', ...
                           'Units', 'normalized', 'Position', [0.720, 0.700, 0.245, 0.185]);

    S.btn_master = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
                             'String', '▶  INICIAR', ...
                             'FontName', 'Segoe UI', 'FontSize', 11.5, 'FontWeight', 'bold', ...
                             'BackgroundColor', C_GREEN, 'ForegroundColor', [1 1 1], ...
                             'Units', 'normalized', 'Position', [0.035, 0.315, 0.930, 0.315], ...
                             'Callback', @onToggleMasterRun);

    uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
              'String', 'CERO (0°)', ...
              'FontName', 'Segoe UI', 'FontSize', 9.0, 'FontWeight', 'bold', ...
              'BackgroundColor', C_GOLD, 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.035, 0.060, 0.450, 0.205], ...
              'Callback', @onZeroEncoder);

    uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
              'String', 'EXPORTAR CSV', ...
              'FontName', 'Segoe UI', 'FontSize', 9.0, 'FontWeight', 'bold', ...
              'BackgroundColor', C_TEAL, 'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.515, 0.060, 0.450, 0.205], ...
              'Callback', @onExportCSV);

    % --- Tarjeta 2: Telemetría en Tiempo Real (KPIs Compactos) ---
    pnl_tel = uipanel('Parent', fig, ...
                      'Title', ' Telemetría ', ...
                      'FontName', 'Segoe UI', 'FontSize', 9.5, 'FontWeight', 'bold', ...
                      'ForegroundColor', C_INK, ...
                      'BackgroundColor', C_CARD, ...
                      'BorderType', 'line', 'HighlightColor', C_BORDER, ...
                      'Units', 'normalized', ...
                      'Position', [0.010, 0.505, 0.270, 0.266]);

    S.lbl_fsm_big = uicontrol('Parent', pnl_tel, 'Style', 'text', ...
                              'String', 'IDLE', ...
                              'FontName', 'Segoe UI', 'FontSize', 10.5, 'FontWeight', 'bold', ...
                              'ForegroundColor', C_BLUE, 'BackgroundColor', C_CARD, ...
                              'HorizontalAlignment', 'left', ...
                              'Units', 'normalized', 'Position', [0.035, 0.805, 0.660, 0.145]);

    S.lbl_time = uicontrol('Parent', pnl_tel, 'Style', 'text', ...
                           'String', 't = 0.0 s', ...
                           'FontName', 'Consolas', 'FontSize', 9.2, 'FontWeight', 'bold', ...
                           'ForegroundColor', C_MUTED, 'BackgroundColor', C_CARD, ...
                           'HorizontalAlignment', 'right', ...
                           'Units', 'normalized', 'Position', [0.710, 0.815, 0.255, 0.130]);

    % Cuadrícula 3x2 de KPIs sin ruido visual
    S.lbl_arm_pos  = makeKpiTile(pnl_tel, [0.035, 0.545, 0.452, 0.230], 'θ Brazo',       '+0.00 °',       C_BLUE);
    S.lbl_arm_vel  = makeKpiTile(pnl_tel, [0.513, 0.545, 0.452, 0.230], 'ω Brazo',       '+0.00 rad/s',   C_INK);
    S.lbl_pend_pos = makeKpiTile(pnl_tel, [0.035, 0.292, 0.452, 0.230], 'α Péndulo',     '+0.00 °',       C_ORANGE);
    S.lbl_pend_err = makeKpiTile(pnl_tel, [0.513, 0.292, 0.452, 0.230], 'α_err Vertical','+180.00 °',     C_PURPLE);
    S.lbl_pend_vel = makeKpiTile(pnl_tel, [0.035, 0.040, 0.452, 0.230], 'dα/dt',         '+0.00 rad/s',   C_INK);
    S.lbl_pwm_val  = makeKpiTile(pnl_tel, [0.513, 0.040, 0.452, 0.230], 'u · PWM',       '+0.0V · +0%',   C_GREEN);

    % --- Tarjeta 3: Constantes de Controladores y Filtros (Firmware STM32) ---
    pnl_params = uipanel('Parent', fig, ...
                         'Title', ' Controladores y Filtros (1 kHz) ', ...
                         'FontName', 'Segoe UI', 'FontSize', 9.5, 'FontWeight', 'bold', ...
                         'ForegroundColor', C_INK, ...
                         'BackgroundColor', C_CARD, ...
                         'BorderType', 'line', 'HighlightColor', C_BORDER, ...
                         'Units', 'normalized', ...
                         'Position', [0.010, 0.018, 0.270, 0.473]);

    % Sección A: Controladores (Modelo M4 Ultra-Silk, R² = 98.51%)
    makeSectionHeader(pnl_params, 0.908, 'CONTROLADORES   (Ts = 1 ms · M4 Ultra-Silk)');
    ctrl_rows = { ...
        'PD α (Péndulo)', 'Kp  14.50  Ki  0.00  Kd  1.50'; ...
        'PID θ (Brazo)',  'Kp  -1.45  Ki -0.75  Kd -0.85'; ...
        'PID Servo 0°',   'Kp  13.50  Ki 65.00  Kd  1.45'; ...
        'PID Vel. Motor', 'Kp   1.42  Ki 27.32  Kd 0.005'; ...
        'Swing-Up PD',    'Kp  13.50  Kd  1.20  ωn  8.85' ...
    };
    for idx = 1:size(ctrl_rows, 1)
        y_pos = 0.838 - (idx - 1) * 0.072;
        bg_c = C_CARD;
        if mod(idx, 2) == 1, bg_c = C_ROWALT; end
        makeParamRow(pnl_params, y_pos, ctrl_rows{idx, 1}, ctrl_rows{idx, 2}, bg_c);
    end

    % Sección B: Filtros y Actuador
    makeSectionHeader(pnl_params, 0.468, 'FILTROS Y ACTUADOR   (PWM 20 kHz)');
    filt_rows = { ...
        'LPF Vel. (×2)',  'IIR τ = 6 ms    | fc ≈ 15.8 Hz'; ...
        'Notch 1 (Fijo)', 'f0 = 11.0 Hz    | Q = 1.2'; ...
        'Notch 2 (Adap)', 'f0 = 6.2 Hz     | Q = 1.4 (-6 dB)'; ...
        'Slew / Sat.',    '0.065–0.40 V/ms | ±6.5–8.5 V'; ...
        'L298N DZ',       '+9.2% / -16.0%  | Vk = 0.65 V' ...
    };
    for idx = 1:size(filt_rows, 1)
        y_pos = 0.398 - (idx - 1) * 0.072;
        bg_c = C_CARD;
        if mod(idx, 2) == 1, bg_c = C_ROWALT; end
        makeParamRow(pnl_params, y_pos, filt_rows{idx, 1}, filt_rows{idx, 2}, bg_c);
    end

    % Pie compacto de parámetros identificados de planta
    uicontrol('Parent', pnl_params, 'Style', 'text', ...
              'String', 'Planta R²=98.5%: Km=3.37 · τm=0.118s · Kac=0.874 · ζ=0.065', ...
              'FontName', 'Consolas', 'FontSize', 7.8, 'FontWeight', 'bold', ...
              'ForegroundColor', C_MUTED, 'BackgroundColor', C_TILE, ...
              'HorizontalAlignment', 'center', ...
              'Units', 'normalized', 'Position', [0.025, 0.022, 0.950, 0.048]);

    %% ====================================================================
    %  ÁREA DERECHA: 5 GRÁFICAS CIENTÍFICAS LIMPIAS SIN SUPERPOSICIONES
    %% ====================================================================
    th_circ = linspace(0, 2*pi, 180);

    % 3.1. Dial Polar del Brazo θ (°)
    S.ax_dial_arm = axes('Parent', fig, 'Units', 'normalized', ...
                         'Position', [0.295, 0.545, 0.205, 0.395], ...
                         'Color', C_BG, 'XColor', 'none', 'YColor', 'none');
    hold(S.ax_dial_arm, 'on'); axis(S.ax_dial_arm, 'equal');
    xlim(S.ax_dial_arm, [-1.30 1.30]); ylim(S.ax_dial_arm, [-1.30 1.30]);
    patch(S.ax_dial_arm, cos(th_circ), sin(th_circ), C_CARD, ...
          'EdgeColor', [0.30 0.36 0.45], 'LineWidth', 1.6, 'HandleVisibility', 'off');
    plot(S.ax_dial_arm, [-1 1], [0 0], ':', 'Color', [0.80 0.84 0.90], 'LineWidth', 0.9, 'HandleVisibility', 'off');
    plot(S.ax_dial_arm, [0 0], [-1 1], ':', 'Color', [0.80 0.84 0.90], 'LineWidth', 0.9, 'HandleVisibility', 'off');
    for ang_tick = -165:15:180
        r_rad = deg2rad(ang_tick);
        if mod(ang_tick, 90) == 0
            plot(S.ax_dial_arm, [0.85*cos(r_rad), cos(r_rad)], [0.85*sin(r_rad), sin(r_rad)], ...
                 'Color', C_INK, 'LineWidth', 1.4, 'HandleVisibility', 'off');
            text(S.ax_dial_arm, 1.15*cos(r_rad), 1.15*sin(r_rad), sprintf('%d°', ang_tick), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                 'FontName', 'Segoe UI', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', C_INK);
        elseif mod(ang_tick, 45) == 0
            plot(S.ax_dial_arm, [0.89*cos(r_rad), cos(r_rad)], [0.89*sin(r_rad), sin(r_rad)], ...
                 'Color', C_MUTED, 'LineWidth', 1.0, 'HandleVisibility', 'off');
            text(S.ax_dial_arm, 1.16*cos(r_rad), 1.16*sin(r_rad), sprintf('%d°', ang_tick), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                 'FontName', 'Segoe UI', 'FontSize', 7.5, 'Color', C_MUTED);
        else
            plot(S.ax_dial_arm, [0.93*cos(r_rad), cos(r_rad)], [0.93*sin(r_rad), sin(r_rad)], ...
                 'Color', [0.70 0.74 0.80], 'LineWidth', 0.8, 'HandleVisibility', 'off');
        end
    end
    S.h_arm_meas = plot(S.ax_dial_arm, [0, 1], [0, 0], '-o', 'Color', C_BLUE, ...
                        'LineWidth', 3.2, 'MarkerSize', 7, 'MarkerFaceColor', C_BLUE);
    plot(S.ax_dial_arm, 0, 0, 'o', 'Color', C_INK, 'MarkerSize', 5, 'MarkerFaceColor', C_INK, ...
         'HandleVisibility', 'off');
    title(S.ax_dial_arm, 'Brazo \theta', 'FontName', 'Segoe UI', 'FontSize', 10.5, ...
          'FontWeight', 'bold', 'Color', C_INK);

    % 3.2. Dial Vertical del Péndulo α (°)
    S.ax_dial_pend = axes('Parent', fig, 'Units', 'normalized', ...
                          'Position', [0.515, 0.545, 0.205, 0.395], ...
                          'Color', C_BG, 'XColor', 'none', 'YColor', 'none');
    hold(S.ax_dial_pend, 'on'); axis(S.ax_dial_pend, 'equal');
    xlim(S.ax_dial_pend, [-1.34 1.34]); ylim(S.ax_dial_pend, [-1.34 1.34]);
    patch(S.ax_dial_pend, cos(th_circ), sin(th_circ), C_CARD, ...
          'EdgeColor', [0.30 0.36 0.45], 'LineWidth', 1.6, 'HandleVisibility', 'off');
    plot(S.ax_dial_pend, [-1 1], [0 0], ':', 'Color', [0.80 0.84 0.90], 'LineWidth', 0.9, 'HandleVisibility', 'off');
    plot(S.ax_dial_pend, [0 0], [-1 1], ':', 'Color', [0.80 0.84 0.90], 'LineWidth', 0.9, 'HandleVisibility', 'off');
    cone_ang = linspace(pi/2 - deg2rad(20), pi/2 + deg2rad(20), 40);
    patch(S.ax_dial_pend, [0, cos(cone_ang), 0], [0, sin(cone_ang), 0], [0.76 0.94 0.80], ...
          'EdgeColor', C_GREEN, 'LineWidth', 1.0, 'FaceAlpha', 0.50, 'HandleVisibility', 'off');
    text(S.ax_dial_pend, 0,  1.17, '180° (±20°)', 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', 'FontName', 'Segoe UI', 'FontSize', 8.5, ...
         'FontWeight', 'bold', 'Color', C_GREEN);
    text(S.ax_dial_pend, 0, -1.17, '0°', 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', 'FontName', 'Segoe UI', 'FontSize', 8.5, ...
         'FontWeight', 'bold', 'Color', C_MUTED);
    text(S.ax_dial_pend,  1.19, 0, '+90°', 'HorizontalAlignment', 'center', ...
         'FontName', 'Segoe UI', 'FontSize', 7.5, 'Color', C_MUTED);
    text(S.ax_dial_pend, -1.19, 0, '-90°', 'HorizontalAlignment', 'center', ...
         'FontName', 'Segoe UI', 'FontSize', 7.5, 'Color', C_MUTED);
    S.h_pend_bar = plot(S.ax_dial_pend, [0, 0], [0, -1], '-o', 'Color', C_ORANGE, ...
                        'LineWidth', 3.4, 'MarkerSize', 8, 'MarkerFaceColor', C_ORANGE);
    plot(S.ax_dial_pend, 0, 0, 'o', 'Color', C_INK, 'MarkerSize', 5, 'MarkerFaceColor', C_INK, ...
         'HandleVisibility', 'off');
    title(S.ax_dial_pend, 'Péndulo \alpha', 'FontName', 'Segoe UI', 'FontSize', 10.5, ...
          'FontWeight', 'bold', 'Color', C_INK);

    % 3.3. Serie Temporal: Brazo θ(t)
    S.ax_pos = axes('Parent', fig, 'Units', 'normalized', ...
                    'Position', [0.765, 0.575, 0.220, 0.365]);
    styleTimeAxes(S.ax_pos);
    hold(S.ax_pos, 'on');
    yline(S.ax_pos, 0, ':', 'Color', [0.55 0.60 0.68], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    S.h_line_theta = plot(S.ax_pos, S.t_hist, S.theta_hist, '-', 'Color', C_BLUE, 'LineWidth', 2.0);
    title(S.ax_pos, 'Brazo \theta(t)', 'FontName', 'Segoe UI', 'FontSize', 10.5, ...
          'FontWeight', 'bold', 'Color', C_INK);
    xlabel(S.ax_pos, 't (s)', 'FontName', 'Segoe UI', 'FontSize', 9.0, 'Color', C_MUTED);
    ylabel(S.ax_pos, '\theta (°)', 'FontName', 'Segoe UI', 'FontSize', 9.0, 'Color', C_BLUE);
    ylim(S.ax_pos, [-45, 45]);

    % 3.4. Serie Temporal: Péndulo α(t) y dα/dt
    S.ax_pend_time = axes('Parent', fig, 'Units', 'normalized', ...
                          'Position', [0.328, 0.082, 0.300, 0.375]);
    styleTimeAxes(S.ax_pend_time);
    hold(S.ax_pend_time, 'on');
    yyaxis(S.ax_pend_time, 'left');
    S.h_line_alpha = plot(S.ax_pend_time, S.t_hist, S.alpha_hist, '-', 'Color', C_ORANGE, ...
                          'LineWidth', 2.0, 'DisplayName', '\alpha (°)');
    ylabel(S.ax_pend_time, '\alpha (°)', 'FontName', 'Segoe UI', 'FontSize', 9.0);
    ylim(S.ax_pend_time, [-190, 190]);
    S.ax_pend_time.YAxis(1).Color = C_ORANGE;

    yyaxis(S.ax_pend_time, 'right');
    S.h_line_alphadot = plot(S.ax_pend_time, S.t_hist, S.alphadot_hist, '--', 'Color', C_PURPLE, ...
                             'LineWidth', 1.4, 'DisplayName', 'd\alpha/dt');
    ylabel(S.ax_pend_time, 'd\alpha/dt (rad/s)', 'FontName', 'Segoe UI', 'FontSize', 9.0);
    ylim(S.ax_pend_time, [-20, 20]);
    S.ax_pend_time.YAxis(2).Color = C_PURPLE;

    title(S.ax_pend_time, 'Péndulo \alpha(t) y d\alpha/dt', 'FontName', 'Segoe UI', ...
          'FontSize', 10.5, 'FontWeight', 'bold', 'Color', C_INK);
    xlabel(S.ax_pend_time, 't (s)', 'FontName', 'Segoe UI', 'FontSize', 9.0, 'Color', C_MUTED);
    legend(S.ax_pend_time, [S.h_line_alpha, S.h_line_alphadot], {'\alpha (°)', 'd\alpha/dt'}, ...
           'Location', 'northeast', 'FontName', 'Segoe UI', 'FontSize', 8.5, 'Box', 'on', ...
           'Color', C_CARD, 'EdgeColor', C_BORDER);

    % 3.5. Serie Temporal: Control L298N (PWM y Tensión u)
    S.ax_pwm = axes('Parent', fig, 'Units', 'normalized', ...
                    'Position', [0.698, 0.082, 0.287, 0.375]);
    styleTimeAxes(S.ax_pwm);
    hold(S.ax_pwm, 'on');
    yline(S.ax_pwm, 0, ':', 'Color', [0.55 0.60 0.68], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    S.h_line_pwm = plot(S.ax_pwm, S.t_hist, S.pwm_hist, '-', 'Color', C_GREEN, ...
                        'LineWidth', 1.9, 'DisplayName', 'PWM (%)');
    S.h_line_u   = plot(S.ax_pwm, S.t_hist, S.u_hist * (100/12), ':', 'Color', C_BLUE, ...
                        'LineWidth', 1.6, 'DisplayName', 'u (% Vmax)');
    title(S.ax_pwm, 'Control L298N', 'FontName', 'Segoe UI', 'FontSize', 10.5, ...
          'FontWeight', 'bold', 'Color', C_INK);
    xlabel(S.ax_pwm, 't (s)', 'FontName', 'Segoe UI', 'FontSize', 9.0, 'Color', C_MUTED);
    ylabel(S.ax_pwm, 'PWM / u (%)', 'FontName', 'Segoe UI', 'FontSize', 9.0, 'Color', C_INK);
    ylim(S.ax_pwm, [-80, 80]);
    legend(S.ax_pwm, [S.h_line_pwm, S.h_line_u], {'PWM (%)', 'u (% Vmax)'}, ...
           'Location', 'northeast', 'FontName', 'Segoe UI', 'FontSize', 8.5, 'Box', 'on', ...
           'Color', C_CARD, 'EdgeColor', C_BORDER);

    % Forzar maximización tras construir los widgets
    drawnow;
    try
        fig.WindowState = 'maximized';
    catch
    end

    % Timer de refresco en tiempo real (30 Hz)
    S.gui_timer = timer('Tag', 'FurutaSystemGUITimer', ...
                        'ExecutionMode', 'fixedRate', ...
                        'Period', 0.033, ...
                        'BusyMode', 'drop', ...
                        'TimerFcn', @onTimerUpdateGUI);

    % Auto-conectar silenciosamente solo si COM13 está conectado
    if auto_connect && ismember('COM13', live_ports)
        try
            connectToSelectedPort(false);
        catch
        end
    end

    if nargout > 0
        fig_out = fig;
    end

    %% ====================================================================
    %  HELPERS DE DISEÑO LIMPIO
    %% ====================================================================
    function h_val = makeKpiTile(parent_pnl, pos, lbl_str, init_val, val_color)
        tile = uipanel('Parent', parent_pnl, ...
                       'BackgroundColor', C_TILE, ...
                       'BorderType', 'line', 'HighlightColor', [0.87 0.90 0.94], ...
                       'Units', 'normalized', 'Position', pos);
        uicontrol('Parent', tile, 'Style', 'text', ...
                  'String', lbl_str, ...
                  'FontName', 'Segoe UI', 'FontSize', 8.2, 'FontWeight', 'bold', ...
                  'ForegroundColor', C_MUTED, 'BackgroundColor', C_TILE, ...
                  'HorizontalAlignment', 'left', ...
                  'Units', 'normalized', 'Position', [0.06, 0.53, 0.88, 0.36]);
        h_val = uicontrol('Parent', tile, 'Style', 'text', ...
                          'String', init_val, ...
                          'FontName', 'Consolas', 'FontSize', 10.6, 'FontWeight', 'bold', ...
                          'ForegroundColor', val_color, 'BackgroundColor', C_TILE, ...
                          'HorizontalAlignment', 'left', ...
                          'Units', 'normalized', 'Position', [0.06, 0.08, 0.90, 0.48]);
    end

    function makeSectionHeader(parent_pnl, y_norm, title_str)
        uicontrol('Parent', parent_pnl, 'Style', 'text', ...
                  'String', title_str, ...
                  'FontName', 'Segoe UI', 'FontSize', 8.4, 'FontWeight', 'bold', ...
                  'ForegroundColor', C_BLUE, 'BackgroundColor', C_CARD, ...
                  'HorizontalAlignment', 'left', ...
                  'Units', 'normalized', 'Position', [0.030, y_norm, 0.940, 0.052]);
    end

    function makeParamRow(parent_pnl, y_norm, name_str, val_str, bg_color)
        uicontrol('Parent', parent_pnl, 'Style', 'text', ...
                  'String', [' ' name_str], ...
                  'FontName', 'Segoe UI', 'FontSize', 8.6, 'FontWeight', 'bold', ...
                  'ForegroundColor', C_INK, 'BackgroundColor', bg_color, ...
                  'HorizontalAlignment', 'left', ...
                  'Units', 'normalized', 'Position', [0.025, y_norm, 0.305, 0.056]);
        uicontrol('Parent', parent_pnl, 'Style', 'text', ...
                  'String', val_str, ...
                  'FontName', 'Consolas', 'FontSize', 8.4, ...
                  'ForegroundColor', C_INK, 'BackgroundColor', bg_color, ...
                  'HorizontalAlignment', 'left', ...
                  'Units', 'normalized', 'Position', [0.330, y_norm, 0.645, 0.056]);
    end

    function styleTimeAxes(ax)
        set(ax, 'Color', C_CARD, ...
                'Box', 'on', ...
                'XGrid', 'on', 'YGrid', 'on', ...
                'GridColor', [0.75 0.80 0.87], 'GridAlpha', 0.45, ...
                'XColor', [0.28 0.34 0.42], 'YColor', [0.28 0.34 0.42], ...
                'FontName', 'Segoe UI', 'FontSize', 8.5, 'LineWidth', 0.9);
    end

    %% ====================================================================
    %  CALLBACKS DE COMUNICACIÓN SERIAL, CONTROL Y TELEMETRÍA
    %% ====================================================================
    function updateMasterButtonUI()
        if S.is_running
            set(S.btn_master, 'String', '■  DETENER (0°)', 'BackgroundColor', C_RED);
        else
            set(S.btn_master, 'String', '▶  INICIAR', 'BackgroundColor', C_GREEN);
        end
    end

    function onToggleMasterRun(~, ~)
        if ~S.is_connected
            warndlg('Conecta el puerto serial primero.', 'Sin Conexión');
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
            connectToSelectedPort(true);
        else
            disconnectSerial();
        end
    end

    function connectToSelectedPort(show_err_dialog)
        ports = get(S.popup_port, 'String');
        sel_port = strtrim(ports{get(S.popup_port, 'Value')});
        try
            S.serial_obj = serialport(sel_port, 115200, 'Timeout', 0.05);
            configureTerminator(S.serial_obj, "CR/LF");
            flush(S.serial_obj);
            S.rx_partial_buf = '';
            S.is_connected   = true;
            S.is_running     = false;
            S.t0_tic         = tic;

            set(S.btn_connect, 'String', 'DESCONECTAR', 'BackgroundColor', C_RED);
            set(S.lbl_conn, 'String', '● ON', 'ForegroundColor', C_GREEN);
            updateMasterButtonUI();

            start(S.gui_timer);
        catch ME
            if show_err_dialog
                errordlg(sprintf('No se pudo abrir %s:\n%s', sel_port, ME.message), ...
                         'Error Serial');
            end
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
            set(S.btn_connect, 'String', 'CONECTAR', 'BackgroundColor', C_GREEN);
            set(S.lbl_conn, 'String', '● OFF', 'ForegroundColor', C_RED);
            set(S.lbl_fsm_big, 'String', 'IDLE', 'ForegroundColor', C_BLUE);
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
            warndlg('No hay datos capturados.', 'Sin Datos');
            return;
        end
        base_dir = fileparts(fileparts(mfilename('fullpath')));
        data_dir = fullfile(base_dir, 'caracterizacion_y_validacion', 'datos_experimentales');
        if ~isfolder(data_dir)
            data_dir = pwd;
        end
        fname = fullfile(data_dir, sprintf('telemetria_furuta_%s.csv', ...
                         char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))));
        fid = fopen(fname, 'w');
        fprintf(fid, 't_ms,state,pwm_pct,u_volts,theta_rad,omega_rad_s,alpha_rad,alpha_err_rad,alpha_dot_rad_s\n');
        fclose(fid);
        writematrix(S.log_matrix, fname, 'WriteMode', 'append');
        msgbox(sprintf('Guardado (%d muestras):\n%s', size(S.log_matrix, 1), fname), 'CSV');
    end

    function str_m = fsmStateName(st)
        names = { ...
            'IDLE', ...
            'CALIBRANDO', ...
            'CARACTERIZACIÓN', ...
            'PID VELOCIDAD', ...
            'SERVO POSICIÓN', ...
            'SWING-UP', ...
            'EQUILIBRIO ACTIVO', ...
            'PARADA SEGURIDAD', ...
            'RECUPERANDO 0°', ...
            'DETENIENDO 0°'};
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
            if S.cmd_burst_left > 0 && ~isempty(S.pending_cmd)
                write(S.serial_obj, sprintf('\r\n%s\r\n', S.pending_cmd), "char");
                S.cmd_burst_left = S.cmd_burst_left - 1;
            end

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

                    if ~isempty(new_rows) && size(S.log_matrix, 1) < 30000
                        S.log_matrix = [S.log_matrix; new_rows];
                    end

                    st_str = fsmStateName(S.fsm_state);
                    set(S.lbl_fsm_big, 'String', st_str);
                    set(S.lbl_time,    'String', sprintf('t = %.1f s', S.t_ms / 1000.0));
                    if S.fsm_state == 6
                        set(S.lbl_fsm_big, 'ForegroundColor', C_GREEN);
                    elseif S.fsm_state == 5 || S.fsm_state == 8
                        set(S.lbl_fsm_big, 'ForegroundColor', C_ORANGE);
                    else
                        set(S.lbl_fsm_big, 'ForegroundColor', C_BLUE);
                    end

                    set(S.lbl_arm_pos,  'String', sprintf('%+6.2f °', S.theta_deg));
                    set(S.lbl_arm_vel,  'String', sprintf('%+5.2f rad/s', S.omega_rad_s));
                    set(S.lbl_pend_pos, 'String', sprintf('%+6.2f °', S.alpha_deg));
                    set(S.lbl_pend_err, 'String', sprintf('%+6.2f °', S.alpha_err_deg));
                    set(S.lbl_pend_vel, 'String', sprintf('%+5.2f rad/s', S.alpha_dot_rad_s));
                    set(S.lbl_pwm_val,  'String', sprintf('%+4.1fV · %+4.0f%%', S.u_volts, S.pwm_pct));

                    r_meas = deg2rad(S.theta_deg);
                    set(S.h_arm_meas, 'XData', [0, cos(r_meas)], 'YData', [0, sin(r_meas)]);

                    r_pend_vis = -pi/2 + deg2rad(S.alpha_deg);
                    set(S.h_pend_bar, 'XData', [0, cos(r_pend_vis)], 'YData', [0, sin(r_pend_vis)]);

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
