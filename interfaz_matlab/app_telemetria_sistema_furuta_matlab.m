function fig_out = app_telemetria_sistema_furuta_matlab(auto_connect)
%% ========================================================================
%  INTERFAZ DE CONTROL AUTÓNOMO, TELEMETRÍA Y GEMELO DIGITAL 3D
%  PÉNDULO INVERTIDO ROTACIONAL DE FURUTA
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

    %% 1. Estado Interno y Geometría Física Real (mm — CAD FreeCAD 1.1)
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

    % Cotas cinemáticas exactas del ensamble impreso en 3D (mm)
    S.L0_mm           = 90.45;   % Centro eje motor -> centro barra péndulo
    S.Z_PIVOT_mm      = 182.80;  % Altura pivote rodamiento 608ZZ
    S.L1_mm           = 152.80;  % Pivote -> punta barra (30 mm despeje mesa)

    % Historial temporal (series 2D) y rastro 3D de la punta del péndulo
    S.history_len     = 400;
    S.t_hist          = linspace(-5, 0, S.history_len);
    S.theta_hist      = zeros(1, S.history_len);
    S.alpha_hist      = zeros(1, S.history_len);
    S.alphadot_hist   = zeros(1, S.history_len);
    S.pwm_hist        = zeros(1, S.history_len);
    S.u_hist          = zeros(1, S.history_len);
    S.log_matrix      = [];
    S.t0_tic          = tic;

    S.trail_len       = 90;
    S.trail_enabled   = true;
    S.trail_x         = nan(1, S.trail_len);
    S.trail_y         = nan(1, S.trail_len);
    S.trail_z         = nan(1, S.trail_len);
    S.last_bal_state  = -1;

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

    %% 2. Paleta Limpia y Ventana Principal Maximizada (Bento 3 Columnas)
    C_BG     = [0.945 0.957 0.976];
    C_CARD   = [1.000 1.000 1.000];
    C_TILE   = [0.965 0.974 0.988];
    C_ROWALT = [0.968 0.977 0.990];
    C_BORDER = [0.820 0.855 0.905];
    C_INK    = [0.100 0.145 0.220];
    C_MUTED  = [0.420 0.480 0.570];
    C_BLUE   = [0.080 0.390 0.780];
    C_CYAN   = [0.020 0.620 0.840];
    C_ORANGE = [0.910 0.340 0.050];
    C_PURPLE = [0.540 0.170 0.720];
    C_GREEN  = [0.090 0.545 0.275];
    C_RED    = [0.795 0.160 0.180];
    C_GOLD   = [0.835 0.520 0.065];
    C_TEAL   = [0.155 0.470 0.615];

    fig = figure('Name', 'Péndulo de Furuta — Control Digital & Gemelo 3D (STM32F446RE)', ...
                 'NumberTitle', 'off', ...
                 'MenuBar', 'none', ...
                 'ToolBar', 'none', ...
                 'Color', C_BG, ...
                 'Units', 'normalized', ...
                 'OuterPosition', [0 0 1 1], ...
                 'WindowState', 'maximized', ...
                 'CloseRequestFcn', @onCloseApp);

    %% ====================================================================
    %  COLUMNA 1 (IZQUIERDA): CONTROL, TELEMETRÍA Y CONSTANTES
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

    uicontrol('Parent', pnl_params, 'Style', 'text', ...
              'String', 'Planta R²=98.5%: Km=3.37 · τm=0.118s · Kac=0.874 · ζ=0.065', ...
              'FontName', 'Consolas', 'FontSize', 7.8, 'FontWeight', 'bold', ...
              'ForegroundColor', C_MUTED, 'BackgroundColor', C_TILE, ...
              'HorizontalAlignment', 'center', ...
              'Units', 'normalized', 'Position', [0.025, 0.022, 0.950, 0.048]);

    %% ====================================================================
    %  COLUMNA 2 (CENTRO): GEMELO DIGITAL 3D HERO + DIALES POLARES
    %% ====================================================================

    % --- Tarjeta 4: Animación 3D en Tiempo Real (Gemelo Digital Furuta) ---
    pnl_3d = uipanel('Parent', fig, ...
                     'Title', ' Gemelo Digital 3D — Péndulo de Furuta (Tiempo Real) ', ...
                     'FontName', 'Segoe UI', 'FontSize', 9.5, 'FontWeight', 'bold', ...
                     'ForegroundColor', C_INK, ...
                     'BackgroundColor', C_CARD, ...
                     'BorderType', 'line', 'HighlightColor', C_BORDER, ...
                     'Units', 'normalized', ...
                     'Position', [0.290, 0.322, 0.365, 0.660]);

    % Barra superior de vistas rápidas y rastro 3D
    uicontrol('Parent', pnl_3d, 'Style', 'pushbutton', 'String', 'Iso 3D', ...
              'FontName', 'Segoe UI', 'FontSize', 8.2, 'FontWeight', 'bold', ...
              'BackgroundColor', C_TILE, 'ForegroundColor', C_INK, ...
              'Units', 'normalized', 'Position', [0.020, 0.930, 0.145, 0.048], ...
              'Callback', @(~,~) set3DView(38, 22));
    uicontrol('Parent', pnl_3d, 'Style', 'pushbutton', 'String', 'Frontal', ...
              'FontName', 'Segoe UI', 'FontSize', 8.2, 'FontWeight', 'bold', ...
              'BackgroundColor', C_TILE, 'ForegroundColor', C_INK, ...
              'Units', 'normalized', 'Position', [0.175, 0.930, 0.145, 0.048], ...
              'Callback', @(~,~) set3DView(90, 8));
    uicontrol('Parent', pnl_3d, 'Style', 'pushbutton', 'String', 'Superior', ...
              'FontName', 'Segoe UI', 'FontSize', 8.2, 'FontWeight', 'bold', ...
              'BackgroundColor', C_TILE, 'ForegroundColor', C_INK, ...
              'Units', 'normalized', 'Position', [0.330, 0.930, 0.145, 0.048], ...
              'Callback', @(~,~) set3DView(0, 88));
    S.btn_trail = uicontrol('Parent', pnl_3d, 'Style', 'pushbutton', 'String', 'Rastro: ON', ...
                            'FontName', 'Segoe UI', 'FontSize', 8.2, 'FontWeight', 'bold', ...
                            'BackgroundColor', C_TEAL, 'ForegroundColor', [1 1 1], ...
                            'Units', 'normalized', 'Position', [0.490, 0.930, 0.185, 0.048], ...
                            'Callback', @onToggleTrail);
    S.lbl_3d_badge = uicontrol('Parent', pnl_3d, 'Style', 'text', ...
                               'String', '● IDLE', ...
                               'FontName', 'Consolas', 'FontSize', 8.6, 'FontWeight', 'bold', ...
                               'ForegroundColor', C_BLUE, 'BackgroundColor', C_TILE, ...
                               'HorizontalAlignment', 'center', ...
                               'Units', 'normalized', 'Position', [0.690, 0.926, 0.290, 0.040]);

    % Viewport 3D de alta precisión (encuadre limpio sin invadir barra inferior)
    S.ax_3d = axes('Parent', pnl_3d, 'Units', 'normalized', ...
                   'Position', [0.020, 0.082, 0.960, 0.835], ...
                   'Color', C_CARD, ...
                   'Projection', 'perspective', ...
                   'XColor', 'none', 'YColor', 'none', 'ZColor', 'none');
    hold(S.ax_3d, 'on');
    buildFuruta3DScene();

    % Pie informativo de pose y guía interactiva
    S.lbl_3d_pose = uicontrol('Parent', pnl_3d, 'Style', 'text', ...
                              'String', 'θ = +0.0°   |   α = +0.0°   |   α_err = +180.0°   ·   Arrastra para orbitar 3D', ...
                              'FontName', 'Consolas', 'FontSize', 8.1, 'FontWeight', 'bold', ...
                              'ForegroundColor', C_MUTED, 'BackgroundColor', C_TILE, ...
                              'HorizontalAlignment', 'center', ...
                              'Units', 'normalized', 'Position', [0.020, 0.015, 0.960, 0.042]);

    % --- Tarjeta 5: Diales Polares Compactos (Brazo θ y Péndulo α) ---
    pnl_dials = uipanel('Parent', fig, ...
                        'Title', ' Indicadores Angulares ', ...
                        'FontName', 'Segoe UI', 'FontSize', 9.5, 'FontWeight', 'bold', ...
                        'ForegroundColor', C_INK, ...
                        'BackgroundColor', C_CARD, ...
                        'BorderType', 'line', 'HighlightColor', C_BORDER, ...
                        'Units', 'normalized', ...
                        'Position', [0.290, 0.018, 0.365, 0.290]);

    th_circ = linspace(0, 2*pi, 180);

    % 2.1. Dial Polar del Brazo θ (°)
    S.ax_dial_arm = axes('Parent', pnl_dials, 'Units', 'normalized', ...
                         'Position', [0.030, 0.040, 0.450, 0.820], ...
                         'Color', C_CARD, 'XColor', 'none', 'YColor', 'none');
    hold(S.ax_dial_arm, 'on'); axis(S.ax_dial_arm, 'equal');
    xlim(S.ax_dial_arm, [-1.32 1.32]); ylim(S.ax_dial_arm, [-1.32 1.32]);
    patch(S.ax_dial_arm, cos(th_circ), sin(th_circ), C_TILE, ...
          'EdgeColor', [0.30 0.36 0.45], 'LineWidth', 1.4, 'HandleVisibility', 'off');
    plot(S.ax_dial_arm, [-1 1], [0 0], ':', 'Color', [0.78 0.82 0.88], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    plot(S.ax_dial_arm, [0 0], [-1 1], ':', 'Color', [0.78 0.82 0.88], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    for ang_tick = -165:15:180
        r_rad = deg2rad(ang_tick);
        if mod(ang_tick, 90) == 0
            plot(S.ax_dial_arm, [0.84*cos(r_rad), cos(r_rad)], [0.84*sin(r_rad), sin(r_rad)], ...
                 'Color', C_INK, 'LineWidth', 1.3, 'HandleVisibility', 'off');
            text(S.ax_dial_arm, 1.18*cos(r_rad), 1.18*sin(r_rad), sprintf('%d°', ang_tick), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                 'FontName', 'Segoe UI', 'FontSize', 7.8, 'FontWeight', 'bold', 'Color', C_INK);
        elseif mod(ang_tick, 45) == 0
            plot(S.ax_dial_arm, [0.89*cos(r_rad), cos(r_rad)], [0.89*sin(r_rad), sin(r_rad)], ...
                 'Color', C_MUTED, 'LineWidth', 0.9, 'HandleVisibility', 'off');
        else
            plot(S.ax_dial_arm, [0.93*cos(r_rad), cos(r_rad)], [0.93*sin(r_rad), sin(r_rad)], ...
                 'Color', [0.72 0.76 0.82], 'LineWidth', 0.7, 'HandleVisibility', 'off');
        end
    end
    S.h_arm_meas = plot(S.ax_dial_arm, [0, 1], [0, 0], '-o', 'Color', C_BLUE, ...
                        'LineWidth', 2.8, 'MarkerSize', 6, 'MarkerFaceColor', C_BLUE);
    plot(S.ax_dial_arm, 0, 0, 'o', 'Color', C_INK, 'MarkerSize', 4.5, 'MarkerFaceColor', C_INK, ...
         'HandleVisibility', 'off');
    title(S.ax_dial_arm, 'Brazo \theta', 'FontName', 'Segoe UI', 'FontSize', 9.2, ...
          'FontWeight', 'bold', 'Color', C_INK);

    % 2.2. Dial Vertical del Péndulo α (°)
    S.ax_dial_pend = axes('Parent', pnl_dials, 'Units', 'normalized', ...
                          'Position', [0.520, 0.040, 0.450, 0.820], ...
                          'Color', C_CARD, 'XColor', 'none', 'YColor', 'none');
    hold(S.ax_dial_pend, 'on'); axis(S.ax_dial_pend, 'equal');
    xlim(S.ax_dial_pend, [-1.36 1.36]); ylim(S.ax_dial_pend, [-1.36 1.36]);
    patch(S.ax_dial_pend, cos(th_circ), sin(th_circ), C_TILE, ...
          'EdgeColor', [0.30 0.36 0.45], 'LineWidth', 1.4, 'HandleVisibility', 'off');
    plot(S.ax_dial_pend, [-1 1], [0 0], ':', 'Color', [0.78 0.82 0.88], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    plot(S.ax_dial_pend, [0 0], [-1 1], ':', 'Color', [0.78 0.82 0.88], 'LineWidth', 0.8, 'HandleVisibility', 'off');
    cone_ang = linspace(pi/2 - deg2rad(20), pi/2 + deg2rad(20), 40);
    patch(S.ax_dial_pend, [0, cos(cone_ang), 0], [0, sin(cone_ang), 0], [0.76 0.94 0.80], ...
          'EdgeColor', C_GREEN, 'LineWidth', 1.0, 'FaceAlpha', 0.55, 'HandleVisibility', 'off');
    text(S.ax_dial_pend, 0,  1.20, '180° (±20°)', 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', 'FontName', 'Segoe UI', 'FontSize', 7.8, ...
         'FontWeight', 'bold', 'Color', C_GREEN);
    text(S.ax_dial_pend, 0, -1.20, '0°', 'HorizontalAlignment', 'center', ...
         'VerticalAlignment', 'middle', 'FontName', 'Segoe UI', 'FontSize', 7.8, ...
         'FontWeight', 'bold', 'Color', C_MUTED);
    text(S.ax_dial_pend,  1.22, 0, '+90°', 'HorizontalAlignment', 'center', ...
         'FontName', 'Segoe UI', 'FontSize', 7.2, 'Color', C_MUTED);
    text(S.ax_dial_pend, -1.22, 0, '-90°', 'HorizontalAlignment', 'center', ...
         'FontName', 'Segoe UI', 'FontSize', 7.2, 'Color', C_MUTED);
    S.h_pend_bar = plot(S.ax_dial_pend, [0, 0], [0, -1], '-o', 'Color', C_ORANGE, ...
                        'LineWidth', 3.0, 'MarkerSize', 6.5, 'MarkerFaceColor', C_ORANGE);
    plot(S.ax_dial_pend, 0, 0, 'o', 'Color', C_INK, 'MarkerSize', 4.5, 'MarkerFaceColor', C_INK, ...
         'HandleVisibility', 'off');
    title(S.ax_dial_pend, 'Péndulo \alpha', 'FontName', 'Segoe UI', 'FontSize', 9.2, ...
          'FontWeight', 'bold', 'Color', C_INK);

    %% ====================================================================
    %  COLUMNA 3 (DERECHA): 3 GRÁFICAS TEMPORALES ALINEADAS
    %% ====================================================================
    pnl_plots = uipanel('Parent', fig, ...
                        'Title', ' Evolución Temporal (Tiempo Real) ', ...
                        'FontName', 'Segoe UI', 'FontSize', 9.5, 'FontWeight', 'bold', ...
                        'ForegroundColor', C_INK, ...
                        'BackgroundColor', C_CARD, ...
                        'BorderType', 'line', 'HighlightColor', C_BORDER, ...
                        'Units', 'normalized', ...
                        'Position', [0.665, 0.018, 0.325, 0.964]);

    % 3.1. Serie Temporal: Brazo θ(t)
    S.ax_pos = axes('Parent', pnl_plots, 'Units', 'normalized', ...
                    'Position', [0.145, 0.718, 0.725, 0.218]);
    styleTimeAxes(S.ax_pos);
    hold(S.ax_pos, 'on');
    yline(S.ax_pos, 0, ':', 'Color', [0.55 0.60 0.68], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    S.h_line_theta = plot(S.ax_pos, S.t_hist, S.theta_hist, '-', 'Color', C_BLUE, 'LineWidth', 1.9);
    title(S.ax_pos, 'Brazo \theta(t)', 'FontName', 'Segoe UI', 'FontSize', 9.8, ...
          'FontWeight', 'bold', 'Color', C_INK);
    xlabel(S.ax_pos, 't (s)', 'FontName', 'Segoe UI', 'FontSize', 8.5, 'Color', C_MUTED);
    ylabel(S.ax_pos, '\theta (°)', 'FontName', 'Segoe UI', 'FontSize', 8.8, 'Color', C_BLUE);
    ylim(S.ax_pos, [-45, 45]);

    % 3.2. Serie Temporal: Péndulo α(t) y dα/dt
    S.ax_pend_time = axes('Parent', pnl_plots, 'Units', 'normalized', ...
                          'Position', [0.145, 0.398, 0.725, 0.218]);
    styleTimeAxes(S.ax_pend_time);
    hold(S.ax_pend_time, 'on');
    yyaxis(S.ax_pend_time, 'left');
    S.h_line_alpha = plot(S.ax_pend_time, S.t_hist, S.alpha_hist, '-', 'Color', C_ORANGE, ...
                          'LineWidth', 1.9, 'DisplayName', '\alpha (°)');
    ylabel(S.ax_pend_time, '\alpha (°)', 'FontName', 'Segoe UI', 'FontSize', 8.8);
    ylim(S.ax_pend_time, [-190, 190]);
    S.ax_pend_time.YAxis(1).Color = C_ORANGE;

    yyaxis(S.ax_pend_time, 'right');
    S.h_line_alphadot = plot(S.ax_pend_time, S.t_hist, S.alphadot_hist, '--', 'Color', C_PURPLE, ...
                             'LineWidth', 1.35, 'DisplayName', 'd\alpha/dt');
    ylabel(S.ax_pend_time, 'd\alpha/dt (rad/s)', 'FontName', 'Segoe UI', 'FontSize', 8.6);
    ylim(S.ax_pend_time, [-20, 20]);
    S.ax_pend_time.YAxis(2).Color = C_PURPLE;

    title(S.ax_pend_time, 'Péndulo \alpha(t) y d\alpha/dt', 'FontName', 'Segoe UI', ...
          'FontSize', 9.8, 'FontWeight', 'bold', 'Color', C_INK);
    xlabel(S.ax_pend_time, 't (s)', 'FontName', 'Segoe UI', 'FontSize', 8.5, 'Color', C_MUTED);
    legend(S.ax_pend_time, [S.h_line_alpha, S.h_line_alphadot], {'\alpha (°)', 'd\alpha/dt'}, ...
           'Location', 'northeast', 'FontName', 'Segoe UI', 'FontSize', 8.0, 'Box', 'on', ...
           'Color', C_CARD, 'EdgeColor', C_BORDER);

    % 3.3. Serie Temporal: Control L298N (PWM y Tensión u)
    S.ax_pwm = axes('Parent', pnl_plots, 'Units', 'normalized', ...
                    'Position', [0.145, 0.078, 0.725, 0.218]);
    styleTimeAxes(S.ax_pwm);
    hold(S.ax_pwm, 'on');
    yline(S.ax_pwm, 0, ':', 'Color', [0.55 0.60 0.68], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    S.h_line_pwm = plot(S.ax_pwm, S.t_hist, S.pwm_hist, '-', 'Color', C_GREEN, ...
                        'LineWidth', 1.85, 'DisplayName', 'PWM (%)');
    S.h_line_u   = plot(S.ax_pwm, S.t_hist, S.u_hist * (100/12), ':', 'Color', C_BLUE, ...
                        'LineWidth', 1.55, 'DisplayName', 'u (% Vmax)');
    title(S.ax_pwm, 'Control L298N', 'FontName', 'Segoe UI', 'FontSize', 9.8, ...
          'FontWeight', 'bold', 'Color', C_INK);
    xlabel(S.ax_pwm, 't (s)', 'FontName', 'Segoe UI', 'FontSize', 8.5, 'Color', C_MUTED);
    ylabel(S.ax_pwm, 'PWM / u (%)', 'FontName', 'Segoe UI', 'FontSize', 8.8, 'Color', C_INK);
    ylim(S.ax_pwm, [-80, 80]);
    legend(S.ax_pwm, [S.h_line_pwm, S.h_line_u], {'PWM (%)', 'u (% Vmax)'}, ...
           'Location', 'northeast', 'FontName', 'Segoe UI', 'FontSize', 8.0, 'Box', 'on', ...
           'Color', C_CARD, 'EdgeColor', C_BORDER);

    % Habilitar rotación interactiva con mouse EXCLUSIVAMENTE sobre el visor 3D
    try
        r3d = rotate3d(fig);
        r3d.Enable = 'on';
        setAllowAxesRotate(r3d, [S.ax_dial_arm, S.ax_dial_pend, ...
                                 S.ax_pos, S.ax_pend_time, S.ax_pwm], false);
    catch
    end

    % Aplicar pose inicial en 3D y 2D
    updateVisualPose(0.0, 0.0, 180.0, 0, false);

    % Exponer callback de inspección/prueba (sin afectar la operación normal)
    setappdata(fig, 'SetPreviewPose', @setPreviewPose);
    setappdata(fig, 'Set3DView',      @set3DView);

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
    %  CONSTRUCCIÓN DEL GEMELO DIGITAL 3D (COTAS REALES FreeCAD EN mm)
    %% ====================================================================
    function buildFuruta3DScene()
        ax = S.ax_3d;

        % Paleta fiel al modelo CAD del Péndulo de Furuta
        c_graphite = [0.20 0.23 0.28];
        c_tower    = [0.25 0.29 0.35];
        c_cap      = [0.30 0.44 0.66];
        c_motor    = [0.13 0.14 0.16];
        c_gearbox  = [0.78 0.79 0.76];
        c_steel    = [0.82 0.85 0.90];
        c_bearing  = [0.54 0.58 0.64];
        c_pcb      = [0.07 0.68 0.30];
        c_magnet   = [0.88 0.12 0.18];
        c_floor    = [0.91 0.93 0.96];
        c_ring     = [0.73 0.78 0.85];

        % 1. Mesa de referencia y brújula polar en Z = 0
        th_f = linspace(0, 2*pi, 96);
        patch('Parent', ax, 'XData', 148*cos(th_f), 'YData', 148*sin(th_f), ...
              'ZData', -0.5*ones(size(th_f)), 'FaceColor', c_floor, ...
              'EdgeColor', c_ring, 'LineWidth', 1.1, 'FaceAlpha', 0.92, ...
              'SpecularStrength', 0.05);
        for r_ring = [90.45, 124]
            plot3(ax, r_ring*cos(th_f), r_ring*sin(th_f), 0.2*ones(size(th_f)), ...
                  ':', 'Color', c_ring, 'LineWidth', 0.9);
        end
        for ang_d = [0, 90, 180, -90]
            ar = deg2rad(ang_d);
            plot3(ax, [64*cos(ar), 146*cos(ar)], [64*sin(ar), 146*sin(ar)], [0.3 0.3], ...
                  ':', 'Color', [0.70 0.75 0.82], 'LineWidth', 0.8);
            text(ax, 163*cos(ar), 163*sin(ar), 3, sprintf('%d°', ang_d), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                 'FontName', 'Segoe UI', 'FontSize', 8.0, 'FontWeight', 'bold', ...
                 'Color', C_MUTED);
        end

        % 2. Estructura Estática: Base 120x120 mm, 4 Aletas, Torre Abierta y Motor Namiki
        addBoxPatch(ax, -60, 60, -60, 60, 0, 8, c_graphite, 1.0, [0.12 0.14 0.18]);
        % 4 tornillos M5 en esquinas de base
        for bx = [-48, 48]
            for by = [-48, 48]
                addCylSurf(ax, 'z', [bx, by, 8], 4.2, 3.0, 14, c_steel, 1.0, true);
            end
        end
        % 4 aletas triangulares estructurales de refuerzo (R=21..55, Z=8..68)
        for ang_fin = [0, pi/2, pi, 3*pi/2]
            addGussetFin(ax, ang_fin, 19, 55, 8, 68, 6.0, c_tower);
        end
        % Pedestal central inferior (Z=8..76) y 4 columnas abiertas (Z=76..148)
        addBoxPatch(ax, -21, 21, -21, 21, 8, 76, c_tower, 1.0, [0.16 0.19 0.24]);
        for cx = [-17.5, 17.5]
            for cy = [-17.5, 17.5]
                addBoxPatch(ax, cx-4.5, cx+4.5, cy-4.5, cy+4.5, 76, 150, c_tower, 1.0, [0.16 0.19 0.24]);
            end
        end
        % Brida superior de la torre (52x52, Z=148..156) y Tapa Motor (Z=156..161)
        addBoxPatch(ax, -26, 26, -26, 26, 148, 156, c_graphite, 1.0, [0.12 0.14 0.18]);
        addBoxPatch(ax, -26, 26, -26, 26, 156, 161, c_cap, 1.0, [0.20 0.30 0.48]);
        % Motor Namiki 22CL (visible por las ventanas de la torre) + Reductor + Eje
        addCylSurf(ax, 'z', [0, 0, 96],  11.0, 36.0, 22, c_motor,   1.0, true);
        addCylSurf(ax, 'z', [0, 0, 132], 11.0, 24.0, 22, c_gearbox, 1.0, true);
        addCylSurf(ax, 'z', [0, 0, 88],  12.0,  8.0, 18, [0.82 0.85 0.15], 1.0, true); % Acoplador AS5600 base
        addCylSurf(ax, 'z', [0, 0, 161],  3.2, 12.0, 16, c_steel,   1.0, true);        % Eje salida motor

        % 3. Grupo Transformable 1: BRAZO HORIZONTAL (rota θ alrededor de +Z)
        S.T_arm = hgtransform('Parent', ax);

        % Cubo del brazo sobre el eje motor
        addCylSurf(S.T_arm, 'z', [0, 0, 162.5], 11.0, 10.5, 22, C_CYAN, 1.0, true);
        % Cuerpo del brazo horizontal monolítico (X=0..82, Z=162.5..171.5)
        addBoxPatch(S.T_arm, 0, 82, -10.5, 10.5, 162.5, 171.5, C_CYAN, 1.0, [0.00 0.42 0.62]);
        % Cabezal distal portarrodamiento (X=68..82, Y=±13.5, Z=162.5..200)
        addBoxPatch(S.T_arm, 68, 82, -13.5, 13.5, 162.5, 199.5, C_CYAN, 1.0, [0.00 0.42 0.62]);
        % Rodamiento 608ZZ metálico en el pivote (Z=182.8, X=74..82.2)
        addCylSurf(S.T_arm, 'x', [73.5, 0, S.Z_PIVOT_mm], 11.2, 8.8, 24, c_bearing, 1.0, true);
        % PCB verde AS5600 distal e imán rubí en cara interna del cabezal
        addBoxPatch(S.T_arm, 65.5, 68.0, -11.5, 11.5, S.Z_PIVOT_mm-11.5, S.Z_PIVOT_mm+11.5, ...
                    c_pcb, 1.0, [0.04 0.45 0.18]);
        addCylSurf(S.T_arm, 'x', [69.0, 0, S.Z_PIVOT_mm], 3.2, 4.5, 16, c_magnet, 1.0, true);

        % Puntero radial proyectado en el suelo para lectura inmediata de θ
        plot3(S.T_arm, [0, S.L0_mm], [0, 0], [0.9, 0.9], '--', 'Color', C_BLUE, 'LineWidth', 1.4);
        plot3(S.T_arm, S.L0_mm, 0, 1.0, 'o', 'Color', C_BLUE, 'MarkerSize', 5, 'MarkerFaceColor', C_BLUE);

        % Cono translúcido de captura vertical (±20° alrededor de α=180°) en el pivote
        cone_a = linspace(-deg2rad(20), deg2rad(20), 28);
        r_cone = 105.0;
        xc_fan = S.L0_mm * ones(1, numel(cone_a) + 2);
        yc_fan = [0, r_cone * sin(cone_a), 0];
        zc_fan = S.Z_PIVOT_mm + [0, r_cone * cos(cone_a), 0];
        S.h_cone_fill = patch('Parent', S.T_arm, 'XData', xc_fan, 'YData', yc_fan, 'ZData', zc_fan, ...
                              'FaceColor', C_GREEN, 'FaceAlpha', 0.14, ...
                              'EdgeColor', C_GREEN, 'EdgeAlpha', 0.55, 'LineWidth', 1.0);
        % Eje vertical objetivo (α = 180°)
        plot3(S.T_arm, [S.L0_mm, S.L0_mm], [0, 0], [S.Z_PIVOT_mm, S.Z_PIVOT_mm + 165], ...
              ':', 'Color', C_GREEN, 'LineWidth', 1.1);

        % 4. Grupo Transformable 2: BARRA DEL PÉNDULO (hijo de T_arm, rota α en eje local +X)
        S.T_pend = hgtransform('Parent', S.T_arm);

        % Eje de acero inoxidable atravesando el rodamiento 608 hasta la barra
        addCylSurf(S.T_pend, 'x', [-14.0, 0, 0], 4.0, 18.0, 18, c_steel, 1.0, true);
        % Cubo pivote naranja de la barra (alrededor del eje local 0)
        addCylSurf(S.T_pend, 'x', [-3.2, 0, 0], 8.2, 6.4, 20, C_ORANGE, 1.0, true);
        % Barra principal del péndulo (desde Z=0 hasta Z=-L1_mm en reposo colgado α=0°)
        addBoxPatch(S.T_pend, -2.8, 2.8, -5.0, 5.0, -S.L1_mm + 4.5, 0, C_ORANGE, 1.0, [0.68 0.22 0.02]);
        % Punta redondeada brillante
        [sx, sy, sz] = sphere(16);
        surf('Parent', S.T_pend, ...
             'XData', 5.2*sx, 'YData', 5.2*sy, 'ZData', -S.L1_mm + 5.2*sz, ...
             'FaceColor', [0.98 0.45 0.08], 'EdgeColor', 'none', ...
             'AmbientStrength', 0.5, 'DiffuseStrength', 0.75, 'SpecularStrength', 0.5);
        % Contrapeso ajustable (púrpura anodizado) + tuerca metálica
        addCylSurf(S.T_pend, 'z', [0, 0, -134.0], 8.8, 24.0, 22, C_PURPLE, 1.0, true);
        addCylSurf(S.T_pend, 'x', [-9.8, 0, -122.0], 2.5, 19.6, 12, c_steel, 1.0, true);

        % 5. Rastro 3D en vivo y sombra vertical proyectada sobre la mesa
        S.h_trail_3d   = plot3(ax, S.trail_x, S.trail_y, S.trail_z, '-', ...
                               'Color', [0.91 0.34 0.05], 'LineWidth', 1.7);
        S.h_tip_drop   = plot3(ax, [S.L0_mm, S.L0_mm], [0, 0], [0.8, S.Z_PIVOT_mm - S.L1_mm], ...
                               ':', 'Color', [0.88 0.45 0.20], 'LineWidth', 0.9);
        S.h_tip_shadow = plot3(ax, S.L0_mm, 0, 0.8, 'o', ...
                               'Color', C_ORANGE, 'MarkerSize', 6, ...
                               'MarkerFaceColor', [0.96 0.65 0.42]);

        % 6. Iluminación de estudio y encuadre centrado sin recortes en ningún ángulo
        xlim(ax, [-215, 215]);
        ylim(ax, [-215, 215]);
        zlim(ax, [-68, 368]);
        axis(ax, 'equal');
        axis(ax, 'vis3d');
        axis(ax, 'off');
        view(ax, 38, 22);
        camtarget(ax, [0, 0, 148]);
        lighting(ax, 'gouraud');
        light('Parent', ax, 'Position', [360, -290, 480], 'Style', 'infinite', 'Color', [1.00 0.99 0.97]);
        light('Parent', ax, 'Position', [-320, 260, 210], 'Style', 'infinite', 'Color', [0.58 0.64 0.74]);
    end

    function addBoxPatch(parent, x0, x1, y0, y1, z0, z1, fcolor, falpha, ecolor)
        V = [x0 y0 z0; x1 y0 z0; x1 y1 z0; x0 y1 z0; ...
             x0 y0 z1; x1 y0 z1; x1 y1 z1; x0 y1 z1];
        F = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 4 3 2; 5 6 7 8];
        patch('Parent', parent, 'Vertices', V, 'Faces', F, ...
              'FaceColor', fcolor, 'FaceAlpha', falpha, ...
              'EdgeColor', ecolor, 'EdgeAlpha', 0.35, 'LineWidth', 0.5, ...
              'AmbientStrength', 0.48, 'DiffuseStrength', 0.72, ...
              'SpecularStrength', 0.28, 'SpecularExponent', 22);
    end

    function addCylSurf(parent, axisDir, origin, r, len, nSeg, fcolor, falpha, withCaps)
        [cx, cy, cz] = cylinder(r, nSeg);
        cz = cz * len;
        if axisDir == 'z'
            X = cx + origin(1);
            Y = cy + origin(2);
            Z = cz + origin(3);
            cap1 = [X(1,:); Y(1,:); Z(1,:)];
            cap2 = [X(2,:); Y(2,:); Z(2,:)];
        else
            % Eje a lo largo de +X
            X = cz + origin(1);
            Y = cx + origin(2);
            Z = cy + origin(3);
            cap1 = [X(1,:); Y(1,:); Z(1,:)];
            cap2 = [X(2,:); Y(2,:); Z(2,:)];
        end
        surf('Parent', parent, 'XData', X, 'YData', Y, 'ZData', Z, ...
             'FaceColor', fcolor, 'FaceAlpha', falpha, 'EdgeColor', 'none', ...
             'AmbientStrength', 0.48, 'DiffuseStrength', 0.74, ...
             'SpecularStrength', 0.42, 'SpecularExponent', 28);
        if withCaps
            patch('Parent', parent, 'XData', cap1(1,:), 'YData', cap1(2,:), 'ZData', cap1(3,:), ...
                  'FaceColor', fcolor, 'FaceAlpha', falpha, 'EdgeColor', 'none', ...
                  'AmbientStrength', 0.48, 'DiffuseStrength', 0.72, 'SpecularStrength', 0.35);
            patch('Parent', parent, 'XData', cap2(1,:), 'YData', cap2(2,:), 'ZData', cap2(3,:), ...
                  'FaceColor', fcolor, 'FaceAlpha', falpha, 'EdgeColor', 'none', ...
                  'AmbientStrength', 0.48, 'DiffuseStrength', 0.72, 'SpecularStrength', 0.35);
        end
    end

    function addGussetFin(parent, angRad, rIn, rOut, zBot, zTop, thick, fcolor)
        ht = thick / 2;
        % Perfil triangular en plano local (r, t, z)
        pts_loc = [rIn, -ht, zBot; rOut, -ht, zBot; rIn, -ht, zTop; ...
                   rIn,  ht, zBot; rOut,  ht, zBot; rIn,  ht, zTop];
        ca = cos(angRad); sa = sin(angRad);
        Rz = [ca -sa 0; sa ca 0; 0 0 1];
        V = (Rz * pts_loc.').';
        F = [1 2 3 3; 4 6 5 5; 1 4 5 2; 2 5 6 3; 3 6 4 1];
        patch('Parent', parent, 'Vertices', V, 'Faces', F, ...
              'FaceColor', fcolor, 'EdgeColor', [0.15 0.18 0.23], 'EdgeAlpha', 0.30, ...
              'AmbientStrength', 0.48, 'DiffuseStrength', 0.72, 'SpecularStrength', 0.20);
    end

    function set3DView(az, el)
        if ishandle(S.ax_3d)
            view(S.ax_3d, az, el);
            camtarget(S.ax_3d, [0, 0, 148]);
        end
    end

    function onToggleTrail(~, ~)
        S.trail_enabled = ~S.trail_enabled;
        if S.trail_enabled
            set(S.btn_trail, 'String', 'Rastro: ON', 'BackgroundColor', C_TEAL);
            set(S.h_trail_3d, 'Visible', 'on');
        else
            set(S.btn_trail, 'String', 'Rastro: OFF', 'BackgroundColor', C_MUTED);
            set(S.h_trail_3d, 'Visible', 'off');
        end
    end

    function clear3DTrail()
        S.trail_x = nan(1, S.trail_len);
        S.trail_y = nan(1, S.trail_len);
        S.trail_z = nan(1, S.trail_len);
        if ishandle(S.h_trail_3d)
            set(S.h_trail_3d, 'XData', S.trail_x, 'YData', S.trail_y, 'ZData', S.trail_z);
        end
    end

    function updateVisualPose(theta_deg, alpha_deg, alpha_err_deg, fsm_st, append_trail)
        th_r = deg2rad(theta_deg);
        al_r = deg2rad(alpha_deg);

        % 1. Transformaciones rígidas 3D exactas (O(1), sin reconstruir mallas)
        set(S.T_arm,  'Matrix', makehgtform('zrotate', th_r));
        set(S.T_pend, 'Matrix', makehgtform('translate', [S.L0_mm, 0, S.Z_PIVOT_mm]) * ...
                                makehgtform('xrotate', al_r));

        % 2. Posición 3D exacta de la punta del péndulo para rastro y sombra
        x_tip = S.L0_mm * cos(th_r) - S.L1_mm * sin(al_r) * sin(th_r);
        y_tip = S.L0_mm * sin(th_r) + S.L1_mm * sin(al_r) * cos(th_r);
        z_tip = S.Z_PIVOT_mm - S.L1_mm * cos(al_r);

        set(S.h_tip_shadow, 'XData', x_tip, 'YData', y_tip, 'ZData', 0.8);
        set(S.h_tip_drop,   'XData', [x_tip, x_tip], 'YData', [y_tip, y_tip], 'ZData', [0.8, z_tip]);

        if append_trail
            S.trail_x = [S.trail_x(2:end), x_tip];
            S.trail_y = [S.trail_y(2:end), y_tip];
            S.trail_z = [S.trail_z(2:end), z_tip];
            if S.trail_enabled
                set(S.h_trail_3d, 'XData', S.trail_x, 'YData', S.trail_y, 'ZData', S.trail_z);
            end
        end

        % 3. Resaltado visual del cono de captura ±20° al entrar en EQUILIBRIO
        is_bal = double(fsm_st == 6);
        if is_bal ~= S.last_bal_state
            S.last_bal_state = is_bal;
            if is_bal
                set(S.h_cone_fill, 'FaceAlpha', 0.30, 'EdgeAlpha', 0.90, 'LineWidth', 1.4);
            else
                set(S.h_cone_fill, 'FaceAlpha', 0.13, 'EdgeAlpha', 0.45, 'LineWidth', 0.9);
            end
        end

        % 4. Actualizar etiquetas del visor 3D
        st_name = fsmStateName(fsm_st);
        if fsm_st == 6
            badge_col = C_GREEN;
            badge_txt = sprintf('● EQUILIBRIO (%.1f°)', alpha_err_deg);
        elseif fsm_st == 5 || fsm_st == 8
            badge_col = C_ORANGE;
            badge_txt = sprintf('● %s', st_name);
        else
            badge_col = C_BLUE;
            badge_txt = sprintf('● %s', st_name);
        end
        set(S.lbl_3d_badge, 'String', badge_txt, 'ForegroundColor', badge_col);
        set(S.lbl_3d_pose,  'String', sprintf('θ = %+6.1f°   |   α = %+6.1f°   |   α_err = %+6.1f°   ·   Arrastra para orbitar 3D', ...
                                              theta_deg, alpha_deg, alpha_err_deg));

        % 5. Diales polares 2D sincronizados
        set(S.h_arm_meas, 'XData', [0, cos(th_r)], 'YData', [0, sin(th_r)]);
        r_pend_vis = -pi/2 + al_r;
        set(S.h_pend_bar, 'XData', [0, cos(r_pend_vis)], 'YData', [0, sin(r_pend_vis)]);
    end

    function setPreviewPose(theta_deg, alpha_deg, fsm_st, u_volts, pwm_pct, omega_r, adot_r)
        if nargin < 3, fsm_st  = 0;   end
        if nargin < 4, u_volts = 0.0; end
        if nargin < 5, pwm_pct = 0.0; end
        if nargin < 6, omega_r = 0.0; end
        if nargin < 7, adot_r  = 0.0; end

        a_err = mod(alpha_deg - 180.0 + 180.0, 360.0) - 180.0;
        S.theta_deg       = theta_deg;
        S.alpha_deg       = alpha_deg;
        S.alpha_err_deg   = a_err;
        S.fsm_state       = fsm_st;
        S.u_volts         = u_volts;
        S.pwm_pct         = pwm_pct;
        S.omega_rad_s     = omega_r;
        S.alpha_dot_rad_s = adot_r;

        t_now = S.t_hist(end) + 0.033;
        S.t_hist        = [S.t_hist(2:end), t_now];
        S.theta_hist    = [S.theta_hist(2:end), theta_deg];
        S.alpha_hist    = [S.alpha_hist(2:end), alpha_deg];
        S.alphadot_hist = [S.alphadot_hist(2:end), adot_r];
        S.pwm_hist      = [S.pwm_hist(2:end), pwm_pct];
        S.u_hist        = [S.u_hist(2:end), u_volts];

        st_str = fsmStateName(fsm_st);
        set(S.lbl_fsm_big, 'String', st_str);
        if fsm_st == 6
            set(S.lbl_fsm_big, 'ForegroundColor', C_GREEN);
        elseif fsm_st == 5 || fsm_st == 8
            set(S.lbl_fsm_big, 'ForegroundColor', C_ORANGE);
        else
            set(S.lbl_fsm_big, 'ForegroundColor', C_BLUE);
        end
        set(S.lbl_arm_pos,  'String', sprintf('%+6.2f °', theta_deg));
        set(S.lbl_arm_vel,  'String', sprintf('%+5.2f rad/s', omega_r));
        set(S.lbl_pend_pos, 'String', sprintf('%+6.2f °', alpha_deg));
        set(S.lbl_pend_err, 'String', sprintf('%+6.2f °', a_err));
        set(S.lbl_pend_vel, 'String', sprintf('%+5.2f rad/s', adot_r));
        set(S.lbl_pwm_val,  'String', sprintf('%+4.1fV · %+4.0f%%', u_volts, pwm_pct));

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

        updateVisualPose(theta_deg, alpha_deg, a_err, fsm_st, true);
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
                'FontName', 'Segoe UI', 'FontSize', 8.2, 'LineWidth', 0.9);
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
            clear3DTrail();
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
            clear3DTrail();

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
        clear3DTrail();
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
                    got_sample = false;
                    for k = 1:numel(lines)
                        ln = strtrim(lines{k});
                        if isempty(ln) || ln(1) == '#' || ln(1) == 't' || ln(1) == '='
                            continue;
                        end
                        vals = sscanf(ln, '%f,%f,%f,%f,%f,%f,%f,%f,%f');
                        if numel(vals) == 9
                            got_sample = true;
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

                    if got_sample
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

                        % Actualizar Gemelo Digital 3D + Diales Polares 2D
                        updateVisualPose(S.theta_deg, S.alpha_deg, S.alpha_err_deg, S.fsm_state, true);

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
