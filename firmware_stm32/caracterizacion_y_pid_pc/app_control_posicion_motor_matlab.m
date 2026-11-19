function app_control_posicion_motor_matlab()
%% ========================================================================
%  APLICACIÓN GRÁFICA INTERACTIVA (GUI) EN MATLAB PARA CONTROL DE POSICIÓN
%  ANGULAR PID DEL MOTOR NAMIKI 22CL + AS5600 EN STM32 NUCLEO-F446RE
%  ========================================================================
%  Proyecto Final — Control Digital (Universidad EAFIT)
%
%  Mejoras de fiabilidad 100% en tiempo real:
%    - Lectura serial 100% no bloqueante por bloques (read(..., "char"))
%      que jamás congela ni bloquea el puerto COM durante envíos.
%    - Cola de transmisión síncrona con ráfaga triple automática (3x burst)
%      compatible con el receptor DMA circular (DMA1_Stream5) de la STM32.
%    - Detección de tecla Enter en la caja de texto sin doble disparo al
%      hacer clic en el botón "EJECUTAR ÁNGULO".
%% ========================================================================

    % Cerrar instancias previas de puertos seriales o timers huérfanos
    try
        old_timers = timerfindall('Tag', 'FurutaMotorGUITimer');
        if ~isempty(old_timers)
            stop(old_timers);
            delete(old_timers);
        end
    catch
    end
    try
        open_ports = serialportfind();
        if ~isempty(open_ports)
            delete(open_ports);
        end
    catch
    end

    %% 1. Estado Interno de la Aplicación
    S = struct();
    S.serial_obj      = [];
    S.is_connected    = false;
    S.is_io_busy      = false;
    S.rx_partial_buf  = '';
    S.pending_cmd     = '';
    S.cmd_burst_left  = 0;

    S.target_deg      = 0.0;
    S.current_deg     = 0.0;
    S.current_vel     = 0.0;
    S.pend_deg        = 0.0;
    S.u_volts         = 0.0;
    S.pwm_pct         = 0.0;
    S.history_len     = 300; % Ventana de 3.0 segundos
    S.t_hist          = linspace(-3, 0, S.history_len);
    S.ref_hist        = zeros(1, S.history_len);
    S.theta_hist      = zeros(1, S.history_len);
    S.pwm_hist        = zeros(1, S.history_len);
    S.t0_tic          = tic;

    % Detectar puertos COM disponibles en Windows
    avail_ports = cellstr(serialportlist("available"));
    if isempty(avail_ports) || all(cellfun(@isempty, avail_ports))
        avail_ports = {'COM13', 'COM3', 'COM4', 'COM5'};
    elseif ~ismember('COM13', avail_ports)
        avail_ports = [{'COM13'}, reshape(avail_ports, 1, [])];
    end

    %% 2. Construcción de la Ventana Gráfica Principal
    fig = figure('Name', 'Control Digital PID de Posición Angular — STM32 Nucleo-F446RE + Motor Namiki 22CL + AS5600', ...
                 'NumberTitle', 'off', ...
                 'Color', [0.95 0.96 0.98], ...
                 'Position', [100, 80, 1220, 720], ...
                 'CloseRequestFcn', @onCloseApp);

    % Panel Lateral Izquierdo de Control
    pnl_ctrl = uipanel('Parent', fig, ...
                       'Title', ' Panel de Control del Motor (PID 1 kHz + DMA) ', ...
                       'FontSize', 11, 'FontWeight', 'bold', ...
                       'BackgroundColor', [0.92 0.94 0.97], ...
                       'Units', 'normalized', ...
                       'Position', [0.015, 0.02, 0.30, 0.96]);

    % --- Sección 1: Conexión Serial ---
    uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
              'String', 'Puerto Serial STM32 (115200 bps):', ...
              'FontSize', 10, 'FontWeight', 'bold', ...
              'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.92 0.94 0.97], ...
              'Units', 'normalized', 'Position', [0.05, 0.92, 0.55, 0.04]);

    idx_com13 = find(strcmpi(avail_ports, 'COM13'), 1);
    if isempty(idx_com13), idx_com13 = 1; end

    S.popup_port = uicontrol('Parent', pnl_ctrl, 'Style', 'popupmenu', ...
                             'String', avail_ports, 'Value', idx_com13, ...
                             'FontSize', 10, ...
                             'Units', 'normalized', 'Position', [0.62, 0.925, 0.33, 0.04]);

    S.btn_connect = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
                              'String', 'CONECTAR STM32', ...
                              'FontSize', 11, 'FontWeight', 'bold', ...
                              'BackgroundColor', [0.16 0.62 0.28], ...
                              'ForegroundColor', [1 1 1], ...
                              'Units', 'normalized', 'Position', [0.05, 0.855, 0.90, 0.055], ...
                              'Callback', @onToggleConnect);

    S.lbl_status = uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
                             'String', 'Estado: DESCONECTADO', ...
                             'FontSize', 10, 'FontWeight', 'bold', ...
                             'ForegroundColor', [0.75 0.15 0.15], ...
                             'BackgroundColor', [0.92 0.94 0.97], ...
                             'Units', 'normalized', 'Position', [0.05, 0.81, 0.90, 0.035]);

    % --- Sección 2: Ingreso de Ángulo Objetivo ---
    uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
              'String', 'Ingresar Ángulo Objetivo (°):', ...
              'FontSize', 11, 'FontWeight', 'bold', ...
              'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.92 0.94 0.97], ...
              'Units', 'normalized', 'Position', [0.05, 0.74, 0.90, 0.04]);

    S.edt_angle = uicontrol('Parent', pnl_ctrl, 'Style', 'edit', ...
                            'String', '45.0', ...
                            'FontSize', 16, 'FontWeight', 'bold', ...
                            'BackgroundColor', [1 1 1], ...
                            'Units', 'normalized', 'Position', [0.05, 0.67, 0.42, 0.065], ...
                            'KeyPressFcn', @onEditKeyPress);

    S.btn_exec = uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
                           'String', 'EJECUTAR ÁNGULO', ...
                           'FontSize', 11, 'FontWeight', 'bold', ...
                           'BackgroundColor', [0.12 0.45 0.82], ...
                           'ForegroundColor', [1 1 1], ...
                           'Units', 'normalized', 'Position', [0.50, 0.67, 0.45, 0.065], ...
                           'Callback', @onExecuteAngle);

    % Slider interactivo [-180° a +180°]
    S.sld_angle = uicontrol('Parent', pnl_ctrl, 'Style', 'slider', ...
                            'Min', -180, 'Max', 180, 'Value', 45, ...
                            'SliderStep', [1/360, 15/360], ...
                            'Units', 'normalized', 'Position', [0.05, 0.61, 0.90, 0.04], ...
                            'Callback', @onSliderMoved);

    uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
              'String', '-180°                                   0°                                  +180°', ...
              'FontSize', 8.5, 'BackgroundColor', [0.92 0.94 0.97], ...
              'Units', 'normalized', 'Position', [0.05, 0.58, 0.90, 0.025]);

    % --- Sección 3: Botones de Ángulos Rápidos cada 15° (-180° a +180°) ---
    uicontrol('Parent', pnl_ctrl, 'Style', 'text', ...
              'String', 'Posiciones Rápidas cada 15° (-180° a +180°):', ...
              'FontSize', 9.5, 'FontWeight', 'bold', ...
              'HorizontalAlignment', 'left', ...
              'BackgroundColor', [0.92 0.94 0.97], ...
              'Units', 'normalized', 'Position', [0.04, 0.545, 0.92, 0.03]);

    presets = -180:15:180; % 25 ángulos: 0, +/-15, +/-30, +/-45, ..., +/-180
    for i = 1:length(presets)
        ang_p = presets(i);
        col = mod(i-1, 5);
        row = floor((i-1) / 5);
        x_pos = 0.04 + col * 0.186;
        y_pos = 0.502 - row * 0.042;
        if ang_p == 0
            btn_str = '0°';
            btn_bg  = [0.18 0.55 0.34];
            btn_fg  = [1 1 1];
        elseif ang_p < 0
            btn_str = sprintf('%d°', ang_p);
            btn_bg  = [0.95 0.89 0.89];
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
                  'Units', 'normalized', 'Position', [x_pos, y_pos, 0.176, 0.038], ...
                  'Callback', @(~,~) setAndSendAngle(ang_p));
    end

    % --- Sección 4: Lecturas Digitales en Vivo ---
    pnl_telemetry = uipanel('Parent', pnl_ctrl, ...
                            'Title', ' Telemetría AS5600 en Tiempo Real ', ...
                            'FontSize', 10, 'FontWeight', 'bold', ...
                            'BackgroundColor', [0.98 0.99 1.00], ...
                            'Units', 'normalized', 'Position', [0.04, 0.145, 0.92, 0.185]);

    S.lbl_ref_val = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                              'String', 'Referencia:      0.00 °', ...
                              'FontSize', 11, 'FontWeight', 'bold', ...
                              'ForegroundColor', [0.80 0.15 0.15], ...
                              'HorizontalAlignment', 'left', ...
                              'BackgroundColor', [0.98 0.99 1.00], ...
                              'Units', 'normalized', 'Position', [0.06, 0.76, 0.88, 0.20]);

    S.lbl_meas_val = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                               'String', 'Ángulo Real:     0.00 °', ...
                               'FontSize', 12, 'FontWeight', 'bold', ...
                               'ForegroundColor', [0.08 0.38 0.78], ...
                               'HorizontalAlignment', 'left', ...
                               'BackgroundColor', [0.98 0.99 1.00], ...
                               'Units', 'normalized', 'Position', [0.06, 0.52, 0.88, 0.22]);

    S.lbl_err_val = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                              'String', 'Error Angular:   0.00 °', ...
                              'FontSize', 10.5, 'FontWeight', 'bold', ...
                              'ForegroundColor', [0.15 0.55 0.20], ...
                              'HorizontalAlignment', 'left', ...
                              'BackgroundColor', [0.98 0.99 1.00], ...
                              'Units', 'normalized', 'Position', [0.06, 0.28, 0.88, 0.20]);

    S.lbl_pwm_val = uicontrol('Parent', pnl_telemetry, 'Style', 'text', ...
                              'String', 'Esfuerzo Motor:  0.00 V (0.0% PWM)', ...
                              'FontSize', 9.5, ...
                              'HorizontalAlignment', 'left', ...
                              'BackgroundColor', [0.98 0.99 1.00], ...
                              'Units', 'normalized', 'Position', [0.06, 0.05, 0.88, 0.20]);

    % --- Sección 5: Botones de Calibración Cero y Parada ---
    uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
              'String', 'CALIBRAR CERO ACTUAL (0°)', ...
              'FontSize', 10, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.92 0.62 0.10], ...
              'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.04, 0.08, 0.92, 0.052], ...
              'Callback', @onZeroEncoder);

    uicontrol('Parent', pnl_ctrl, 'Style', 'pushbutton', ...
              'String', 'PARADA DE EMERGENCIA (STOP)', ...
              'FontSize', 10.5, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.82 0.15 0.15], ...
              'ForegroundColor', [1 1 1], ...
              'Units', 'normalized', 'Position', [0.04, 0.018, 0.92, 0.055], ...
              'Callback', @onEmergencyStop);

    %% 3. Ejes Gráficos en Tiempo Real (Derecha)
    % 3.1. Radar / Dial Polar 2D de la Posición del Brazo
    S.ax_dial = axes('Parent', fig, ...
                     'Units', 'normalized', ...
                     'Position', [0.35, 0.53, 0.26, 0.40]);
    hold(S.ax_dial, 'on');
    axis(S.ax_dial, 'equal');
    xlim(S.ax_dial, [-1.25 1.25]);
    ylim(S.ax_dial, [-1.25 1.25]);
    grid(S.ax_dial, 'on');
    title(S.ax_dial, 'Dial Angular del Brazo \theta (°)', 'FontSize', 11, 'FontWeight', 'bold');

    th_circ = linspace(0, 2*pi, 200);
    plot(S.ax_dial, cos(th_circ), sin(th_circ), 'k-', 'LineWidth', 1.5);
    for ang_tick = -165:15:180
        r_rad = deg2rad(ang_tick);
        if mod(ang_tick, 30) == 0
            plot(S.ax_dial, [0.88*cos(r_rad), 1.0*cos(r_rad)], [0.88*sin(r_rad), 1.0*sin(r_rad)], 'k-', 'LineWidth', 1.3);
            text(S.ax_dial, 1.14*cos(r_rad), 1.14*sin(r_rad), sprintf('%d°', ang_tick), ...
                 'HorizontalAlignment', 'center', 'FontSize', 8);
        else
            plot(S.ax_dial, [0.93*cos(r_rad), 1.0*cos(r_rad)], [0.93*sin(r_rad), 1.0*sin(r_rad)], 'Color', [0.45 0.45 0.45], 'LineWidth', 0.9);
        end
    end
    S.h_dial_ref  = plot(S.ax_dial, [0, 1], [0, 0], 'r--', 'LineWidth', 2.2, 'DisplayName', 'Referencia');
    S.h_dial_meas = plot(S.ax_dial, [0, 1], [0, 0], 'b-o', 'LineWidth', 3.0, 'MarkerFaceColor', 'b', 'DisplayName', 'Brazo Real');
    legend(S.ax_dial, 'Location', 'southoutside', 'Orientation', 'horizontal');

    % 3.2. Gráfica Temporal de Seguimiento Angular theta(t) vs Referencia
    S.ax_pos = axes('Parent', fig, ...
                    'Units', 'normalized', ...
                    'Position', [0.65, 0.55, 0.32, 0.38]);
    hold(S.ax_pos, 'on');
    grid(S.ax_pos, 'on');
    S.h_line_ref   = plot(S.ax_pos, S.t_hist, S.ref_hist, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Referencia \theta_{ref} (°)');
    S.h_line_theta = plot(S.ax_pos, S.t_hist, S.theta_hist, 'b-', 'LineWidth', 2.2, 'DisplayName', 'Medición AS5600 \theta (°)');
    title(S.ax_pos, 'Seguimiento Angular PID en Tiempo Real', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel(S.ax_pos, 'Tiempo (s)');
    ylabel(S.ax_pos, 'Ángulo \theta (°)');
    ylim(S.ax_pos, [-190, 190]);
    legend(S.ax_pos, 'Location', 'northeast');

    % 3.3. Gráfica Inferior: Señal de Control PWM (%) enviada al Puente H L298N
    S.ax_pwm = axes('Parent', fig, ...
                    'Units', 'normalized', ...
                    'Position', [0.36, 0.08, 0.61, 0.36]);
    hold(S.ax_pwm, 'on');
    grid(S.ax_pwm, 'on');
    S.h_line_pwm = plot(S.ax_pwm, S.t_hist, S.pwm_hist, 'Color', [0.15 0.55 0.25], 'LineWidth', 1.8, ...
                        'DisplayName', 'Ciclo Útil PWM Aplicado (%)');
    yline(S.ax_pwm, 0, 'k:');
    title(S.ax_pwm, 'Esfuerzo de Control Digital del Motor (PWM % en Driver L298N)', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel(S.ax_pwm, 'Tiempo (s)');
    ylabel(S.ax_pwm, 'PWM (%)');
    ylim(S.ax_pwm, [-105, 105]);
    legend(S.ax_pwm, 'Location', 'northeast');

    % Timer de refresco visual y despacho serial a 30 Hz (33 ms)
    S.gui_timer = timer('Tag', 'FurutaMotorGUITimer', ...
                        'ExecutionMode', 'fixedRate', ...
                        'Period', 0.033, ...
                        'BusyMode', 'drop', ...
                        'TimerFcn', @onTimerUpdateGUI);

    %% ====================================================================
    %  FUNCIONES CALLBACK Y COMUNICACIÓN SERIAL NO BLOQUEANTE
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

                set(S.btn_connect, 'String', 'DESCONECTAR STM32', ...
                                   'BackgroundColor', [0.75 0.22 0.18]);
                set(S.lbl_status, 'String', sprintf('Estado: CONECTADO A %s', sel_port), ...
                                  'ForegroundColor', [0.12 0.58 0.22]);

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
                pause(0.05);
            catch
            end
            delete(S.serial_obj);
        end
        S.serial_obj = [];
        S.is_connected = false;
        if ishandle(fig)
            set(S.btn_connect, 'String', 'CONECTAR STM32', ...
                               'BackgroundColor', [0.16 0.62 0.28]);
            set(S.lbl_status, 'String', 'Estado: DESCONECTADO', ...
                              'ForegroundColor', [0.75 0.15 0.15]);
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
            errordlg('Ingresa un valor numérico válido en grados (ej: 45, -30, 90).', 'Ángulo Inválido');
            return;
        end
        setAndSendAngle(val_deg);
    end

    function onSliderMoved(~, ~)
        val_deg = round(get(S.sld_angle, 'Value'), 1);
        setAndSendAngle(val_deg);
    end

    function queueSerialCommand(cmd_str)
        S.pending_cmd    = cmd_str;
        S.cmd_burst_left = 3; % Enviar en 3 tramas seguidas para garantía 100%
        % Enviar primera trama de inmediato si el canal no está ocupado
        if S.is_connected && ~S.is_io_busy && ~isempty(S.serial_obj) && isvalid(S.serial_obj)
            try
                write(S.serial_obj, sprintf('\r\n%s\r\n', cmd_str), "char");
                S.cmd_burst_left = 2;
            catch
            end
        end
    end

    function setAndSendAngle(val_deg)
        S.target_deg = val_deg;
        set(S.edt_angle, 'String', sprintf('%.1f', val_deg));
        sld_clamped = max(-180, min(180, val_deg));
        set(S.sld_angle, 'Value', sld_clamped);
        queueSerialCommand(sprintf('POS=%.2f', val_deg));
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

    function onTimerUpdateGUI(~, ~)
        if ~S.is_connected || S.is_io_busy || isempty(S.serial_obj) || ~isvalid(S.serial_obj)
            return;
        end
        S.is_io_busy = true;

        try
            % 1. Despachar ráfaga de comandos pendientes hacia la STM32
            if S.cmd_burst_left > 0 && ~isempty(S.pending_cmd)
                write(S.serial_obj, sprintf('\r\n%s\r\n', S.pending_cmd), "char");
                S.cmd_burst_left = S.cmd_burst_left - 1;
            end

            % 2. Lectura 100% NO BLOQUEANTE de todos los bytes disponibles
            n_avail = S.serial_obj.NumBytesAvailable;
            if n_avail > 0
                raw_chunk = read(S.serial_obj, n_avail, "char");
                S.rx_partial_buf = [S.rx_partial_buf, raw_chunk];

                % Separar por saltos de línea
                idx_nl = find(S.rx_partial_buf == newline);
                if ~isempty(idx_nl)
                    complete_block = S.rx_partial_buf(1:idx_nl(end));
                    S.rx_partial_buf = S.rx_partial_buf(idx_nl(end)+1:end);

                    lines = splitlines(complete_block);
                    for k = 1:numel(lines)
                        ln = strtrim(lines{k});
                        if isempty(ln) || ln(1) == '#' || ln(1) == 't' || ln(1) == '='
                            continue;
                        end
                        vals = sscanf(ln, '%f,%f,%f,%f,%f,%f,%f,%f,%f');
                        if numel(vals) == 9
                            t_now = toc(S.t0_tic);
                            S.pwm_pct     = vals(3);
                            S.u_volts     = vals(4);
                            S.current_deg = rad2deg(vals(5));
                            S.current_vel = vals(6);
                            S.pend_deg    = rad2deg(vals(7));

                            S.t_hist     = [S.t_hist(2:end), t_now];
                            S.ref_hist   = [S.ref_hist(2:end), S.target_deg];
                            S.theta_hist = [S.theta_hist(2:end), S.current_deg];
                            S.pwm_hist   = [S.pwm_hist(2:end), S.pwm_pct];
                        end
                    end

                    % 3. Actualizar interfaz gráfica
                    err_deg = S.target_deg - S.current_deg;
                    if abs(err_deg) < 0.06 && abs(S.pwm_pct) < 0.05
                        S.current_deg = S.target_deg;
                        err_deg = 0.0;
                    end
                    set(S.lbl_ref_val,  'String', sprintf('Referencia:    %+7.2f °', S.target_deg));
                    set(S.lbl_meas_val, 'String', sprintf('Ángulo Real:   %+7.2f °', S.current_deg));
                    set(S.lbl_err_val,  'String', sprintf('Error Angular: %+7.2f °', err_deg));
                    set(S.lbl_pwm_val,  'String', sprintf('Esfuerzo: %+5.2f V (%+5.1f%% PWM)', S.u_volts, S.pwm_pct));

                    r_ref  = deg2rad(S.target_deg);
                    r_meas = deg2rad(S.current_deg);
                    set(S.h_dial_ref,  'XData', [0, cos(r_ref)],  'YData', [0, sin(r_ref)]);
                    set(S.h_dial_meas, 'XData', [0, cos(r_meas)], 'YData', [0, sin(r_meas)]);

                    set(S.h_line_ref,   'XData', S.t_hist, 'YData', S.ref_hist);
                    set(S.h_line_theta, 'XData', S.t_hist, 'YData', S.theta_hist);
                    set(S.h_line_pwm,   'XData', S.t_hist, 'YData', S.pwm_hist);

                    t_min = S.t_hist(1);
                    t_max = max(S.t_hist(end), t_min + 0.5);
                    xlim(S.ax_pos, [t_min, t_max]);
                    xlim(S.ax_pwm, [t_min, t_max]);

                    y_max = max([100, abs(S.target_deg) + 30, max(abs(S.theta_hist)) + 20]);
                    ylim(S.ax_pos, [-y_max, y_max]);

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
