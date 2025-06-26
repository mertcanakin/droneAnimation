clc; clear; close all

% You can enter one log to display for one drone.
% logFiles = {'log_PID.mat'};
logFiles = {'log_SMC.mat', 'log_INDI.mat'};

% Set legends and colors
legends = {'SMC', 'INDI'};
colors = {'r', 'b'};
trailColors = {[1 0.5 0.5], [0.5 0.5 1]}; 

% legends = {'PID', 'SMC', 'INDI'};
% colors = {'r', 'g', 'b'};
% trailColors = {[1 0.5 0.5], [0.5 1 0.5], [0.5 0.5 1]}; 

% TO-DO: change here
% Get limits from a log
limdata = load('log_INDI.mat');
limXmin = min(limdata.log.x_log);
limYmin = min(limdata.log.y_log);
limZmin = min(limdata.log.z_log);
limXmax = max(limdata.log.x_log);
limYmax = max(limdata.log.y_log);
limZmax = max(limdata.log.z_log);
cnst = 1;
plotlim = [limXmin-cnst; limXmax+cnst; limYmin-cnst; limYmax+cnst; limZmin-cnst; limZmax+cnst];

% Call animation function
animateQuad(logFiles, colors, trailColors, 'quadcopter_anim.gif', plotlim, legends);

%% Function
function animateQuad(logs, colors, trailColors, outputGif, plotlim, legends)
    
    N_anim = size(logs,2);
    data = cell(1,N_anim);
    
    % TO-DO: Find a more modular way
    for i = 1:N_anim
        data{i} = load(logs{i});
        data{i}.time   = data{i}.log.time(:);
        data{i}.x      = data{i}.log.x_log(:);
        data{i}.y      = data{i}.log.y_log(:);
        data{i}.z      = data{i}.log.z_log(:);
        data{i}.x_des  = data{i}.log.x_des_log(:);
        data{i}.y_des  = data{i}.log.y_des_log(:);
        data{i}.z_des  = data{i}.log.z_des_log(:);
        data{i}.roll   = -data{i}.log.phi_log(:);
        data{i}.pitch  = -data{i}.log.theta_log(:);
        data{i}.yaw    = -data{i}.log.psi_log(:);
    end

    % Simulation time and time step
    maxTime = data{1}.time(end);
    timeStep = 0.2;

    % Create figure with fixed properties
    fig = figure('Position', [100, 100, 800, 600]);
    ax = axes('Parent', fig, 'DataAspectRatio', [1 1 1], ...
        'XLim', [plotlim(1) plotlim(2)], 'YLim', [plotlim(3) plotlim(4)], 'ZLim', [plotlim(5) plotlim(6)]);
    view(ax, 3);
    grid(ax, 'on');
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
    hold(ax, 'on');

    % for i = 1:N_anim
    %     plot3(ax, data{i}.x_des, data{i}.y_des, data{i}.z_des, '--k', 'LineWidth', 0.5, 'HandleVisibility','off');
    % end

    for i = 1:N_anim
        scatter3(ax, data{i}.x_des, data{i}.y_des, data{i}.z_des, 40, 'k*', 'HandleVisibility','off');
    end
    

    % Initialize trailing lines
    for i = 1:N_anim  
        trails{i} = plot3(0, 0, 0, '-', ...
                         'Color', trailColors{i}, 'LineWidth', 1.5, ...
                         'DisplayName', legends{i});

    end
    h_legend = legend('show');
    set(h_legend, 'Location', 'northoutside', 'Orientation','horizontal');
    title(h_legend, 'Quadcopter Animation');

    % Initialize quadcopter model
    for i = 1:N_anim
        % Create main transform for entire quadcopter
        transforms{i} = hgtransform('Parent', ax);
        
        % Arm parameters
        armLength = 0.75;
        armEnds = armLength*[1 0 0; -1 0 0; 0 1 0; 0 -1 0];

        theta = linspace(0, 2*pi, 20);  
                                
        xCircle = 0.2 * cos(theta);     
        yCircle = 0.2 * sin(theta);    
        % Create arms
        arms{i} = cell(1,4);
             
        for j = 1:4
            % Arm
            arms{i}{j} = plot3(ax, [0 armEnds(j,1)], [0 armEnds(j,2)], [0 0], ...
                'Color', colors{i}, 'LineWidth', 3, 'Parent', transforms{i});
            
            % Propeller
            fill3(ax, xCircle + armEnds(j,1), ...
                       yCircle + armEnds(j,2), ...
                       zeros(size(theta))+0.1, ...
                       'k', 'Parent', transforms{i});
        end
    end

    % Animation loop
    currentTime = 0;
    frameCount = 0;
    gifDelayTime = 0.05;
    trailData = cell(1,N_anim);

    while currentTime <= maxTime
        for i = 1:N_anim
            % Find closest time index
            [~, idx] = min(abs(data{i}.time - currentTime));
            % Store current position for trail
            trailData{i} = [trailData{i}; data{i}.x(idx), data{i}.y(idx), data{i}.z(idx)];
            
            % Update trailing line
            set(trails{i}, 'XData', trailData{i}(:,1), ...
                          'YData', trailData{i}(:,2), ...
                          'ZData', trailData{i}(:,3));

            % Get position and orientation
            pos = [data{i}.x(idx), data{i}.y(idx), data{i}.z(idx)];
            roll = data{i}.roll(idx);
            pitch = data{i}.pitch(idx);
            yaw = data{i}.yaw(idx);
            
            % Create transformation matrix for entire quadcopter
            rotX = makehgtform('xrotate', roll);
            rotY = makehgtform('yrotate', pitch);
            rotZ = makehgtform('zrotate', yaw);
            trans = makehgtform('translate', pos);
            set(transforms{i}, 'Matrix', trans*rotZ*rotY*rotX);
            
        end
        
        % Update figure and capture frame
        drawnow;
        frameCount = frameCount + 1;
        title(ax, sprintf('Time: %.2f s', currentTime));
        
        % Capture frame for GIF
        frame = getframe(fig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        
        % Write to GIF
        if frameCount == 1
            imwrite(imind, cm, outputGif, 'gif', 'Loopcount', inf, 'DelayTime', gifDelayTime);
        else
            imwrite(imind, cm, outputGif, 'gif', 'WriteMode', 'append', 'DelayTime', gifDelayTime);
        end
       
        currentTime = currentTime + timeStep;
    end
    
    hold(ax, 'off');

end