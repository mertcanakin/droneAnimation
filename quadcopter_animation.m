%% Quadcopter animation script
% Description:This script was created to animate flight log
% In the example log file there is out struct and in this struct position
% and angle results can be found. If you ahve different format you can
% change "Extract Data" part according to your file.

% NOTE: Dont forget to change rotation matrix if you are using different
% one.

function quadcopter_animation()
    %% Load log file
    [filename, path] = uigetfile('*.mat', 'Select Log File');
    if filename == 0, return; end
    data = load(fullfile(path, filename));
    
    %% Gif options
    gif_filename = 'quadcopter_animation.gif';
    delay_time = 0.05;
    frame_step = 50;
   
    %% Extract data
    pos = data.out.actPos_m;
    des_pos = data.out.desPos_m;
    ang = data.out.actAng_deg;
    des_ang = data.out.desAng_deg;
    time = data.out.time_s;

    %% Quadcopter parameters
    arm_length = 1;
    prop_radius = 0.3;
    body_size = 0.15;
    axis_radius = 5; % 5m in each direction for 3D view 

    %% Create figure with subplots
    fig = figure('Color', 'white', 'Position', [50 50 1200 900]);
    
    % Subplot 1: 3D Animation
    ax1 = subplot(2,2,[1,3]);
    hold(ax1, 'on');
    grid(ax1, 'on');
    axis(ax1, 'equal');
    view(ax1, 3);
    xlabel(ax1, 'X (North)'); ylabel(ax1, 'Y (East)'); zlabel(ax1, 'Z (Down)');
    title(ax1, '3D Quadcopter Animation');
    
    % Subplot 2: Attitude Angles
    ax2 = subplot(2,2,2);
    hold(ax2, 'on');
    grid(ax2, 'on');
    title(ax2, 'Attitude Angles');
    xlabel(ax2, 'Time (s)'); ylabel(ax2, 'Angle (deg)');
    
    % Initialize angle plots
    roll_plot = plot(ax2, time(1), ang(1,1), 'r', 'DisplayName', 'Roll');
    des_roll_plot = plot(ax2, time(1), des_ang(1,1), 'r--', 'DisplayName', 'Des Roll');
    pitch_plot = plot(ax2, time(1), ang(1,2), 'g', 'DisplayName', 'Pitch');
    des_pitch_plot = plot(ax2, time(1), des_ang(1,2), 'g--', 'DisplayName', 'Des Pitch');
    yaw_plot = plot(ax2, time(1), ang(1,3), 'b', 'DisplayName', 'Yaw');
    legend(ax2, 'show', 'Location', 'best');
    
    %% Subplot 3: XY Position
    ax3 = subplot(2,2,4);
    hold(ax3, 'on');
    grid(ax3, 'on');
    axis(ax3, 'equal');
    title(ax3, 'XY Position');
    xlabel(ax3, 'X (North)'); ylabel(ax3, 'Y (East)');
    
    %% Initialize position plots
    pos_plot = plot(ax3, pos(1,1), pos(1,2), 'b', 'DisplayName', 'Actual');
    des_pos_plot = plot(ax3, des_pos(1,1), des_pos(1,2), 'r--', 'DisplayName', 'Desired');
    legend(ax3, 'show', 'Location', 'best');
    
    %% Initialize 3D visualization
    [arms, body, props,  trajectory] = ...
        init_3d_visualization(ax1, prop_radius, body_size, pos(1,:));
    
    %% Animation loop
    for k = 1:frame_step:length(time)

        % Rotation matrix (NED)
        phi = deg2rad(ang(k,1));
        theta = deg2rad(ang(k,2));
        psi = deg2rad(ang(k,3));
        
        Rz = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
        Ry = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
        Rx = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
        R = Rz * Ry * Rx;
        
        % Update 3D visualization
        update_3d_visualization(ax1, arms, body, props,  trajectory, pos, ang(k,:), R, arm_length, prop_radius, body_size, k);
        
        % Update attitude plots
        set(roll_plot, 'XData', time(1:k), 'YData', ang(1:k,1));
        set(des_roll_plot, 'XData', time(1:k), 'YData', des_ang(1:k,1));
        set(pitch_plot, 'XData', time(1:k), 'YData', ang(1:k,2));
        set(des_pitch_plot, 'XData', time(1:k), 'YData', des_ang(1:k,2));
        set(yaw_plot, 'XData', time(1:k), 'YData', ang(1:k,3));
        
        % Update XY position plot
        set(pos_plot, 'XData', pos(1:k,1), 'YData', pos(1:k,2));
        set(des_pos_plot, 'XData', des_pos(1:k,1), 'YData', des_pos(1:k,2));
        
        % Adjust XY plot limits if needed
        current_lim = axis(ax3);
        margin = 1;
        if pos(k,1) < current_lim(1)+margin || pos(k,1) > current_lim(2)-margin || pos(k,2) < current_lim(3)+margin || pos(k,2) > current_lim(4)-margin
            axis(ax3, [min([pos(:,1); des_pos(:,1)])-margin, max([pos(:,1); des_pos(:,1)])+margin, min([pos(:,2); des_pos(:,2)])-margin, max([pos(:,2); des_pos(:,2)])+margin]);
        end
        
        % Center 3D view on drone with 10m box
        axis(ax1, [pos(k,1)-axis_radius, pos(k,1)+axis_radius, ...
                   pos(k,2)-axis_radius, pos(k,2)+axis_radius, ...
                   pos(k,3)-axis_radius, pos(k,3)+axis_radius]);
        
        % Update figure title with time
        sgtitle(fig, sprintf('Quadcopter Animation | Time: %.2f s', time(k)));
        
        drawnow;
                % Capture frame for GIF (only after the first frame)
        if k == 1
            % First frame - initialize GIF
            frame = getframe(fig);
            [im, map] = rgb2ind(frame.cdata, 256);
            imwrite(im, map, gif_filename, 'gif', 'LoopCount', Inf, 'DelayTime', delay_time);
        else
            % Subsequent frames
            frame = getframe(fig);
            [im, map] = rgb2ind(frame.cdata, 256);
            imwrite(im, map, gif_filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
        end
    end
end

function [arms, body, props, trajectory] = ...
    init_3d_visualization(ax, prop_radius, body_size, init_pos)
    
    % Create quadcopter components
    % Arms (4 lines)
    arms = gobjects(4,1);
    for i = 1:4
        arms(i) = plot3(ax, [0 0], [0 0], [0 0], 'k', 'LineWidth', 3);
    end
    
    % Body (central sphere)
    [X,Y,Z] = sphere(10);
    body = surf(ax, X*body_size, Y*body_size, Z*body_size, ...
               'FaceColor', 'b', 'EdgeColor', 'none');
    
    % Propellers (4 disks)
    [X,Y,Z] = create_propeller(prop_radius);
    props = gobjects(4,1);
    for i = 1:4
        props(i) = surf(ax, X, Y, Z, 'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.7);
    end
    
    % Create trajectory line
    trajectory = plot3(ax, init_pos(1), init_pos(2), init_pos(3), 'm:', 'LineWidth', 1.5);
end

function update_3d_visualization(ax, arms, body, props, trajectory, pos, ang, R, arm_length, prop_radius, body_size, k)
    
    % Arm endpoints in body frame (NED)
    arm_ends = arm_length * [1 0 0; -1 0 0; 0 1 0; 0 -1 0];
    
    % Update arms and propellers
    for i = 1:4
        % Transform arm to world frame
        world_end = pos(k,:) + (R * arm_ends(i,:)')';
        set(arms(i), 'XData', [pos(k,1) world_end(1)], ...
                     'YData', [pos(k,2) world_end(2)], ...
                     'ZData', [pos(k,3) world_end(3)]);
        
        % Update propeller position and orientation
        [X,Y,Z] = create_propeller(prop_radius);
        for j = 1:numel(X)
            rotated = R * [X(j); Y(j); Z(j)];
            X(j) = rotated(1) + world_end(1);
            Y(j) = rotated(2) + world_end(2);
            Z(j) = rotated(3) + world_end(3);
        end
        set(props(i), 'XData', X, 'YData', Y, 'ZData', Z);
    end
    
    % Update body position
    [X,Y,Z] = sphere(10);
    set(body, 'XData', X*body_size + pos(k,1), ...
              'YData', Y*body_size + pos(k,2), ...
              'ZData', Z*body_size + pos(k,3));
    
    
    % Update trajectory
    set(trajectory, 'XData', pos(1:k,1), ...
                    'YData', pos(1:k,2), ...
                    'ZData', pos(1:k,3));
end

function [X,Y,Z] = create_propeller(radius)
    % Create a propeller disk with slight thickness
    theta = linspace(0, 2*pi, 20);
    r = linspace(0, radius, 3);
    [theta_grid, r_grid] = meshgrid(theta, r);
    X = r_grid .* cos(theta_grid);
    Y = r_grid .* sin(theta_grid);
    Z = zeros(size(X)) - 0.01; % Slight thickness
end