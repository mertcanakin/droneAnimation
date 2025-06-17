# Modular Quadcopter Animation Script

This MATLAB script provides a flexible and modular way to create animations of multiple quadcopters simultaneously. It can visualize and compare different flight logs.

## Features

- **Multi-quadcopter Visualization**: Can animate multiple quadcopters simultaneously
- **GIF Export**: Automatically saves the animation as a GIF file
- **3D Visualization**: Full 3D rendering with proper perspective and grid

## Usage

1. **Set up log files**:
```matlab
% Single quadcopter
logFiles = {'log_SMC.mat'};

% Multiple quadcopters
logFiles = {'log_PID.mat', 'log_SMC.mat', 'log_INDI.mat'};
```

2. **Configure visualization parameters**:
```matlab
% Set legends and colors
legends = {'PID', 'SMC', 'INDI'};
colors = {'r', 'g', 'b'};
trailColors = {[1 0.5 0.5], [0.5 1 0.5], [0.5 0.5 1]}; 
```

3. **Run the animation**:
The script will automatically:
- Load all specified log files
- Calculate appropriate plot limits
- Create the animation
- Save it as 'quadcopter_anim.gif'

## Log File Requirements

Each log file should contain a structure `log` with the following fields:
- `time`: Time vector
- `x_log`, `y_log`, `z_log`: Actual position
- `x_des_log`, `y_des_log`, `z_des_log`: Desired position
- `phi_log`, `theta_log`, `psi_log`: Euler angles

For example x data for log_PID.mat: log.x_log(:)
You can change log structure type below according to your flight log.

```matlab
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
    data{i}.pitch  = data{i}.log.theta_log(:);
    data{i}.yaw    = -data{i}.log.psi_log(:);
end
```
## Example Output

The script generates a GIF animation (`quadcopter_anim.gif`) showing:
- Each quadcopter's movement in 3D space
- Trailing paths showing the flight history
- Desired trajectory paths
