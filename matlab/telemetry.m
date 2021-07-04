clear; clc;

s = serialport("COM8", 115200);

% Set carrige return to be the line ending character
configureTerminator(s, "CR");

iMax = 1000;

% Logging variables
periodLog = zeros(1, iMax);
ticksLog = zeros(1, iMax);
accLog = zeros(iMax, 3);
gyrLog = zeros(iMax, 3);
angleLog = zeros(iMax, 3);

angleEffortLog = zeros(iMax, 3);
gyroEffortLog = zeros(iMax, 3);

% Control variables
angle = [0 0 0]; % Preset becuase found from intergral of gyro

angleIntergral = [0 0 0];
angleLast = [0 0 0];

angleRef = [0 0 0]; % Reference angle

angleKp = [1 1 0];
angleKi = [0 0 0];
angleKd = [0.5 0.5 0];

gyroIntergral = [0 0 0];
gyroLast = [0 0 0];

gyroKp = [1 1 0];
gyroKi = [0 0 0];
gyroKd = [0 0 0];

base = 300;

motorEffortLog = zeros(iMax, 4);

% Engage the motors
tic;
while toc < 4
    motorEffort = uint16([48, 48, 48, 48]);

    msb = bitshift(motorEffort, -8);
    lsb = motorEffort - (msb * 256);

    write(s, msb, 'uint8');
    write(s, lsb, 'uint8');
    write(s, 0x0D, 'uint8');
end

disp("Start");
flush(s);

for i = 1:iMax
    % Read latest data from drone
    line = readline(s);
    data = double(split(line,','));
    
    if(length(data) < 7)
        data = zeros(7, 1);
    end

    % Extract values from packet
    period = data(1);
    acc = data(2:4)';
    gyr = data(5:7)';
    
    % CONTROL SYSTEM GOES HERE
    
    time = (period * (1024/16000000));
    
    if(time == 0)
        time = 0.01;
    end
    
    angle = angle + (gyr .* time); % Intergrate gyro for angle
    
    % OUTER ANGLE CONTROL LOOP
    % Compute error, intergral and derivitve
    angleError = angleRef - angle;
    
    angleIntergral = angleIntergral + (angleError .* time); % intergrate
    angleDerivative = (angleError - angleLast) ./ time; % differnetiate
    angleLast = angleError;
    
    angleEffort = (angleError .* angleKp) + (angleIntergral .* angleKi) + (angleDerivative .* angleKd);
        
    % INNER GYRO CONTROL LOOP
    gyroError = angleEffort - gyr;
    
    gyroIntergral = gyroIntergral + (gyroError .* time); % intergrate
    gyroDerivative = (gyroError - gyroLast) ./ time; % differnetiate
    gyroLast = gyroError;
    
    gyroEffort = (gyroError .* gyroKp) + (gyroIntergral .* gyroKi) + (gyroDerivative .* gyroKd);
    
    % APPLY EFFORTS TO MOTORS
    motorEffort(1) = base - gyroEffort(1) - gyroEffort(2);
    motorEffort(2) = base + gyroEffort(1) - gyroEffort(2);
    motorEffort(3) = base + gyroEffort(1) + gyroEffort(2);
    motorEffort(4) = base - gyroEffort(1) + gyroEffort(2);
    
    % Log all the values
    periodLog(i) = period; % Ticks this loop ran for
    ticksLog(i) = sum(periodLog); % Ticks since starting
    
    accLog(i, :) = acc; % Raw acc data
    gyrLog(i, :) = gyr; % Raw gyr data
    angleLog(i, :) = angle;
    
    angleEffortLog(i, :) = angleEffort;
    gyroEffortLog(i, :) = gyroEffort;
    motorEffortLog(i, :) = motorEffort;
    
    motorEffort = uint16(motorEffort);
    
    % Send data back downstream
    
    msb = bitshift(motorEffort, -8);
    lsb = motorEffort - (msb * 256);
    
    write(s, msb, 'uint8');
    write(s, lsb, 'uint8');
    write(s, 0x0D, 'uint8');
end

disp("Stop");

% Kill the motors
motorEffort = uint16([0, 0, 0, 0]);

msb = bitshift(motorEffort, -8);
lsb = motorEffort - (msb * 256);
    
write(s, msb, 'uint8');
write(s, lsb, 'uint8');
write(s, 0x0D, 'uint8');

% Disconnect
clear s;

x = ticksLog .* (1024/16000000);

figure(1);
subplot(3,1,1);
plot(x, motorEffortLog(:, 1),x, motorEffortLog(:, 2),x, motorEffortLog(:, 3),x, motorEffortLog(:, 4));
legend("Effort A", "Effort B", "Effort C", "Effort D")
title("Motor Throttles");

subplot(3,1,2);
plot(x, accLog(:, 1), x, gyrLog(:, 1), x, angleLog(:, 1), x, angleEffortLog(:, 1), x, gyroEffortLog(:, 1));
legend("Acceleration (g)", "Gyro (degrees/s)", "Angle (degrees)", "Angle Effort", "Gyro Effort")
title("X Axis");

subplot(3,1,3);
plot(x, accLog(:, 2), x, gyrLog(:, 2), x, angleLog(:, 2), x, angleEffortLog(:, 2), x, gyroEffortLog(:, 2));
legend("Acceleration (g)", "Gyro (degrees/s)", "Angle (degrees)", "Angle Effort", "Gyro Effort");
title("Y Axis");
