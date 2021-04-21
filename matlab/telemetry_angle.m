clear; clc;

s = serialport("COM8", 115200);

% Set carrige return to be the line ending character
configureTerminator(s, "CR");

iMax = 200;

% Logging variables
period = zeros(1, iMax);
ticks = zeros(1, iMax);
angLog = zeros(iMax, 3);
angEffLog = zeros(iMax, 3);
angErrLog = zeros(iMax, 3);

angEffpLog = zeros(iMax, 3);
angEffiLog = zeros(iMax, 3);
angEffdLog = zeros(iMax, 3);

% Control loop refernce
angRef = [0 0 0];

% PID constants
angKp = [15 15 0];
angKi = [0 0 0];
angKd = [0.0 0.0 0];

% Used in loop
angInte = [0 0 0];
angDerv = [0 0 0];
angLast = [0 0 0];
angEff = [0 0 0];

% throttle stuff
base = 300;
maxThrottle = 700;
maxAngle = 13;
stopped = 0;

effort = zeros(iMax, 4);

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

flush(s);

for i = 1:iMax
    % Read latest data from drone
    line = readline(s);
    data = double(split(line,','));
    
    if(length(data) < 4)
        data = zeros(4, 1);
    end
    
    % Extract values from packet
    period(i) = data(1);
    ticks(i) = sum(period);
    ang = data(2:4)';
   
    % CONTROL SYSTEM GOES HERE

    angErr = angRef - ang;
    
    angInte = angInte + angErr;
    angDerv = (angErr - angLast) / (period(i)*(1024/16000000));
    
    angEff = (angErr .* angKp) + (angInte .* angKi) + (angDerv .* angKd);
    
    angLast = angErr;
    
    motorEffort(1) = base - angEff(1) + angEff(2);
    motorEffort(2) = base + angEff(1) + angEff(2);
    motorEffort(3) = base - angEff(1) - angEff(2);
    motorEffort(4) = base + angEff(1) - angEff(2);
    
    motorEffort = uint16(motorEffort);
    
    if(i > 200 || abs(angErr(1)) > maxAngle || abs(angErr(2)) > maxAngle || stopped)
        motorEffort = uint16([48, 48, 48, 48]);
        stopped = 1;
    end
    
    motorEffort = max(48, min(maxThrottle, motorEffort));
    
    % log data
    angLog(i, :) = ang;
    angEffLog(i, :) = angEff;
    angErrLog(i, :) = angErr;
    angEffpLog(i, :) = (angErr .* angKp);
    angEffiLog(i, :) = (angInte .* angKi);
    angEffdLog(i, :) = (angDerv .* angKd);
    effort(i, :) = double(motorEffort);
    
    % Send data back downstream
    
    msb = bitshift(motorEffort, -8);
    lsb = motorEffort - (msb * 256);
    
    write(s, msb, 'uint8');
    write(s, lsb, 'uint8');
    write(s, 0x0D, 'uint8');
end

motorEffort = uint16([48, 48, 48, 48]);

msb = bitshift(motorEffort, -8);
lsb = motorEffort - (msb * 256);
    
write(s, msb, 'uint8');
write(s, lsb, 'uint8');
write(s, 0x0D, 'uint8');

clear s;

x = ticks .* (1024/16000000);

figure(1);
subplot(3,1,1);
plot(x, effort(:, 1),x, effort(:, 2),x, effort(:, 3),x, effort(:, 4));
legend("Effort A", "Effort B", "Effort C", "Effort D")
title("Motor Throttles");

subplot(3,1,2);
plot(x, angLog(:, 1), x, angEffLog(:, 1), x, angEffpLog(:, 1), x, angEffiLog(:, 1), x, angEffdLog(:, 1), x, angErrLog(:, 1));
legend("Angle", "Effort", "P Effort", "I Effort", "D Effort", "Error")
title("X Axis");

subplot(3,1,3);
plot(x, angLog(:, 2), x, angEffLog(:, 2), x, angEffpLog(:, 2), x, angEffiLog(:, 2), x, angEffdLog(:, 2), x, angErrLog(:, 2));
legend("Angle", "Effort", "P Effort", "I Effort", "D Effort", "Error");
title("Y Axis");
