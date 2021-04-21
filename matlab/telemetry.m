clear; clc;

s = serialport("COM8", 115200);

% Set carrige return to be the line ending character
configureTerminator(s, "CR");

iMax = 500;

period = zeros(1, iMax);
acc = zeros(iMax, 3);
gyr = zeros(iMax, 3);

accRef = [0 0 1];
gyrRef = [0 0 0];

accKp = [10 10 0];
gyrKp = [10 10 0];

base = 300;

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
    
    if(length(data) < 7)
        data = zeros(7, 1);
    end
    
    disp(line);
    
    % Extract values from packet
    period(i) = data(1);
    acc(i, :) = data(2:4)';
    gyr(i, :) = data(5:7)';
   
    % CONTROL SYSTEM GOES HERE

    gyrErr = gyrRef - gyr;
    gyrEff = gyrErr .* gyrKp;
    
    motorEffort = uint16([base + gyrEff(1) - gyrEff(2), base - gyrEff(1) - gyrEff(2), base + gyrEff(1) + gyrEff(2), base - gyrEff(1) + gyrEff(2)]);
    
    if(i > 350)
        motorEffort = uint16([48, 48, 48, 48]);
    end
    
    % Store effort
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

x = 1:iMax;

figure(1);
subplot(3,1,1);
plot(x, effort(:, 1),x, effort(:, 2),x, effort(:, 3),x, effort(:, 4));
legend("Effort A", "Effort B", "Effort C", "Effort D")

subplot(3,1,2);
plot(x, gyr(:, 1), x, gyr(:, 2), x, gyr(:, 3));
legend("Gyro X", "Gyro Y", "Gyro Z")

subplot(3,1,3);
plot(x, acc(:, 1), x, acc(:, 2), x, acc(:, 3));
legend("Acc X", "Acc Y", "Acc Z")

