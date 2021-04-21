clear; clc;

s = serialport("COM8", 115200);

% Set carrige return to be the line ending character
configureTerminator(s, "CR");

iMax = 5000;

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
    
    motorEffort(1) = 150;
    motorEffort(2) = 150;
    motorEffort(3) = 150;
    motorEffort(4) = 150;
    
    motorEffort = uint16(motorEffort);
    
    motorEffort = max(48, motorEffort);

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
