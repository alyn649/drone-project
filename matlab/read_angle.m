clear; clc;

s = serialport("COM8", 115200);

% Set carrige return to be the line ending character
configureTerminator(s, "CR");

iMax = 500;

period = zeros(1, iMax);
ang = zeros(iMax, 3);

motorEffort = uint16([0, 0, 0, 0]);

flush(s);

for i = 1:iMax
    % Read latest data from drone
    line = readline(s);
    data = double(split(line,','));
    
    if(length(data) < 4)
        data = zeros(4, 1);
    end
    
    %disp(line);
    
    % Extract values from packet
    period(i) = data(1);
    ang(i, :) = data(2:4)';
   
    % CONTROL SYSTEM GOES HERE

    
    % Send data back downstream
    
    msb = bitshift(motorEffort, -8);
    lsb = motorEffort - (msb * 256);
    
    write(s, msb, 'uint8');
    write(s, lsb, 'uint8');
    write(s, 0x0D, 'uint8');
end

clear s;

x = 1:iMax;

figure(1);
% subplot(3,1,1);
% plot(x, effort(:, 1),x, effort(:, 2),x, effort(:, 3),x, effort(:, 4));
% legend("Effort A", "Effort B", "Effort C", "Effort D")
% 
% subplot(3,1,2);
plot(x, ang(:, 1), x, ang(:, 2), x, ang(:, 3));
legend("Angle X", "Angle Y", "Angle Z")
% 
% subplot(3,1,3);
% plot(x, acc(:, 1), x, acc(:, 2), x, acc(:, 3));
% legend("Acc X", "Acc Y", "Acc Z")

