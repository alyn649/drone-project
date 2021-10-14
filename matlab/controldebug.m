clear s;

s = serialport("COM8", 115200, 'timeout', 60);

configureTerminator(s, "CR");

data = cell(1000, 1);

for i = 1:1000
    data{i} = readline(s);
end

clear s; % close serial port

y = zeros(1000, 6);

for i = 2:1000
    y(i, :) = double(split(data{i},','));
    
end

t = linspace(0,5,1000);

subplot(3,2,1);
plot(t, y(:,4));
title("Gyro X");
ylabel("Degrees / Second");

subplot(3,2,2);
plot(t, y(:,1));
title("effort X");

subplot(3,2,3);
plot(t, y(:,5));
title("Gyro Y");
ylabel("Degrees / Second");

subplot(3,2,4);
plot(t, y(:,2));
title("effort Y");

subplot(3,2,5);
plot(t, y(:,6));
title("Gyro Z");
ylabel("Degrees / Second");

subplot(3,2,6);
plot(t, y(:,3));
title("effort Z");
