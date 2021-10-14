clear s;

s = serialport("COM8", 115200, 'timeout', 60);

configureTerminator(s, "CR");

data = cell(1000, 1);

for i = 1:1000
    data{i} = readline(s);
    disp(i);
end

clear s; % close serial port

y = zeros(1000, 9);

for i = 2:1000
    y(i, :) = double(split(data{i},','));
    
end

t = linspace(0,5,1000);

subplot(3,3,1);
plot(t, y(:,7));
title("Angle X");
ylabel("Degrees");

subplot(3,3,2);
plot(t, y(:,4));
title("Gyro X");
ylabel("Degrees / Second");

subplot(3,3,3);
plot(t, y(:,1));
title("Accel X");
ylabel("G - Force");

subplot(3,3,4);
plot(t, y(:,8));
title("Angle Y");
ylabel("Degrees");

subplot(3,3,5);
plot(t, y(:,5));
title("Gyro Y");
ylabel("Degrees / Second");

subplot(3,3,6);
plot(t, y(:,2));
title("Accel Y");
ylabel("G - Force");

subplot(3,3,7);
plot(t, y(:,9));
title("Angle Z");
ylabel("Degrees");

subplot(3,3,8);
plot(t, y(:,6));
title("Gyro Z");
ylabel("Degrees / Second");

subplot(3,3,9);
plot(t, y(:,3));
title("Accel Z");
ylabel("G - Force");
