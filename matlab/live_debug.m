clear s;

s = serialport("COM4", 115200, 'timeout', 60);

configureTerminator(s, "CR");

data = cell(1000, 1);

tic;
for i = 1:1000
    data{i} = readline(s);
    disp(i);
end
t = toc;

clear s; % close serial port

cols = length(double(split(data{1},',')));

y = zeros(1000, cols);

for i = 2:1000
    y(i, :) = double(split(data{i},','));
    
end

t = linspace(0,t,999);

for i = 1:cols
    subplot(6,2,i);
    plot(t, y(2:end,i));
    title("Val " + num2str(i));
end

