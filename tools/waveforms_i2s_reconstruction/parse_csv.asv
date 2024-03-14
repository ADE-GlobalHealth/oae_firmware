clear all; clf;
data = readmatrix("i2s_v6.csv")

data_clean = rmmissing(data)
time = data_clean(:,1);
left = data_clean(:,2);
right = data_clean(:,3);


figure(1)
plot(time, left, linewidth = 0.25);
xlabel("Time"); ylabel("Left channel")

interp1(linspace(min(time), max(time), ))
% sound()

% figure(2)
% plot(time, right);
% xlabel("Time"); ylabel("Right channel")