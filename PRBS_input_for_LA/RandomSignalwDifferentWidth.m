% Kanan Roy
% chowkana@isu.edu
% Input for linear actuator model with different time steps and different
% width

close all 
clear all
clc

p = 50; % No. of different time-steps
r = 0.06 + (2.4 - 0.06)*rand(1,p) % Using 10% of settling time to 400% to generate required time-steps

X = [0];

for n = 1:(size(r,2))
    X(1,n+1) = X(1,n) + r(1,n); % creating a series of time for plotting
end
X
Y = 1 + (4 - 1)*rand(1,p+1) % generating different width values from 1 to 4

stairs(X,Y,'LineWidth',2,'Color','r')
xlabel('Time')
ylabel('Movement Amplitude')
title('Input sequence')

data_mat = [X; Y]
% xlswrite('Input',data_mat)