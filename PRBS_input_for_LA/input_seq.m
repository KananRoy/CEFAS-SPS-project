% Kanan Roy
% chowkana@isu.edu
% Actual input sequence used to generate particular prbs input for LA.
% Settling time: 0.7 sec
% Band: 20% to 300% of settling time
% Sampling time: 0.1 sec
% Max. stroke length: 6 inch
% Min. Stroke length: 0 inch
% PRBS input: +/- 10V (generated prbs reversed to get valid response)


clc 
close all
clear all
warning('off')
format bank

F = 1/.1; % Frequency of each element generated by PRBS method.
Timestep = 1/F; % Interval in sec.
Duration = 100; % Duration in sec.
Ts=0.7; % Settling time for linear actuator
N = ceil(Duration/Timestep)+1; % Total number of samples.
B = Timestep/(Ts*0.2); % 1/B is minimum number of samples assigned to per PRBS element 
% 20% of the settling time
dt = 1/F; % Time stamp for each sample in sec. 
Range = [-10 10];
S_N= 3*(Ts/Timestep); % 3 times the settling sample 
Band = [1/S_N B]; % [0 B]; idinput takes floor(B)
channel=1;

% Input

u_input = ones(N,channel).*idinput(N, 'prbs', Band, Range); % Empty output/Binary hence [0,1]
u_input = iddata([], u_input, dt);
u_in = u_input.u(:,1);
iter = size(u_in);
u_in_re = zeros(iter(1),1);
for n = 1:iter(1) % flipping the original sequence to get valid output from Linear actuator
    if u_in(n,1) == -10
        u_in_re(n,1) = +10;
    else
        u_in_re(n,1) = -10;
    end
end


time = Timestep*(0:Duration/Timestep)'; % timestep data to import in simulink
tt = timetable(seconds(time), u_in_re); % timetable data to import in simulink inport

figure()
subplot(2,1,1)
plot(u_input);
title('PRBS data');
subplot(2,1,2)
plot(time,u_in_re,'r')
title('PRBS input for LA')
xlabel('Time (sec)')
ylabel('Amplitude (V)')
% axis([-1,D+1, Range(1)-1,Range(2)+1])
grid on;



figure()
plot(time,out.simout)
title('Output from LA model')
axis([0 100 -.02 0.15])
xlabel('Time (sec)')
ylabel('Movement (meter)')
% t  = linspace(0,Duration-Timestep,N)';
% data = [t, u_in]';
% % Raw input data : You can save as .mat
% u_ = u_input.u(:,1);
% t  = linspace(0,D-T,N)';
% data = [t,u_];
% figure();
% plot(t,u_);
% axis([-1,D+1, Range(1)-1,Range(2)+1]);
% grid on;

% csvwrite('in_data.csv', data);
