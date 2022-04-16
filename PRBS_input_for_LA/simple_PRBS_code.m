% Kanan Roy
% chowkana@isu.edu
% This code will generate a binary pseudo random sequence of numbers based
% on the assigned probability. p is probability of switching


p = 0.5; % switching probability

rng('default');

num = 100; % no. of steps

rng(1) % specific seed to generate the same random sequence
random_num = rand(1,num);

t = linspace(0,num-1,num);
x = zeros(1,num);
for n = 1:num
    if random_num(1,n)>= 0.5 % +/- 10 will be assigned based on the 
        % value of random number being higher or lower then the probability
        x(1,n) = +10
    else
        x(1,n) = -10
    end
end
stairs(t,x,'LineWidth',2,'Color','r')
xlabel('Time')
ylabel('Movement Amplitude')
title('Input sequence')
data_mat = [t; x]