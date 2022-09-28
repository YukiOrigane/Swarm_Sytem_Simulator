load('narrow_data.mat');
%T = 10*50;
T = 30*50;
%T = 80*50;
initial_position = zeros(20,2);
initial_position(:,1) = x(:,T);
initial_position(:,2) = y(:,T);
clear x y T