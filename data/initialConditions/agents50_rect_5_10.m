initial_position = zeros(50,2);
[X,Y] = meshgrid(-2:1:2,-4.5:1:4.5);
initial_position(:,1) = reshape(X,50,1);
initial_position(:,2) = reshape(Y,50,1);