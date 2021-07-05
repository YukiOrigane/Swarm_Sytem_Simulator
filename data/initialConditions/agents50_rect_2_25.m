initial_position = zeros(50,2);
[X,Y] = meshgrid(-0.5:1:0.5,-4.8:0.4:4.8);
initial_position(:,1) = reshape(X,50,1);
initial_position(:,2) = reshape(Y,50,1);