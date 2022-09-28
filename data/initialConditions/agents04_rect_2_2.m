initial_position = zeros(4,2);
initial_position(:,2) = [-0.5,-0.5,0.5,0.5];
initial_position(:,1) = [-0.5,0.5,-0.5,0.5];
initial_position(:,1) = flip(initial_position(:,1));
initial_position(:,2) = flip(initial_position(:,2));