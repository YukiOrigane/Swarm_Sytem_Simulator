
% 20台のロボットによる

addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

Nt = 1000;
dt = 0.01;
t_vec = 0:dt:dt*Nt;

field_x = [-5,5];
field_y = [-5,5];

Na = 20;

dim = 2;

rv = 1.0;

m = ones(Na,dim);
d = ones(Na,dim);
k = zeros(Na,dim);
ref = zeros(Na,dim);
swarm = LinearTwoIntegerSystemAgents(Na,dim,Nt,dt);
swarm = swarm.setGraphProperties(1,rv,false);
swarm = swarm.setMechanicalParameters(m,d,k);
controller = LinearFeedBackController();
k = 1;
controller = controller.setGainMatrix(k*eye(Na*dim));

swarm = swarm.setInitialCondition(rand(Na,dim),zeros(Na,dim));

for t = 1:Nt
    swarm = swarm.observe(t);
    controller = controller.calcInput(t,swarm,ref);
    swarm = swarm.update(t,controller.u);
end

plot(t_vec, permute(swarm.x(:,1,:),[1,3,2]));

