
addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

Nt = 1000;
dt = 0.01;
t_vec = 0:dt:dt*Nt;

%field_x = [-4,4];
%field_y = [-4,4];
field = Fields(repmat([-4,4],2,1));

Na = 20;

dim = 2;

rv = 1.5;

m = ones(Na,dim);
d = ones(Na,dim);
k = zeros(Na,dim);
ref = zeros(Na,dim);
swarm = LinearTwoIntegerSystemAgents(Na,dim,Nt,dt);
swarm = swarm.setGraphProperties(1:2,rv,false);
swarm = swarm.setMechanicalParameters(m,d,k);
controller = LinearFeedBackController();
k = 1;
controller = controller.setGainMatrix(k*eye(Na*dim));

initial_position = [
    2,1.5;
    2,0.5;
    2,-0.5;
    2,-1.5;
    1,1.5;
    1,0.5;
    1,-0.5;
    1,-1.5;
    0,1.5;
    0,0.5;
    0,-0.5;
    0,-1.5;
    -1,1.5;
    -1,0.5;
    -1,-0.5;
    -1,-1.5;
    -2,1.5;
    -2,0.5;
    -2,-0.5;
    -2,-1.5;];
swarm = swarm.setInitialCondition(initial_position,zeros(Na,dim));


for t = 1:1
    swarm = swarm.calcGraphMatrices(t); % グラフ関連行列を計算してもらう
    %controller = controller.calcInput(t,swarm,ref);
    %swarm = swarm.update(t,controller.u);
end

[V,D] = eig(swarm.Lap);
mode_list = [1,2,3,4,5,6];
cnt = 0;
for mode = mode_list
    cnt = cnt+1;
    subplot(2,3,cnt);
    viewer = PlaneBasicViewer(swarm,field);
    viewer.plotPosition(1,V(:,mode));
    title(string(mode)+" th mode ; \lambda_"+string(mode)+" = " +string(D(mode,mode)));
end
%plot(t_vec, permute(swarm.x(:,1,:),[1,3,2]));

