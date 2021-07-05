
%addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
addpath(genpath('C:\Users\origa\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'))

Nt = 1000;
dt = 0.01;
t_vec = 0:dt:dt*Nt;


field = Fields(repmat([-5,5],2,1));

%run("agents20_random.m")
%run("agents20_rect_2_10_perturb.m")
%run("agents20_rect_4_5_perturb.m")
run("agents20_rect_4_5")
%run("agents20_rect_2_10")
Na = length(initial_position);

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

swarm = swarm.setInitialCondition([initial_position, zeros(Na,dim)]);


for t = 1:1
    swarm = swarm.calcGraphMatrices(t); % グラフ関連行列を計算してもらう
    %controller = controller.calcInput(t,swarm,ref);
    %swarm = swarm.update(t,controller.u);
end

viewer = collisionAvoidanceTermAnalyzerViewer(swarm,field);
viewer = viewer.calcCollisionAvoindaceTerm(1);
viewer = viewer.calcPotential(1,0.1,0.1,1.0);
figure
viewer.plotPotential();
figure
subplot(2,1,1)
viewer.plotGradient(1);
subplot(2,1,2)
viewer.plotGradient(2);

% 衝突回避力の視覚化

figure
viewer = collisionAvoidanceTermAnalyzerViewer(swarm,field);
viewer = viewer.calcCollisionAvoindaceTerm(1);
mode_list = [2,3,4,11,12,13];
viewer.plotForceVector(1,mode_list,[2,3],"Position");
figure
viewer.plotForceAbsolute();
figure
viewer.plotForceAbsoluteVsFreq();


% モードの視覚化
%{
figure
viewer = PlaneModeViewer(swarm,field);
viewer = viewer.analyzeGraphMode(1);
mode_list = [15,16,17,18,19,20];
viewer.spacialModeView(1,mode_list,[2,3],"Mesh");
figure
viewer.spatialSpectrumView();
figure
mode_list = [15,16,17,18,19,20];
viewer.spacialModeView(1,mode_list,[2,3],"Position");
%}


%figure
% モード対衝突回避力比較用
%{
viewer = collisionAvoidanceTermAnalyzerViewer(swarm,field);
viewer = viewer.calcCollisionAvoindaceTerm(1);
subplot(1,3,1)
viewer.spatialSpectrumView();
subplot(1,3,2)
viewer.plotForceAbsolute();
subplot(1,3,3)
viewer.plotForceAbsoluteVsFreq();
%}