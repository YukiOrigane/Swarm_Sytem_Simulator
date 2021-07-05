
%addpath(genpath('C:\Users\origa\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'))
addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'))



Nt = 1000;
dt = 0.01;
t_vec = 0:dt:dt*Nt;


field = Fields(repmat([-5,5],2,1));

%run("agents20_rect_4_5")
run("agents20_rect_2_10")
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
swarm_DOF = Agents(Na,dim*2,Nt,dt);   % 集団自由度に座標変換したシステム

controller = SDOF_Controller();
G = zeros(Na*dim);
r=4;
mu = 2;
%G(1,1) = 1;
%ref(1,1) = r;%1
G(mu,mu) = 1;
ref(mu,1) = r;%1
controller = controller.setGainMatrix(G);

swarm = swarm.setInitialCondition([initial_position, zeros(Na,dim)]);

swarm = swarm.calcEignExpansion(1);
P = swarm.P;
swarm_DOF.x(:,:,1) = P.'*swarm.x(:,1:4,1); % \xi = P.'x

for t = 1:Nt
    swarm = swarm.calcGraphMatrices(t); % グラフ関連行列を計算してもらう
    controller = controller.calcInput(t,swarm,ref);
    swarm = swarm.update(t,controller.u);
    swarm = swarm.calcEignExpansion(t);
    P = swarm.P;
    swarm_DOF.x(:,:,t+1) = P.'*swarm.x(:,1:4,t+1); % \xi = P.'x
end

%% View

viewer = PlaneModeViewer(swarm, field);
viewer2 = GeneralPlotViewer(swarm, []);
viewer3 = GeneralPlotViewer(swarm_DOF, []);

viewer2.timeTransitionView(1,1:20);
title("x-t")
grid on
set(gca,'Fontsize',12);
figure
viewer3.timeTransitionView(1,1:3);
title("\xi-t")
grid on
set(gca,'Fontsize',12);
line([0,10],[r,r])

%% animation setting
%animation_speed = 1;
anime = Animation(viewer);


%% animation

anime = anime.play(@snap,[]);
%anime.save([],[]);

function snap(viewer,t)
    viewer = viewer.analyzeGraphMode(t);
    viewer.spacialModeView(t,2,[1,1],"Position");
end

%{
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
%}

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