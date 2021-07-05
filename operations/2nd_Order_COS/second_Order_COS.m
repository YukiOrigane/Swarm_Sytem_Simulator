
% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
%addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
addpath(genpath('C:\Users\origa\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

Nt = 1000;          %
dt = 0.01;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt; % 時間ベクトル

field = Fields(repmat([-5,5],2,1));

%run("agents20_rect_2_10")
run("agents50_rect_5_10")
Na = length(initial_position);

sp_dim = 2;         % 空間的な次元（≠状態変数の次元）
osc_dim = 2;        % 振動子の次元

rv = 1.5;           % 観測範囲（グラフ構成用）
Omega_0 = 1;
kappa = 1.0;
gamma = 0;

m = ones(Na,sp_dim);
d = ones(Na,sp_dim);
k = zeros(Na,sp_dim);
swarm = RobotWithCOS(Na,sp_dim,osc_dim,Nt,dt);
swarm = swarm.setGraphProperties(1:2,rv,false);
swarm.sys_robot = swarm.sys_robot.setMechanicalParameters(m,d,k);
swarm.sys_robot = swarm.sys_robot.setInitialCondition([initial_position, zeros(Na,2)]);
swarm.sys_cos = swarm.sys_cos.setInitialCondition(rand(Na,osc_dim));

osc_controller = COS_Second_Order_Controller();
osc_controller = osc_controller.setParam(Omega_0,kappa,gamma);
osc_controller = osc_controller.setLeaderNum(1);

for t = 1:Nt
    swarm = swarm.observe(t);
    osc_controller = osc_controller.calcInput(t,swarm.sys_cos);
    swarm = swarm.update(t,zeros(Na*2*sp_dim,1),[osc_controller.u_cos]);
end
figure
viewer = RobotWithCOSViewer(swarm,field);
viewer.cos_view.phaseGapPlot([1,3,5,7,9],1);
hold on
grid on
title("r_v = "+string(rv)+", \Omega_0 = "+string(Omega_0)+", \kappa = "+string(kappa)+", \gamma = "+string(gamma))
gca.FontSize = 11;
%viewer.basicTimePlotX(1);

hold off

%% animation setting
%animation_speed = 1;
anime = Animation(viewer);


%% animation

%anime = anime.play(@snap,[]);
%anime.save([],[]);

function snap(viewer,t)
    viewer.phasePositionPlot(t,false,"normal");
end

