
% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
%addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
addpath(genpath('C:\Users\origane\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

Nt = 1000;          % ロボットの台数
dt = 0.01;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt; % 時間ベクトル

field = Fields(repmat([-5,5],2,1));

Na = 20;            % 台数

sp_dim = 2;         % 空間的な次元（≠状態変数の次元）
osc_dim = 1;        % 振動子の次元

rv = 1.0;           % 観測範囲（グラフ構成用）
    
m = ones(Na,sp_dim);
d = ones(Na,sp_dim);
k = zeros(Na,sp_dim);
swarm = RobotWithCOS(Na,sp_dim,osc_dim,Nt,dt);
swarm = swarm.setGraphProperties(1:2,rv,false);
swarm.sys_robot = swarm.sys_robot.setMechanicalParameters(m,d,k);
swarm.sys_cos = swarm.sys_cos.setInitialCondition(rand(Na,osc_dim));

osc_controller = COS_LF_Controller();
osc_controller = osc_controller.setParam(10,11,0.1);
osc_controller = osc_controller.setLeaderNum(1);

for t = 1:Nt
    swarm = swarm.observe(t);
    osc_controller = osc_controller.calcInput(t,swarm.sys_cos);
    swarm = swarm.update(t,zeros(Na*2*sp_dim,1),osc_controller.u_cos);
end

viewer = CoupledOscillatorViewer(swarm.sys_cos,field);
viewer.basicTimePlotX(1);