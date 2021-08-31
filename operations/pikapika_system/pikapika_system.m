
% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
%addpath(genpath('C:\Users\origa\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

%if exist('param')==0
    param = PikaPikaParameters();
%end

Nt = param.Nt;          %
dt = param.dt;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt; % 時間ベクトル

field = Fields(repmat([-5,5],2,1));

%run("agents20_rect_2_10")
run(param.formation)
%run("agents50_rect_2_25")
Na = length(initial_position);

sp_dim = 2;         % 空間的な次元（≠状態変数の次元）
osc_dim = 1;        % 振動子の次元

rv = param.rv;           % 観測範囲（グラフ構成用）
Omega_0 = param.Omega_0;
kappa = param.kappa;
gamma = param.gamma;
m = param.m*ones(Na,sp_dim);
d = param.d*ones(Na,sp_dim);
k = zeros(Na,sp_dim);

swarm = RobotWithCOS(Na,sp_dim,osc_dim,Nt,dt);
swarm = swarm.setGraphProperties(1:2,rv,false);
swarm.sys_robot = swarm.sys_robot.setMechanicalParameters(m,d,k);
swarm.sys_robot = swarm.sys_robot.setInitialCondition([initial_position, zeros(Na,2)]);

swarm.sys_robot = swarm.sys_robot.setMechanicalParameters(m,d,k);
swarm.sys_robot = swarm.sys_robot.setInitialCondition([initial_position, zeros(Na,2)]);
swarm.sys_cos = swarm.sys_cos.setInitialCondition(zeros(Na,osc_dim));
swarm.sys_cos = swarm.sys_cos.setPeriodic(false);
osc_controller = COS_PikaPika_Controller();
osc_controller = osc_controller.setParam(param.I_0,Omega_0, kappa);
osc_controller = osc_controller.setPeriodicPurturbation(param.lambda, param.T_f);
robot_controller = VirtualForceController();
robot_controller = robot_controller.setParam(param.k_P, param.k_F, param.k_O, param.d_O, param.r_C);
cbf = CBF("true");
cbf = cbf.setSecondOrderLinear(param.r_s, param.gamma, param.T);

for t = 1:Nt
    swarm = swarm.observe(t);
    swarm.sys_robot = swarm.sys_robot.calcGraphMatrices(t);
    swarm.sys_cos.Lap = swarm.sys_robot.Lap;    % グラフラプラシアンの共有
    osc_controller = osc_controller.calcInput(t,swarm);
    %swarm.sys_cos.u_histry(:,1,t) = osc_controller.f;
    robot_controller = robot_controller.calcInput(t, swarm);
    cbf = cbf.calcInput(t,swarm.sys_robot,robot_controller.u);
    swarm = swarm.update(t, [zeros(2*Na,1); cbf.u], osc_controller.u_cos);
end

%%
viewer = RobotWithCOSViewer(swarm,field);
anime = Animation(viewer);
anime = anime.play(@snap,5);
%anime.save([],[]);

function snap(viewer,t)
    viewer.phasePositionPlot(t,false,"gap");
    colorbar
end