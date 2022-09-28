
%%%%% 2次オシレータを持つ平面上のエージェント群のシミュレーションを実施するテンプレート
% 使い方：
% 0. SwarmSystemSimulatorのパスを通す
% 例)
% addpath(genpath('C:\Users\origane\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
% 1. paramに条件を設定したmyParameters_COSの実体を入れる
% 例) 
% param = myParameters_COS;
% param.formation = "agents20_rect_1_20";
% 2. こいつを走らせる
% run(SecondOrderCOS_2d_20220808);
% 3. swarmに計算後のRobotWithCOSの実体が返ってくるので，解析なりなんなりする

%% 定義

% 以下各種設定


Nt = param.Nt;          % シミュレーション長さ
dt = param.dt;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt;     % 時間ベクトル

field = Fields(repmat([-5,5],2,1)); % 移動空間の定義．10m×10mの２次元平面を生成

run(param.formation) % 初期位置の決定
Na = length(initial_position); % エージェント数の決定

sp_dim = 2;         % 空間的な次元（≠状態変数の次元）
osc_dim = 2;        % 振動子の次元

rv = param.rv;           % 観測範囲（グラフ構成用）

Omega_0 = param.Omega_0; % 振動子システムのパラメタ設定
kappa = param.kappa;
gamma = param.gamma;

m = ones(Na,sp_dim);    % ロボットの移動システムの設定
d = ones(Na,sp_dim);
k = zeros(Na,sp_dim);

% 群れシステムの定義
swarm = RobotWithCOS(Na,sp_dim,2,Nt,dt,"true");   % 振動子付きロボット．陽解法で計算，振動子は2階微分

swarm = swarm.setGraphProperties(1:2,rv,false);         % 観測範囲を設定
swarm.sys_robot = swarm.sys_robot.setMechanicalParameters(m,d,k);   % 移動システムのパラメタ反映
swarm.sys_robot = swarm.sys_robot.setInitialCondition([initial_position, zeros(Na,2)]); % 初期値の設定
swarm.sys_cos = swarm.sys_cos.setInitialCondition(zeros(Na,2));
swarm.sys_cos = swarm.sys_cos.setPeriodic(false);

% 制御器の定義
controller_1 = COS_Second_Order_Controller();
controller_1 = controller_1.setParam(Omega_0,kappa,gamma);
controller_1 = controller_1.setLeaderNum(1);
controller_1 = controller_1.setInpulseInput(100,2,30,1);

if param.scheme == "explicit"
    controller_1 = controller_1.setExplicit("true");
end

%% シミュレーション
%%%%%%%%%%%%%%%% シミュレーション本体

for t = 1:Nt
    
    swarm = swarm.observe(t);
    swarm.sys_robot = swarm.sys_robot.calcGraphMatrices(t);
    swarm.sys_cos.Lap = swarm.sys_robot.Lap;    % グラフラプラシアンの共有

    controller_1 = controller_1.calcInput(t,swarm.sys_cos);
    swarm = swarm.update(t,zeros(Na*2*sp_dim,1),[controller_1.u_cos]);
end
%%