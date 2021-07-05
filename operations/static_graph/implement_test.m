
% 20台のロボットがそれぞれ原点に線形FBするデモ

% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

Nt = 1000;          % ロボットの台数
dt = 0.01;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt; % 時間ベクトル

field_x = [-5,5];   % フィールド横幅
field_y = [-5,5];   % フィールド縦幅

Na = 20;            % 台数

sp_dim = 2;         % 空間的な次元（≠状態変数の次元）

rv = 1.0;           % 観測範囲（グラフ構成用）
    
m = ones(Na,sp_dim);
d = ones(Na,sp_dim);
k = zeros(Na,sp_dim);
ref = zeros(Na,sp_dim*2);
swarm = LinearTwoIntegerSystemAgents(Na,sp_dim,Nt,dt);
swarm = swarm.setGraphProperties(1:2,rv,false);
swarm = swarm.setMechanicalParameters(m,d,k);
controller = LinearFeedBackController();
FB_gain = 1;
controller = controller.setGainMatrix( FB_gain*eye(Na*sp_dim*2));

swarm = swarm.setInitialCondition([rand(Na,sp_dim), zeros(Na,sp_dim)]);

for t = 1:Nt
    swarm = swarm.observe(t);
    controller = controller.calcInput(t,swarm,ref);
    swarm = swarm.update(t,controller.u);
end

plot(t_vec, permute(swarm.x(:,1,:),[1,3,2]));

