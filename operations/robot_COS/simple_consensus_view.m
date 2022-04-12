
% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
%addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
addpath(genpath('C:\Users\origane\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

Nt = 100;          % シミュレーション時間
dt = 0.1;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt; % 時間ベクトル

field = Fields(repmat([-5,5],2,1)); % 空間の定義．
run("agents20_rect_4_5")    % エージェント配置の定義
%run("agents20_rect_2_10")
Na = length(initial_position);

sp_dim = 2;         % 空間的な次元（≠状態変数の次元）
osc_dim = 1;        % 振動子の次元

rv = 1.1;           % 観測範囲（グラフ構成用）
    
m = ones(Na,sp_dim);    % 質量の設定
d = ones(Na,sp_dim);    % ダンパの設定
k = zeros(Na,sp_dim);   % ばね係数の設定
swarm = RobotWithCOS(Na,sp_dim,osc_dim,Nt,dt);  % 結合振動子付き群れの定義
swarm = swarm.setGraphProperties(1:2,rv,false); % 状態量の次元と観測距離を設定
swarm.sys_robot = swarm.sys_robot.setMechanicalParameters(m,d,k);   % マスばねダンパの設定
swarm.sys_robot = swarm.sys_robot.setInitialCondition([initial_position, zeros(Na,sp_dim)]);    % 初期値の決定（位置）
swarm.sys_cos = swarm.sys_cos.setInitialCondition(2*pi*rand(Na,osc_dim));   % 初期値の決定（位相）

osc_controller = COS_Simple_Consensus();    % 位相振動子の制御はコンセンサス制御
osc_controller = osc_controller.setParam(2*pi/2, 0.7);  % 制御器のパラメタ設定．Omega_0, kappa

%% シミュレーション本体
for t = 1:Nt    % シミュレーションループ
    swarm = swarm.observe(t);   % 観測
    osc_controller = osc_controller.calcInput(t,swarm.sys_cos); % 入力決定
    swarm = swarm.update(t,zeros(Na*2*sp_dim,1),osc_controller.u_cos);  % 更新
end

%% 描画

% 位相プロット
figure
plot(t_vec, sin(swarm.sys_cos.x(:,:)))
xlabel("Time (s)")
ylabel("Phase of Agents")
title("Time Change of Phases")
grid on
ax = gca;
ax.FontSize = 11;

% 秩序パラメータプロット
figure
plot(t_vec, abs(mean(exp(1j*swarm.sys_cos.x(:,:)),1)));
xlabel("Time (s)")
ylabel("Value of Order Parameter")
title("Time Change of Order Parameter")
grid on
ax = gca;
ax.FontSize = 11;

%%
% 動画の描画
F = [];
figure
for t = 1:Nt
    clc;
    str_time = sprintf('経過時間は %f [s]です．\n',t*dt);
    disp(str_time)

    if strcmp(get(gcf,'currentcharacter'),'q')  % qキーで停止
        break;
    end
    viewer = PlaneBasicViewer(swarm.sys_robot, field);
    viewer.plotPosition(t,sin(swarm.sys_cos.x(:,t)));
    drawnow
    F = [F, getframe];%軸含めない
end

%% 動画の保存

    v = VideoWriter("consensus.mp4",'MPEG-4');
    open(v);
    writeVideo(v,F);
    close(v);

