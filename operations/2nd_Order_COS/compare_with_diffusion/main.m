
param = myParameters; % パラメタの一括読み込み

% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
addpath(genpath('C:\Users\origane\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

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
swarm_1 = RobotWithCOS(Na,sp_dim,2,Nt,dt,"true");   % 振動子付きロボット．陽解法で計算，振動子は2階微分
swarm_2 = RobotWithCOS(Na,sp_dim,1,Nt,dt);   % 振動子付きロボット．陽解法で計算，振動子は1階微分

swarm_1 = swarm_1.setGraphProperties(1:2,rv,false);         % 観測範囲を設定
swarm_1.sys_robot = swarm_1.sys_robot.setMechanicalParameters(m,d,k);   % 移動システムのパラメタ反映
swarm_1.sys_robot = swarm_1.sys_robot.setInitialCondition([initial_position, zeros(Na,2)]); % 初期値の設定
swarm_1.sys_cos = swarm_1.sys_cos.setInitialCondition(zeros(Na,2));
swarm_1.sys_cos = swarm_1.sys_cos.setPeriodic(false);
swarm_1.sys_cos = swarm_1.sys_cos.setCalcModeCoordinate();

swarm_2 = swarm_2.setGraphProperties(1:2,rv,false);         % 観測範囲を設定
swarm_2.sys_robot = swarm_2.sys_robot.setMechanicalParameters(m,d,k);   % 移動システムのパラメタ反映
swarm_2.sys_robot = swarm_2.sys_robot.setInitialCondition([initial_position, zeros(Na,2)]); % 初期値の設定
swarm_2.sys_cos = swarm_2.sys_cos.setInitialCondition(zeros(Na,1));
swarm_2.sys_cos = swarm_2.sys_cos.setPeriodic(false);
swarm_2.sys_cos = swarm_2.sys_cos.setCalcModeCoordinate();

% 制御器の定義
controller_1 = COS_Second_Order_Controller();
controller_1 = controller_1.setParam(Omega_0,kappa,gamma);
controller_1 = controller_1.setLeaderNum(1);
controller_1 = controller_1.setInpulseInput(100,2,30,1);

controller_2 = COS_Simple_Consensus();    % 位相振動子の制御はコンセンサス制御
controller_2 = controller_2.setParam(0,kappa);  % 継承にしなかったが故の過ち
controller_2 = controller_2.setLeaderNum(1);
controller_2 = controller_2.setInpulseInput(100,2,30,1);

if param.scheme == "explicit"
    controller_1 = controller_1.setExplicit("true");
end

%% シミュレーション
%%%%%%%%%%%%%%%% シミュレーション本体

for t = 1:Nt
    
    swarm_1 = swarm_1.observe(t);
    swarm_1.sys_robot = swarm_1.sys_robot.calcGraphMatrices(t);
    swarm_1.sys_cos.Lap = swarm_1.sys_robot.Lap;    % グラフラプラシアンの共有

    swarm_2 = swarm_2.observe(t);
    swarm_2.sys_robot = swarm_2.sys_robot.calcGraphMatrices(t);
    swarm_2.sys_cos.Lap = swarm_2.sys_robot.Lap;    % グラフラプラシアンの共有

    controller_1 = controller_1.calcInput(t,swarm_1.sys_cos);
    controller_2 = controller_2.calcInput(t,swarm_2.sys_cos);
    swarm_1 = swarm_1.update(t,zeros(Na*2*sp_dim,1),[controller_1.u_cos]);
    swarm_2 = swarm_2.update(t,zeros(Na*2*sp_dim,1),[controller_2.u_cos]);
    swarm_2.sys_cos.u_histry(:,:,t) = controller_2.u_cos;
    swarm_1.sys_cos = swarm_1.sys_cos.calcModeCoordinate(t);
    swarm_2.sys_cos = swarm_2.sys_cos.calcModeCoordinate(t);
end

%%%%%%%%%%%%%%%%% 解析
disp("解析開始")

%% 描画
figure
viewer = RobotWithCOSViewer(swarm_1,field);
viewer.cos_view.phaseMeanPlot(1:4); 
ax = gca;
ax.FontSize = 13;
grid on
xlabel("Time (s)")
ylabel("Phase")
legend("$i=1$","$i=2$","$i=3$",'Interpreter','latex')

figure
viewer = RobotWithCOSViewer(swarm_2,field);
viewer.cos_view.phaseMeanPlot(1:3); 
ax = gca;
ax.FontSize = 13;
grid on
xlabel("Time (s)")
ylabel("Phase")
legend("$i=1$","$i=2$","$i=3$",'Interpreter','latex')

%% 描画2 (モードプロット)
figure
viewer = RobotWithCOSViewer(swarm_1,field);
viewer.cos_view.phaseModePlot(1:3); 
ax = gca;
ax.FontSize = 13;
grid on
xlabel("Time (s)")
ylabel("Phase")
legend("$i=1$","$i=2$","$i=3$",'Interpreter','latex')

figure
viewer = RobotWithCOSViewer(swarm_2,field);
viewer.cos_view.phaseModePlot(1:3); 
ax = gca;
ax.FontSize = 13;
grid on
xlabel("Time (s)")
ylabel("Mode")
legend("$i=1$","$i=2$","$i=3$",'Interpreter','latex')

%% ナンバーリスト作成
%{
%% 番号リスト生成
figure
viewer2 = PlaneBasicViewer(swarm_2.sys_robot,field);
viewer2.plotPositionNumber(1,[],true);
%}

%% モデル解析
ch = 2;

u = zeros(Na,1,Nt);
u(1,1,101) = 30;
u = pagemtimes(swarm_2.sys_cos.P.',u);
u = permute(u,[1,3,2]);
y = permute(swarm_2.sys_cos.xi(:,:,1:Nt),[1,3,2]);
z = iddata(y.',u.',param.dt);

plot(z(:,ch,ch));
grid on
ax = gca;
ax.FontSize = 11;

%GS = spa(z(:,ch,ch));
%h = bodeplot(GS);
mtf = tfest(z(:,ch,ch), 1, 0);

expected_pole = param.kappa*swarm_2.sys_cos.Lambda(ch,ch);
disp("予測値："+string(expected_pole));
disp("観測値："+string(mtf.Denominator(2)));
disp("誤差："+string((expected_pole-mtf.Denominator(2))/expected_pole));