
param = myParameters_weighted; % パラメタの一括読み込み

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

swarm_1 = swarm_1.setGraphProperties(1:2,rv,false);         % 観測範囲を設定
swarm_1.sys_robot = swarm_1.sys_robot.setMechanicalParameters(m,d,k);   % 移動システムのパラメタ反映
swarm_1.sys_robot = swarm_1.sys_robot.setInitialCondition([initial_position, zeros(Na,2)]); % 初期値の設定
swarm_1.sys_cos = swarm_1.sys_cos.setInitialCondition(zeros(Na,2));
swarm_1.sys_cos = swarm_1.sys_cos.setPeriodic(false);

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
    
    swarm_1 = swarm_1.observe(t);
    swarm_1.sys_robot = swarm_1.sys_robot.calcGraphMatrices(t);
    swarm_1.sys_cos.Lap = swarm_1.sys_robot.Lap;    % グラフラプラシアンの共有

    controller_1 = controller_1.calcInput(t,swarm_1.sys_cos);
    swarm_1 = swarm_1.update(t,zeros(Na*2*sp_dim,1),[controller_1.u_cos]);
end
%%
%%%%%%%%%%%%%%%%% 解析
disp("解析開始")
clf %図を消す
type_list = ["counting","dist1","dist2","dist3"];
folder_name = "0802_dist_weight_norm/naname/";
legend_list = ["$w_{ij}\in\{1,0\}$","$1/\|x_{ij}\|$","$1/\|x_{ij}\|^2$","$1/\|x_{ij}\|^3$"];
%color_list = ["#0072BD","#D95319","#EDB120","#7E2F8E"];
%color_list = [[0 0.4470 0.7410]; [0.8500 0.3250 0.0980]; [0.9290 0.6940 0.1250]; [0.4940 0.1840 0.5560]];
fig_1st_mode = figure;
fig_2nd_mode = figure;
Lap_list = zeros(Na,Na,length(type_list));
d_list = zeros(length(type_list),Na);

for i=1:length(type_list)
    
    swarm_1.sys_robot = swarm_1.sys_robot.calcGraphMatrices(t,type_list(i));
    L = swarm_1.sys_robot.Lap;
    %L = swarm_1.sys_robot.Lap_norm;
    Lap_list(:,:,i) = L;
    [V,D] = eig(L);
    d = diag(D);
    d_list(i,:) = sort(d);
    viewer = RobotWithCOSViewer(swarm_1,field);
    viewer.robot_view.V = V;
    viewer.robot_view.D = D;
    viewer.robot_view.d = d;

    figure(fig_1st_mode)
    subplot(1,length(type_list),i)
    viewer.robot_view.spacialModeView(10,2,[1,1],"Position");
    title(legend_list(i)+" : "+string(sqrt(d(2)))+ " rad/m", 'Interpreter','latex');
    grid on
    figure(fig_2nd_mode)
    subplot(1,length(type_list),i)
    viewer.robot_view.spacialModeView(10,3,[1,1],"Position");
    title(legend_list(i)+" : "+string(sqrt(d(3)))+ " rad/m", 'Interpreter','latex');
    grid on
end

saveas(fig_1st_mode,folder_name+param.formation+"_eigenmodes_1st.png")
saveas(fig_2nd_mode,folder_name+param.formation+"_eigenmodes_2nd.png")

%% 固有値プロット
figure
plot(d_list(2:4,:).','*','LineWidth',1.5);
hold on
plot(d_list(1,:).','*','LineWidth',1.5, 'Color','#7E2F8E');
legend('$1/\|x_{ij}\|$','$1/\|x_{ij}\|^2$','$1/\|x_{ij}\|^3$','$w_{ij}\in\{1,0\}$', 'Interpreter','latex', 'FontSize', 12);
grid on;
xlim([0,21])
xlabel("Order of Eigenvalues");
ylabel("Value of Eigenvalues");
ax = gca;
ax.FontSize = 12;
saveas(gcf,folder_name+param.formation+"_eigenvalues.png")

% 相関プロット
figure
plot(d_list(1,:).',d_list(2:4,:).','*','LineWidth',1.5);
hold on
grid on;
plot(d_list(1,:).',d_list(1,:).', 'LineStyle','--', 'Color', '#7E2F8E');
legend('$1/\|x_{ij}\|$','$1/\|x_{ij}\|^2$','$1/\|x_{ij}\|^3$','$w_{ij}\in\{1,0\}$', 'Interpreter','latex', 'FontSize', 12);
%xlim([1,20])
xlabel("Eigenvalues of Counting")
ylabel("Value of Eigenvalues")
ax = gca;
ax.FontSize = 12;
saveas(gcf,folder_name+param.formation+"_eigenvalues_relative.png")

%% 描画
%{
figure
viewer = RobotWithCOSViewer(swarm_1,field);
viewer.robot_view.V = V;
viewer.robot_view.D = D;
viewer.robot_view.d = d;
%viewer.robot_view = viewer.robot_view.analyzeGraphMode(10);
%viewer.robot_view.spacialModeView(10,2:4,[3,1],"Linear");
viewer.robot_view.spacialModeView(10,2:3,[1,2],"Position");
grid on
%}
%%
figure
viewer2 = PlaneBasicViewer(swarm_1.sys_robot,field);
viewer2.plotPositionNumber(1,[],true);

%%
%{
figure
r = 0.1:0.05:2.0;
w_counting = r<param.rv;
w_1 = 1./r;
w_2 = 1./(r.^2);
w_3 = 1./(r.^3);
plot(r, [w_1; w_2; w_3; w_counting],'LineWidth',1.5);
legend('$1/\|x_{ij}\|$','$1/\|x_{ij}\|^2$','$1/\|x_{ij}\|^3$','$w_{ij}\in\{1,0\}$', 'Interpreter','latex', 'FontSize', 12);
xlabel("$\|x_{ij}\|$", 'Interpreter','latex');
ylabel("$w_{ij}$", 'Interpreter','latex');
ax = gca;
ax.FontSize = 12;
grid on;
ylim([0,2]);
%}
