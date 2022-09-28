
param = myParameters_cheeger; % パラメタの一括読み込み

% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
%addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
addpath(genpath('C:\Users\origane\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
%addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator')
%% 定義

% 以下各種設定


Nt = 2;%param.Nt;          % シミュレーション長さ
dt = param.dt;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt;     % 時間ベクトル

field = Fields(repmat([-8,8],2,1)); % 移動空間の定義．10m×10mの２次元平面を生成

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
swarm_1.sys_cos = swarm_1.sys_cos.setCalcModeCoordinate();

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
    swarm_1.sys_cos = swarm_1.sys_cos.calcModeCoordinate(t);
end

%%%%%%%%%%%%%%%%% 解析
disp("解析開始")

Lap = swarm_1.sys_cos.Lap;
[P,Sigma] = eig(Lap);
disp(Sigma);
Lap_p = swarm_1.sys_cos.Lap*inv(swarm_1.sys_robot.Deg);
[P_p,Sigma_p] = eig(Lap_p);
disp(Sigma_p);
name = "agents20_T00";

%% 浦川先生の定理による計算を評価
figure
G = graph(swarm_1.sys_robot.Adj);
diamG = max(distances(G),[],'all');
m_inftyG = max(swarm_1.sys_robot.Deg,[],'all');
s = 2:diamG;
Urakawa_criterion = m_inftyG-2*sqrt(m_inftyG-1)*cos(2*s*pi./(diamG+2*s));
Urakawa_criterion_p = 1-2*sqrt(m_inftyG-1)/m_inftyG*cos(2*s*pi./(diamG+2*s))
sigma = sort(diag(Sigma)); 
sigma_p = sort(diag(Sigma_p));
plot(s,sigma(s),'*','MarkerSize',12)
hold on
plot(s,Urakawa_criterion,'o','MarkerSize',12)
xlim([1,diamG+1])
ylim([0,10])
grid on

xlabel("Order of Eigenvalues")
ylabel("Value of Eigenvalues")
legend("Actual Values", "Urakawa's Criterion",'FontSize',12)
ax = gca;
ax.FontSize = 12;
uraFig = gcf;
saveas(gcf,name+'.png')

%% ナンバーリスト作成

%% 番号リスト生成
figure
viewer2 = PlaneBasicViewer(swarm_1.sys_robot,field);
viewer2.plotPositionNumber(1,[],true);
saveas(gcf,name+'_shape.png')

%% チーガー定数計算．多分やばいので注意
Deg = swarm_1.sys_robot.Deg;    % 面倒なので各行列を取り出し
Adj = swarm_1.sys_robot.Adj;
phi_G = 100;                    % チーガー定数
phi_C_list = [];
S_G = {};                       % チーガー定数を与えるノード集合
for k = 1:ceil(Na/2)                  % 選び出しの個数
    S = nchoosek(1:Na,k);    % ノードの部分集合
    for i=1:size(S,1)
        %S(i,:) = [2,3];
        bar_S = setdiff(1:Na,S(i,:));   % V\Sの取り出し
        %m = min( sum(Deg(S(i,:),:),'all'), sum(Deg(bar_S,:),'all') ); % SとSの排他部分の体積を比較して小さいほう
        m = sum(Deg(S(i,:),:),'all');
        num_partial_S = sum( Adj(S(i,:),bar_S) ,'all');                 % 境界枝の本数
        phi_C = num_partial_S/m;    % Sに対応するチーガー商
        %phi_C_list = [phi_C_list phi_C];
        if phi_C<phi_G
            phi_G = phi_C;          % チーガー商が最小を更新したらチーガー定数を置き換え
            S_G = S(i,:);                % チーガー定数に対応するノード集合を更新
        end
    end
end

ue_2 = m_inftyG+sqrt(m_inftyG^2-phi_G^2);
shita_2 = m_inftyG-sqrt(m_inftyG^2-phi_G^2);
ue = 2*phi_G;
shita = phi_G^2/2;


% 描画
figure
v = zeros(Na,1);
v(S_G) = 1;
%v(S(20000,:)) = 1;
viewer2.plotPosition(1,v,'true');   % Sに対応するエージェントを表示

figure(uraFig)
plot([2,2],[shita,ue], '-o')
disp("推移"+string(shita)+", "+sigma_p(2)+", "+string(ue))
disp("そのまま"+string(shita_2)+", "+sigma(2)+", "+string(ue_2)+", "+string(Urakawa_criterion(2)));

%%
R = [0.001385	0.013639	0.10526	0.00069264	0.024623	3.9993	0.54206;
0.002551	0.037389	0.14286	0.00085046	0.097897	5.9991	1.9176;
0.013007	0.13613	0.32258	0.0032531	0.38197	7.9967	3.5824;
0.015571	0.15968	0.35294	0.0025957	0.63301	11.9974	6.6365;
];

R(:,8) = [0.1456 0.4122 0.6402 0.8706];

%y_list = ["1\times 20台", "2\times 10台", "4\times 5台", "ball"];
subplot(2,1,1)

y_list = R(:,5);
semilogy(y_list, R(:,4), '-o',y_list, R(:,5), '-*', y_list, R(:,6), '-o', y_list, R(:,7), '-^')
xlabel("Value of Eigenvalues")
legend("Cheeger inf", "Actual Values", "Cheeger sup", "Urakawa's Criterion",'FontSize',12,'NumColumns',2)
ax = gca;
ax.FontSize = 12;
title("Eigenvalue Evaluation of \Delta_A")
grid on

subplot(2,1,2)
y_list = R(:,2);
semilogy(y_list, R(:,1), '-o', y_list, R(:,2), '-*', y_list, R(:,3), '-o', y_list, R(:,8), '-^');
xlabel("Value of Eigenvalues")
legend("Cheeger inf2", "Actual Values", "Cheeger sup2","Urakawa's Criterion2",'FontSize',12,'NumColumns',2)
ax = gca;
ax.FontSize = 12;
title("Eigenvalue Evaluation of \Delta_P")
ylim([10^-4, 10^2])
grid on

