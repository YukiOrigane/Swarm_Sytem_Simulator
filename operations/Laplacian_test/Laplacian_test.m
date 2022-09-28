

% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
addpath(genpath('C:\Users\origane\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

param = myParameters_COS; % パラメタの一括読み込み
%param.formation = "agents04_rect_1_4";
param.formation = "agents20_rect_1_20";

run("SecondOrderCOS_2d_20220808");  % シミュレーションの実施


%%%%%%%%%%%%%%%%% 解析
disp("解析開始")

%% 描画
figure
viewer = RobotWithCOSViewer(swarm,field);
viewer.robot_view = viewer.robot_view.analyzeGraphMode(10);
viewer.robot_view.spacialModeView(10,2,[1,1],"Linear");
hold on
x = swarm.sys_robot.x(:,1,1);
l = max(x)-min(x);
sig = sign(viewer.robot_view.V(1,2))*-1;
y = sin(pi/(l).*x);
y = normalize(y,'norm');
plot(x,sig*y,'--','LineWidth',1.5);

%% TODO : 差の比較
