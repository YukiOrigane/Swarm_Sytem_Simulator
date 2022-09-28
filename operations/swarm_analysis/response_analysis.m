
% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
%addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
addpath(genpath('C:\Users\origane\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

%if exist('param')==0
    param = Parameters;
%end

Nt = param.Nt;          %
dt = param.dt;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt; % 時間ベクトル

field = Fields(repmat([-3,3],2,1));

%run("agents20_rect_2_10")
run(param.formation)
%run("agents50_rect_2_25")
Na = length(initial_position);

sp_dim = 2;         % 空間的な次元（≠状態変数の次元）
osc_dim = 2;        % 振動子の次元

rv = param.rv;           % 観測範囲（グラフ構成用）
Omega_0 = param.Omega_0;
kappa = param.kappa;
gamma = param.gamma;

m = ones(Na,sp_dim);
d = ones(Na,sp_dim);
k = zeros(Na,sp_dim);
%swarm = RobotWithCOS(Na,sp_dim,osc_dim,Nt,dt);
if param.scheme == "explicit"
    swarm = RobotWithCOS(Na,sp_dim,osc_dim,Nt,dt,"true");
else
    swarm = RobotWithCOS(Na,sp_dim,osc_dim,Nt,dt);
end
swarm = swarm.setGraphProperties(1:2,rv,false);
swarm.sys_robot = swarm.sys_robot.setMechanicalParameters(m,d,k);
swarm.sys_robot = swarm.sys_robot.setInitialCondition([initial_position, zeros(Na,2)]);
%swarm.sys_cos = swarm.sys_cos.setInitialCondition(rand(Na,osc_dim));
if param.osc_IC == "random"
    swarm.sys_cos = swarm.sys_cos.setInitialCondition(2*pi*rand(Na,osc_dim));
else
    swarm.sys_cos = swarm.sys_cos.setInitialCondition(zeros(Na,osc_dim));
end
swarm.sys_cos = swarm.sys_cos.setPeriodic(false);
swarm.sys_cos = swarm.sys_cos.setCalcModeCoordinate();  % モード座標計算を有効化
osc_controller = COS_Second_Order_Controller();
osc_controller = osc_controller.setParam(Omega_0,kappa,gamma);
osc_controller = osc_controller.setLeaderNum(1);
%osc_controller = osc_controller.setInpulseInput(100,2,3,1);
osc_controller = osc_controller.setInpulseInput(param.tau,param.T,param.I_0,param.num);
if param.scheme == "explicit"
    osc_controller = osc_controller.setExplicit("true");
end
osc_controller = osc_controller.setNormalize(param.is_normalize); % 正規化ラプラシアンの使用
%osc_controller = osc_controller.setWhiteNoise(param.tau,param.T,param.I_0,param.num);
%osc_controller = osc_controller.setPeriodicInput(param.tau,param.T,param.I_0,param.num,param.omega_f);

%%%%%%%%%%%%%%%% シミュレーション本体

for t = 1:Nt
    swarm = swarm.observe(t);

    swarm.sys_robot = swarm.sys_robot.calcGraphMatrices(t);
    swarm.sys_cos.Deg = swarm.sys_robot.Deg;
    swarm.sys_cos.Lap = swarm.sys_robot.Lap;    % グラフラプラシアンの共有
    osc_controller = osc_controller.calcInput(t,swarm.sys_cos);
    swarm.sys_cos.u_histry(:,1,t) = osc_controller.f;
    swarm = swarm.update(t,zeros(Na*2*sp_dim,1),[osc_controller.u_cos]);
    swarm.sys_cos = swarm.sys_cos.calcEnergy(t,param.kappa);
    swarm.sys_cos = swarm.sys_cos.calcModeCoordinate(t);    % モード座標計算
end


%%%%%%%%%%%%%%%%% プロットここから

%% 時系列プロット
figure
%subplot(1,2,1)
viewer = RobotWithCOSViewer(swarm,field);
%viewer.cos_view.phaseGapPlot([1,2,3,4],1);
%viewer.cos_view.phaseMeanPlot([1,2,3]);
plt = viewer.cos_view.phaseDotPlot(1:Na);
for i = 1:Na
    plt(i).LineWidth = 1.0;
end
hold on
grid on
title("Velocity of Phase : r_v = "+string(rv)+", \kappa = "+string(kappa)+", \gamma = "+string(gamma))
xlabel("time (s)")
ylabel("Velocity of Phase")
xlim([0 20]);
ax = gca;
ax.FontSize = 11;
hold off
saveas(gcf,'velocity_of_phase.png')

%% 時系列プロットその２
figure
plt = viewer.cos_view.phaseModeDotPlot(1:Na);
for i = 1:Na
    plt(i).LineWidth = 1.0;
end
hold on
grid on
title("Velocity of Mode : r_v = "+string(rv)+", \kappa = "+string(kappa)+", \gamma = "+string(gamma))
xlabel("time (s)")
ylabel("Velocity of Mode")
xlim([0 20]);
ax = gca;
ax.FontSize = 11;
hold off
saveas(gcf,'velocity_of_mode.png')

%% 時系列プロットその3
figure
plt = viewer.cos_view.phaseModePlot(1:Na);
for i = 1:Na-1
    plt(i).LineWidth = 1.0;
end
hold on
grid on
title("Time Change of Mode : r_v = "+string(rv)+", \kappa = "+string(kappa)+", \gamma = "+string(gamma))
xlabel("time (s)")
ylabel("Mode")
xlim([0 20]);
ax = gca;
ax.FontSize = 11;
hold off
saveas(gcf,'timeChange_of_mode.png')

%% 位相差プロット
%{
figure
viewer = RobotWithCOSViewer(swarm,field);
%viewer.cos_view.phaseGapPlot([1,2,3,4],1);
viewer.cos_view.phaseGapPlot(2,1);
hold on
viewer.cos_view.phaseGapPlot(3,2);
grid on
title("r_v = "+string(rv)+", \Omega_0 = "+string(Omega_0)+", \kappa = "+string(kappa)+", \gamma = "+string(gamma))
xlabel("time (s)")
legend("2-1","3-2");
ax = gca;
ax.FontSize = 11;

hold off
%}
%% 番号リスト生成
figure
viewer2 = PlaneBasicViewer(swarm.sys_robot,field);
%viewer2.plotPositionNumber(1,[],true);
viewer2.plotPosition(1,[],true);
for i = 1:Na
    text(swarm.sys_robot.x(i,1,1)-0.3,swarm.sys_robot.x(i,2,1)+0.2,string(i),'FontSize',14)
end
%}
figure
viewer.robot_view = viewer.robot_view.analyzeGraphMode(10);
%viewer.robot_view.spacialModeView(10,2:4,[3,1],"Linear");
viewer.robot_view.spacialModeView(10,2:4,[1,3],"Position");
grid on

title("3rd mode")
xlabel("robot number")
ylabel("value of eigenvector")

%% animation setting
%animation_speed = 1;
anime = Animation(viewer);


%% animation

%anime = anime.play(@snap4,5);
%{
figure
subplot(1,3,1)
anime.frameShot(@snap,500,false);
subplot(1,3,2)
anime.frameShot(@snap,2500,false);
subplot(1,3,3)
anime.frameShot(@snap,4500,false);
%}
%anime.save([],[]);

function snap(viewer,t)
    viewer.phasePositionPlot(t,true,"gap");
    colorbar
    text(1,-2,"t = "+string(t*viewer.sys.dt),'FontSize',14)
end

function snap2(viewer,t)
    viewer.phaseMeshPlot(t);
    zlim([-0.05,0.05]);
    colorbar
end

function snap3(viewer,t)
    viewer.phaseImagePlot(t);
    caxis([-0.01,0.01]);
    colorbar
end

function snap4(viewer,t)
    viewer.phaseLinearPlot(t,1:viewer.sys.N);
    %ylim([-0.1,0.1]);
    ylim([-10,10]);
    xlim([-5,5]);
    grid on
    xlabel("Position (m)")
    ylabel("$\Phi_i-\bar{\Phi}$",'Interpreter','latex')
    text(4,0.101,"t = "+string(t*viewer.sys.dt),'FontSize',14)
end