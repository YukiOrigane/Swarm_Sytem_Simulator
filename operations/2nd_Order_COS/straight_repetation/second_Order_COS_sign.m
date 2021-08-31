
% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
%addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
addpath(genpath('C:\Users\origa\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

%if exist('param')==0
    param = Parameters;
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
swarm.sys_cos = swarm.sys_cos.setInitialCondition(zeros(Na,osc_dim));
swarm.sys_cos = swarm.sys_cos.setPeriodic(false);
osc_controller = COS_Second_Order_Controller();
osc_controller = osc_controller.setParam(Omega_0,kappa,gamma);
osc_controller = osc_controller.setLeaderNum(1);
osc_controller = osc_controller.setInpulseInput(100,2,3,1);
if param.scheme == "explicit"
    osc_controller = osc_controller.setExplicit("true");
end
%osc_controller = osc_controller.setWhiteNoise(param.tau,param.T,param.I_0,param.num);
%osc_controller = osc_controller.setPeriodicInput(param.tau,param.T,param.I_0,param.num,param.omega_f);

%%%%%%%%%%%%%%%% シミュレーション本体

for t = 1:Nt
    swarm = swarm.observe(t);
    swarm.sys_robot = swarm.sys_robot.calcGraphMatrices(t);
    swarm.sys_cos.Lap = swarm.sys_robot.Lap;    % グラフラプラシアンの共有
    osc_controller = osc_controller.calcInput(t,swarm.sys_cos);
    swarm.sys_cos.u_histry(:,1,t) = osc_controller.f;
    swarm = swarm.update(t,zeros(Na*2*sp_dim,1),[osc_controller.u_cos]);
end

%%%%%%%%%%%%%%%%% プロットここから

%% 時系列プロット
figure
subplot(1,2,1)
viewer = RobotWithCOSViewer(swarm,field);
viewer.cos_view.phaseGapPlot([1,2,3,4],1);
hold on
grid on
title("r_v = "+string(rv)+", \Omega_0 = "+string(Omega_0)+", \kappa = "+string(kappa)+", \gamma = "+string(gamma))
xlabel("time (s)")
ylabel("Phase Gap \phi_j-\phi_1")
ax = gca;
ax.FontSize = 11;

hold off

%% 時系列プロットその２
%{
viewer = RobotWithCOSViewer(swarm,field);
viewer = viewer.setParamList(param);
viewer.cos_view.phasePhiPlot([1,2,3,4],Omega_0);
hold on
grid on
title("r_v = "+string(rv)+", \Omega_0 = "+string(Omega_0)+", \kappa = "+string(kappa)+", \gamma = "+string(gamma))
ax = gca;
ax.FontSize = 11;

hold off
%}

%% 運動エネルギー履歴
subplot(1,2,2)
viewer.cos_view.virtualEnergyVIewer();
%plot(t_vec, 1/2*permute(sum(swarm.sys_cos.x(:,2,:).^2 + param.kappa*swarm.sys_cos.x(:,1,:).^2,1),[3,1,2]))
title("Sum of Energy (\Delta t = "+string(dt)+", \kappa = "+string(kappa)+", \gamma = "+string(gamma)+")")
xlabel("time (s)")
ylabel("Sum of Energy")
grid on
ax = gca;
ax.FontSize = 11;

%% 入力履歴
figure
subplot(1,2,1)
viewer.cos_view.inputTimePlotLinear(1,1);
title("Input Histry")
xlabel("time (s)")
ylabel("Input")
grid on
ax = gca;
ax.FontSize = 11;

subplot(1,2,2)
viewer.cos_view.inputSpectrumPlotLinear(1,1);
title("Input Spectrum")
grid on
ax = gca;
ax.FontSize = 11;
udat = [t_vec(1:Nt).', permute(swarm.sys_cos.u_histry(1,1,:),[1,3,2]).'];
[p1,f1] = pspectrum(udat(:,2),udat(:,1));
loglog(2*pi*f1,p1)
grid on
%% スペクトラムプロット
figure
viewer = viewer.spectrumAnalyze(100,osc_controller);
%h = bodeplot(viewer.sys_phi(1),viewer.sys_phi(2),viewer.sys_phi(3),viewer.sys_phi(4));
%h = bodeplot(viewer.sys_phi(1:4));
h = bodeplot(viewer.sys_phi(1));
setoptions(h,'PhaseVisible','off');
xlim([0.01,1000])
%title("\Phi_1","\Phi_2","\Phi_3","\Phi_4")
%legend("\Phi_1","\Phi_2","\Phi_3","\Phi_4")
grid on
ax = gca;
ax.FontSize = 11;

figure
viewer = viewer.spectrumAnalyze(100,osc_controller);
%h = bodeplot(viewer.sys_xi(1),viewer.sys_xi(2),viewer.sys_xi(3),viewer.sys_xi(4));
h = bodeplot(viewer.sys_xi(2:5));
setoptions(h,'PhaseVisible','off');
%legend("\xi_1","\xi_2","\xi_3","\xi_4")
grid on

viewer.robot_view = viewer.robot_view.analyzeGraphMode(10);
viewer.robot_view.spatialSpectrumView;

viewer.robot_view = viewer.robot_view.analyzeGraphMode(10);
viewer.robot_view.spacialModeView(10,2:5,[2,2],"Linear");
grid on
%{
%% 番号リスト生成
figure
viewer2 = PlaneBasicViewer(swarm.sys_robot,field);
viewer2.plotPositionNumber(1,[],true);
%}
figure
viewer.robot_view = viewer.robot_view.analyzeGraphMode(10);
viewer.robot_view.spacialModeView(10,1:3,[3,1],"Linear");
grid on

title("3rd mode")
xlabel("robot number")
ylabel("value of eigenvector")

%% animation setting
%animation_speed = 1;
anime = Animation(viewer);


%% animation

anime = anime.play(@snap4,5);
%anime.save([],[]);

function snap(viewer,t)
    viewer.phasePositionPlot(t,false,"gap");
    colorbar
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
    ylim([-0.1,0.1]);
    xlim([-5,5]);
    grid on
    xlabel("Position (m)")
    ylabel("$\Phi_i-\bar{\Phi}$",'Interpreter','latex')
    text(4,0.101,"t = "+string(t*viewer.sys.dt),'FontSize',14)
end