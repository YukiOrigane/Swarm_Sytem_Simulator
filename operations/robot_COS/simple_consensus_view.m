
% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

Nt = 100;          % 
dt = 0.1;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt; % 時間ベクトル

field = Fields(repmat([-5,5],2,1));
run("agents20_rect_4_5")
%run("agents20_rect_2_10")
Na = length(initial_position);

sp_dim = 2;         % 空間的な次元（≠状態変数の次元）
osc_dim = 1;        % 振動子の次元

rv = 1.1;           % 観測範囲（グラフ構成用）
    
m = ones(Na,sp_dim);
d = ones(Na,sp_dim);
k = zeros(Na,sp_dim);
swarm = RobotWithCOS(Na,sp_dim,osc_dim,Nt,dt);
swarm = swarm.setGraphProperties(1:2,rv,false);
swarm.sys_robot = swarm.sys_robot.setMechanicalParameters(m,d,k);
swarm.sys_robot = swarm.sys_robot.setInitialCondition([initial_position, zeros(Na,sp_dim)]);
swarm.sys_cos = swarm.sys_cos.setInitialCondition(2*pi*rand(Na,osc_dim));

osc_controller = COS_Simple_Consensus();
osc_controller = osc_controller.setParam(2*pi/2, 0.7);

for t = 1:Nt
    swarm = swarm.observe(t);
    osc_controller = osc_controller.calcInput(t,swarm.sys_cos);
    swarm = swarm.update(t,zeros(Na*2*sp_dim,1),osc_controller.u_cos);
end

figure

plot(t_vec, sin(swarm.sys_cos.x(:,:)))
grid on

figure
plot(t_vec, abs(mean(exp(1j*swarm.sys_cos.x(:,:)),1)));


%%
F = [];
figure
for t = 1:Nt
    clc;
    str_time = sprintf('経過時間は %f [s]です．\n',t*dt);
    disp(str_time)

    if strcmp(get(gcf,'currentcharacter'),'q')  % キーで停止
        break;
    end
    viewer = PlaneBasicViewer(swarm.sys_robot, field);
    viewer.plotPosition(t,sin(swarm.sys_cos.x(:,t)));
    drawnow
    F = [F, getframe];%軸含めない
end

%% 動画の保存

    v = VideoWriter("consensus_2.mp4",'MPEG-4');
    open(v);
    writeVideo(v,F);
    close(v);

