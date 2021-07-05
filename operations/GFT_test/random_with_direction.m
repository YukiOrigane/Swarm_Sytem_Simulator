
% フォルダ全体を検索パスに入れる．パスの前半部分は要修正
addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

Nt = 1000;          % シミュレーションカウント
dt = 0.1;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt; % 時間ベクトル

field = Fields(repmat([-5,5],2,1));

Na = 100;            % 台数

sp_dim = 2;         % 空間的な次元（≠状態変数の次元）

rv = 2;           % 観測範囲（グラフ構成用）

swarm = SimpleOneIntegerSystemAgents(Na, sp_dim, Nt, dt);   % 一階の超単純システム
swarm = swarm.setGraphProperties(1:2,rv,false);
swarm = swarm.setInitialCondition(10*rand(Na,sp_dim)-5);
controller = RandomWithPatternController();
area_constraint = [-5 5; -5 5];

for t = 1:Nt
   swarm = swarm.areaConstraintJump(t, area_constraint);
    swarm = swarm.observe(t);
    controller = controller.calcInput(t,swarm);
    swarm = swarm.update(t, controller.u);
end
%%
viewer = PlaneBasicViewer(swarm, field);
viewer2 = PlaneModeViewer(swarm, field);
theta = atan2(controller.u(101:200),controller.u(1:100));
viewer2 = viewer2.analyzeGraphMode(t);
bar_theta = (viewer2.V).'*theta;

figure
viewer2.spacialModeView(t,1:9,[3,3],'Position');
figure
quiver(swarm.x(:,1,t),swarm.x(:,2,t), cos(theta),  sin(theta));
%dec_swarm = Agents(Na,3,1,dt);
%dec_swarm.x(:,1:2,1,dt) = 

figure
plot(1:100, abs(bar_theta))

tr = zeros(100,1);
tr(3)= 1;tr(13) = 1;tr(19) = 1;
theta_2 = (viewer2.V)*(tr.*bar_theta);
figure
quiver(swarm.x(:,1,t),swarm.x(:,2,t), cos(theta_2),  sin(theta_2));

figure
subplot(1,2,1)
plot(swarm.x(:,1,t),theta,'*')
subplot(1,2,2)
plot(swarm.x(:,1,t),theta_2,'*')

%% animation setting
%animation_speed = 1;
anime = Animation(viewer);

%% animation

anime = anime.play(@snap,[]);
anime.save([],[]);

function snap(viewer,t)
    viewer.plotPosition(t,[],false);
end

%{
figure
F = [];
for t = 1:Nt
    if strcmp(get(gcf,'currentcharacter'),'q')  % key stop
        break;
    end
    
    viewer.plotPosition(t,[],false);
    hold off
    drawnow;
    F = [F, getframe];%軸含めない
end
%}

%% save animation
%{
is_save = true;
video_name = 'GFT_order.mp4';
if is_save
    v = VideoWriter(video_name,'MPEG-4');
    open(v);
    writeVideo(v,F);
    close(v);
end
%}