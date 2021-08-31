
addpath(genpath('C:\Users\origa\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

%% ピカピカシミュレータからのデータフォーマット変更
dt = 0.02;
rv = 1.6;
Nsim_view = 1;  % 表示シミュレーション回
swarm = Agents(repeat_wrap_config.Nr, 2, repeat_Nt-1, dt);
swarm = swarm.setGraphProperties(1:2, rv, false);
swarm.x(:,1,1:repeat_Nt) = permute(repeat_memory_x(:,1:repeat_Nt,Nsim_view),[1,3,2]);
swarm.x(:,2,1:repeat_Nt) = permute(repeat_memory_y(:,1:repeat_Nt,Nsim_view),[1,3,2]);

%% 表示
field = Fields(repmat([-5,5],2,1));
viewer = PlaneBasicViewer(swarm, field);

figure
t = 100;
viewer.plotPosition(t,[],true);
viewer.basicTimePlotX(1);

%% アニメーション

anime = Animation(viewer);
anime = anime.play(@snap,5);
%anime.save([],[]);

function snap(viewer,t)
    viewer.plotPosition(t,[],true);
    colorbar
end