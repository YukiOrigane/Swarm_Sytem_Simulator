
%addpath(genpath('C:\Users\yuori\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));
addpath(genpath('C:\Users\origa\OneDrive - m.titech.ac.jp\05_simulation\Swarm_Sytem_Simulator'));

Nt = 10000;          %
dt = 10^-3;          % シミュレーション刻み時間
t_vec = 0:dt:dt*Nt; % 時間ベクトル

N = 1;

field = Fields(repmat([0,400; 0, 400; -900, 200],1,1));

%swarm = EularAngle1stOrderSystem(N,6,Nt,dt);
swarm = QuatAngle1stOrderSystem(N,7,Nt,dt);
%swarm = swarm.setInitialCondition([0;0;0;0;-pi/2;0]); % theta_2だけは指定しないと特異
swarm = swarm.setInitialCondition([0;0;0;eul2quat([0 -pi/2 0],'ZYZ').']); % theta_2だけは指定しないと特異

controller = ChlamydomonasController(N,Nt,dt);
I_0 = 0.5; gamma_0 = 1.0; tau_0 = 320;
v_0 = 120; omega_c_0 = 4*pi; omega_a_0 = 2*pi;
controller = controller.setParam(I_0, v_0, omega_c_0, omega_a_0, gamma_0, tau_0);

for t = 1:Nt
    swarm = swarm.observe(t);
    controller = controller.calcInput(t,swarm);
    swarm = swarm.update(t,controller.u);
end

%% plot
figure
plot(t_vec, permute(controller.interior_sys.x(1,1,:),[3,1,2]))
xlabel("t[s]")
ylabel("I(t)")

figure
plot(t_vec, quat2eul(permute(swarm.x(1,4:7,:),[3,2,1]),'ZYX'))
xlabel("t[s]")
ylabel("Eular Angle")
legend("roll(Z)","pitch(Y)","yaw(X)")

figure
x = permute(swarm.x(:,:,:),[1,3,2]); % [エージェント数,時間,次元]
plot3(x(1,:,1),x(1,:,2),x(1,:,3))
grid on

figure
l = permute(repmat([0,0,-1],Nt+1,1),[3,2,1]); % エージェント番号,要素,時間
b = controller.interior_sys.x(1,9:11,:);
c = controller.interior_sys.x(1,6:8,:);
theta = acos(pagemtimes(c,'none',l,'transpose')./vecnorm(c,2,2)./vecnorm(l,2,2));
phi = acos(pagemtimes(b(1,1:2,:),'none',c(1,1:2,:),'transpose')./vecnorm(b(1,1:2,:),2,2)./vecnorm(c(1,1:2,:),2,2));
plot(t_vec, permute(theta,[3,1,2]));
hold on
plot(t_vec, permute(phi,[3,1,2]));
legend("\theta","\phi")
grid on
%{
figure
plot(t_vec, permute(x(1,:,1:3),[2,3,1]));
legend("x","x","z")

figure
plot(t_vec, permute(x(1,:,4:6),[2,3,1]));
legend("\theta_1","\theta_2","\theta_3")
%}
%% for anime
viewer = ChlamydomonasViewer(swarm,controller.interior_sys ,field);

%% animation setting
%animation_speed = 1;
anime = Animation(viewer);


%% animation

anime = anime.play(@snap,20);
%anime.save(["NP"],1);

function snap(viewer,t)
    f = gcf;
    f.Position = [200,200,400,800];
    viewer.ChlamydoPositionPlot(t);
    hold off
end
