
y0 = zeros(20,1);
y0(1) = 0.1;
[t,y] = ode45(@func,[0 60],y0);

figure
plot(t,y(:,1:4));
grid on
xlabel("time (s)")
ylabel("x")
legend("1","2","3","4")
ax = gca;
ax.FontSize = 11;

figure
[p1,f1] = pspectrum(y(:,1),t);
loglog(2*pi*f1,p1)
grid on
xlabel("角周波数[rad/s]")
ylabel("パワー")
title("Spectrum of \Phi_1")
%{
sys = CoupledOscillator(10,2,length(t),t(2)-t(1));
sys.x(:,1,:) = permute(y(:,1:10),[2,3,1]);
sys.x(:,2,:) = permute(y(:,11:20),[2,3,1]);
viewer = CoupledOscillatorViewer(sys,[]);
anime = Animation(viewer);
anime = anime.play(@snap,1);
%}

function dydt = func(t,y)
    L = [1,-1,0,0,0,0,0,0,0,0;-1,2,-1,0,0,0,0,0,0,0;0,-1,2,-1,0,0,0,0,0,0;0,0,-1,2,-1,0,0,0,0,0;0,0,0,-1,2,-1,0,0,0,0;0,0,0,0,-1,2,-1,0,0,0;0,0,0,0,0,-1,2,-1,0,0;0,0,0,0,0,0,-1,2,-1,0;0,0,0,0,0,0,0,-1,2,-1;0,0,0,0,0,0,0,0,-1,1];
    gamma = 0.;
    kappa = 1;
    N = 10;
    f = zeros(2*N,1);
    if 0.01<t&&t<0.03
        %f(11) = 1;
    end
    dydt = [zeros(N,N),eye(N,N); -kappa*L, -gamma*eye(N,N)]*y+f;
end

function snap(viewer,t)
    plot(1:10,viewer.sys.x(1:10,1,t)-mean(viewer.sys.x(1:10,1,t)));
end