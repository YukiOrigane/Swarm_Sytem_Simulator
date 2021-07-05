
% second_COS_impulseが既に走っていることを前提とする
% Analyzerみたいなクラスを作るかは要検討

x = permute(swarm.sys_cos.x(:,1,:),[1,3,2]);
Phi = x-rem(Omega_0*t_vec,2*pi);
%%%%%%%% 予測
i = 1;


%%%%%%%% Phiの方
%% 震源地
subplot(1,2,1)
[p1,f1] = pspectrum(Phi(1,:),t_vec);
loglog(2*pi*f1,p1)
grid on
xlabel("角周波数[rad/s]")
ylabel("パワー")
title("Spectrum of \Phi_1")
%xlim([0,5])
subplot(1,2,2)
[p1,f1] = pspectrum(Phi(2,:),t_vec);
loglog(2*pi*f1,p1)
grid on
xlabel("角周波数[rad/s]")
ylabel("パワー")
title("Spectrum of \Phi_2")

%% 中心
subplot(1,2,1)
[p1,f1] = pspectrum(Phi(8,:),t_vec);
loglog(2*pi*f1,p1)
grid on
xlabel("角周波数[rad/s]")
ylabel("パワー")
title("Spectrum of \Phi_5")
%xlim([0,5])
subplot(1,2,2)
[p1,f1] = pspectrum(Phi(9,:),t_vec);
loglog(2*pi*f1,p1)
grid on
xlabel("角周波数[rad/s]")
ylabel("パワー")
title("Spectrum of \Phi_6")

%% 時間プロット
plot(t_vec,Phi(1:3,:))
title("\Phi_i(t)")
legend("i=1","i=2","i=3")
grid on

%%%%%%%% xの方
%%
subplot(1,2,1)
pspectrum(x(1,:),t_vec)
title("Spectrum of \phi_1")
xlim([0,5])
subplot(1,2,2)
pspectrum(x(1,:)-x(11,:),t_vec)
title("Spectrum of \phi_1-\phi_{11}")
xlim([0,5])

%%
figure
hold on
pspectrum(x(1,:)-x(2,:),t_vec)
pspectrum(x(1,:)-x(11,:),t_vec)
pspectrum(x(1,:)-x(12,:),t_vec)
title("Spectrum of \phi_1-\phi_{j}")
%legend("j = 2","j = 11")
legend("j = 2","j = 11","j = 12")
xlim([0,5])
hold off

%%
figure
hold on
i=25;
pspectrum(x(i,:)-x(15,:),t_vec)
pspectrum(x(i,:)-x(24,:),t_vec)
pspectrum(x(i,:)-x(26,:),t_vec)
title("Spectrum of $\phi_{ij}$ ( i = "+string(i)+" )",'Interpreter','latex')
%legend("j = 2","j = 11")
legend("j = 15","j = 24","j = 26")
xlim([0,5])
hold off

%%
figure
hold on
i=25;
pspectrum(x(36,:)-x(16,:),t_vec)
pspectrum(x(27,:)-x(25,:),t_vec)
title("Spectrum of $\phi_{ij}$ ",'Interpreter','latex')
%legend("j = 2","j = 11")
legend("i=36,j=16","i=27,j=25")
xlim([0,5])
hold off

%%
figure
hold on
i=25;
pspectrum(x(25,:)-1/3*(x(16,:)+x(15,:)+x(14,:)),t_vec)
pspectrum(x(25,:)-1/3*(x(36,:)+x(35,:)+x(34,:)),t_vec)
title("Spectrum of $\phi_{ij}$ ",'Interpreter','latex')
%legend("j = 2","j = 11")
legend("i=25,j=av(16,15,14)","i=25,j=av(36,35,34)")
xlim([0,5])
hold off

%%
figure
hold on
i=25;
pspectrum(x(14,:)-x(39,:),t_vec)
pspectrum(x(14,:)-x(13,:),t_vec)
title("Spectrum of $\phi_{ij}$ ",'Interpreter','latex')
%legend("j = 2","j = 11")
legend("i=14,j=39","i=14,j=13")
xlim([0,5])
hold off